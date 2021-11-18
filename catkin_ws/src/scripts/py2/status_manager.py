#!/usr/bin/env python

# Python STL
import time
import math
import threading
import collections

# ROS
import rospy
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, \
  Twist, TwistStamped
from autoware_config_msgs.msg import ConfigNDT, ConfigVoxelGridFilter
from autoware_msgs.msg import NDTStat, Lane
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32, Header
import tf
import tf2_geometry_msgs

# Original
from text_to_speech import TextToSpeech
from common import LockedObj, pose_is_almost_same, create_twist, \
  XYZRPY, call_pose_initialize_service, DEFAULT_CONFIG_NDT, CONFIG_VOXEL_FILT_SEARCH, \
  CONFIG_VOXEL_FILT_RUN, create_pose_stamped_with_cov, pose_is_not_nan, create_xyzrpy_from_pose, \
  FULL_INIT_REQUEST, twist_is_almost_zero, is_avoidance_ok_waypoint, compute_distance_to_obstacle_on_waypoint, \
  transform_pose, create_pose_from_xyzrpy, create_transform_from_pose, create_tf_transform_from_pose

STATUS_CHECK_HZ = 1
CONTROL_LOOP_HZ = 20

STANDSTILL_VX_THR = 0.01
STANDSTILL_WX_THR = 0.01

LOCALIZATION_TRANS_THRESH = 2.0
LOCALIZATION_YAW_THRESH = 45.0 / 180.0 * math.pi
LOCALIZATION_CNT_REINIT = CONTROL_LOOP_HZ * 2

LOCALIZATION_RELIABLE_CNT = 60
LOCALIZATION_TRANS_RELIABLE_THRESH = 0.75
LOCALIZATION_YAW_RELIABLE_THRESH = 15 / 180.0 * math.pi
LOCALIZATION_REINIT_MINIMUM_PERIOD = 15
LOCALIZATION_ODOM_TRUST_PERIOD = 60

GNSS_INIT_CNT = 10

OBSTACLE_DIST = 2.0
WAIT_BEFORE_AVOID_OBST_SEC = 5

class Subscribers:

  def __init__(self, announcer):

    self.announcer = announcer

    # Localization related.
    self.gnss_pose = LockedObj()
    self.ndt_stat = LockedObj()
    self.ndt_pose = LockedObj()
    self.odom = LockedObj()
    self.amcl_pose = LockedObj()
    self.twist_raw = LockedObj()

    self.gnss_local_sub = rospy.Subscriber('gnss_pose_local', PoseStamped, self.gnss_pose_local_callback) 
    self.ndt_stat_sub = rospy.Subscriber('ndt_stat', NDTStat, self.ndt_stat_callback)
    self.ndt_pose_sub = rospy.Subscriber('ndt_pose', PoseStamped, self.ndt_pose_callback)
    self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
    self.amcl_pose_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.amcl_pose_callback)
    self.twist_raw_sub = rospy.Subscriber('twist_raw', TwistStamped, self.twist_raw_callback)
    self.base_waypnts_sub = rospy.Subscriber('/base_waypoints', Lane, self.base_wps_callback)
    self.cur_wp_idx_sub = rospy.Subscriber('/closest_waypoint', Int32, self.cur_wp_callback)

    # Status subscribe
    self.avoidance_done_sub = rospy.Subscriber('avoidance_request_done', Header, self.avoidance_done_callback)

    self.gnss_call_cnt = 0

    # Object detection related.
    self.final_wps = LockedObj()
    self.obst_idx = LockedObj()

    self.final_wps_sub = rospy.Subscriber('final_waypoints', Lane, self.final_wps_callback)
    self.obst_wp_idx_sub = rospy.Subscriber('obstacle_waypoint', Int32, self.obst_wp_idx_callback)

    self.cur_twist = Twist()
    self.base_wps = Lane()
    self.goal_speech_cnt = 0


  def gnss_pose_local_callback(self, pose):
    if (GNSS_INIT_CNT < self.gnss_call_cnt and pose_is_not_nan(pose.pose)):
      self.gnss_pose.set_object(pose)
    elif(self.gnss_call_cnt <= GNSS_INIT_CNT and pose_is_not_nan(pose.pose)):
      self.gnss_call_cnt += 1

  def ndt_stat_callback(self, ndt_stat):
    if (0.5 < ndt_stat.score):
      rospy.logwarn('High ndt score : {}'.format(ndt_stat.score))
    self.ndt_stat.set_object(ndt_stat)

  def ndt_pose_callback(self, ndt_pose):
    self.ndt_pose.set_object(ndt_pose)

  def odom_callback(self, odom):
    self.odom.set_object(odom)
    self.twist = odom.twist.twist

  def amcl_pose_callback(self, amcl_pose):
    self.amcl_pose.set_object(amcl_pose)

  def twist_raw_callback(self, twist_stamped):
    self.twist_raw.set_object(twist_stamped)

  def final_wps_callback(self, lane):
    self.final_wps.set_object(lane)

  def obst_wp_idx_callback(self, idx):
    self.obst_idx.set_object(idx)

  def avoidance_done_callback(self, header):
    self.announcer.add_speech_text("Avoidance done. Back to waypoint following mode.")

  def base_wps_callback(self, base_wps):
    self.base_wps = base_wps

  def cur_wp_callback(self, idx):
    cur_wp_idx = idx.data

    if (self.base_wps is not None and len(self.base_wps.waypoints) - cur_wp_idx < 3 \
          and twist_is_almost_zero(self.twist, STANDSTILL_VX_THR, STANDSTILL_WX_THR)):
      if (self.goal_speech_cnt % 50 == 0):
        self.announcer.add_speech_text("Reached stop waypoint. Please provide next waypoints to follow.")
        self.goal_speech_cnt = 0

      self.goal_speech_cnt += 1

class Publishers:

  def __init__(self):
    # Control 
    self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    self.cur_pose = rospy.Publisher('current_pose', PoseStamped, queue_size=10)
    self.cur_vel = rospy.Publisher('current_velocity', TwistStamped, queue_size=10)

    # Config
    self.cfg_ntd = rospy.Publisher('/config/ndt', ConfigNDT, queue_size=10, latch=True)
    self.cfg_voxel = rospy.Publisher('config/voxel_grid_filter', ConfigVoxelGridFilter, queue_size=10, latch=True)
    self.amcl_init = rospy.Publisher('/mcl_3dl/initialpose', PoseWithCovarianceStamped, queue_size=10, latch=True)
    self.astar_avoid = rospy.Publisher('onetime_avoidance_request', Header, queue_size=10, latch=True)

class SharedData:

  def __init__(self):
    self.localization_not_reliable_cnt = 0
    self.stopped_due_to_obstable_cnt = 0
    self.pose_initializing = False
    self.last_pose_init_time = None
    self.last_cur_pose = LockedObj()
    self.last_valid_ndt_pose = None
    self.last_valid_tf_odom_to_map = None
    self.last_valid_ndt_pose_queue = collections.deque(maxlen=LOCALIZATION_RELIABLE_CNT)
    self.standstill_count = 0
    self.ndt_pose_unreliable_cnt = 0

class StatusManagerConfig:

  def __init__(self, pose_init, via_gnss, xyzrpy):
    self.pose_init = pose_init
    self.via_gnss = via_gnss
    self.xyzrpy = xyzrpy

class StatusManager:

  def __init__(self, config):

    # X. Initialize announcer.
    self.announcer = TextToSpeech()
    self.announcer.add_speech_text("Initializing system. Please wait for a moment.")

    # X. Tf
    self.broadcaster = tf.TransformBroadcaster()
    self.listener = tf.TransformListener()

    # Prepare shared object.
    self.data = SharedData()

    # Publishers & Subscribers
    self.pubs = Publishers()
    self.subs = Subscribers(self.announcer)

    # Store tf
    if (not config.pose_init):
      while (self.subs.odom.get_object() is None):
        time.sleep(0.5)
      odom = self.subs.odom.get_object()
      odom_xyzrpy = create_xyzrpy_from_pose(odom.pose.pose)
      pose_stamped = PoseStamped()
      pose_stamped.header.stamp = rospy.Time.now()
      pose_stamped.header.frame_id = 'map'
      pose_stamped.pose = create_pose_from_xyzrpy(config.xyzrpy)
      self.data.last_valid_ndt_pose = pose_stamped
      self.data.last_valid_tf_odom_to_map = create_transform_from_pose(config.xyzrpy, odom_xyzrpy)

    # Initilize Pose
    self.initialize_pose(config)

    # Start checking thread
    self.checking_thread = threading.Thread(target=self.run_checking)
    self.checking_thread.start()

    # Start control
    self.run_control()

    # Terminate thread.
    self.announcer.terminate()
    self.checking_thread.join()

  def initialize_pose(self, config):

    init_xyzrpy = config.xyzrpy
    if (config.via_gnss):
      # X. Wait update from gnss.
      while (self.subs.gnss_pose.get_object() is None and not rospy.is_shutdown()):
        rospy.logwarn("Waiting for GNSS data to be received.")
        time.sleep(0.5)

      init_xyzrpy = create_xyzrpy_from_pose(self.subs.gnss_pose.get_object().pose)
      init_xyzrpy.roll = 0
      init_xyzrpy.pitch = 0
      init_xyzrpy.yaw = 0

    # X. Start pose initializer if necessary.
    if (config.pose_init):
      self.announcer.add_speech_text("Start pose initializer. This will take about twenty seconds.")
      # X. Switch to low resolution.
      self.pubs.cfg_voxel.publish(CONFIG_VOXEL_FILT_SEARCH())
      req = FULL_INIT_REQUEST(init_xyzrpy)
      init_xyzrpy = call_pose_initialize_service(req)

    # X. Set high resolution mode.
    self.pubs.cfg_voxel.publish(CONFIG_VOXEL_FILT_RUN())

    # X. Send for ndt localizer.
    self.pubs.cfg_ntd.publish(DEFAULT_CONFIG_NDT(init_xyzrpy))

    # X. Send for amcl localizer.
    rospy.logwarn("amcl : {}".format(create_pose_stamped_with_cov(init_xyzrpy)))
    self.pubs.amcl_init.publish(create_pose_stamped_with_cov(init_xyzrpy))

    # X. Notify
    self.announcer.add_speech_text("Pose initialization is done. Start control.")

  def run_pose_initialize_if_necessary(self):

    if (self.data.last_pose_init_time is None \
        or rospy.Time.now() - self.data.last_pose_init_time > rospy.Duration(LOCALIZATION_REINIT_MINIMUM_PERIOD)):

      if (LOCALIZATION_CNT_REINIT <= self.data.localization_not_reliable_cnt):
        self.data.pose_initializing = True

        INIT_POSE_STANDSTILL_COUNT = 10
        wait_cnt = 0
        while (self.data.standstill_count < INIT_POSE_STANDSTILL_COUNT):
          wait_cnt += 1
          if (wait_cnt % 3):
            self.announcer.add_speech_text("Wait robot to stop.")  
          time.sleep(1.0)

        if (self.data.last_valid_ndt_pose is not None \
            and rospy.Time.now() - self.data.last_valid_ndt_pose.header.stamp < rospy.Duration(LOCALIZATION_ODOM_TRUST_PERIOD)):
          self.announcer.add_speech_text("Reinitialize localization based on odometry. Robot will stop.")
          config = StatusManagerConfig(True, False, create_xyzrpy_from_pose(self.data.last_cur_pose.get_object().pose))
        else:
          self.announcer.add_speech_text("Reinitialize localization based on GNSS. Robot will stop.")
          config = StatusManagerConfig(True, True, XYZRPY(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        
        # X. This function takes time.
        self.initialize_pose(config)
        self.data.last_pose_init_time = rospy.Time.now()  

        self.data.pose_initializing = False
        self.data.localization_not_reliable_cnt \
            = max(0, self.data.localization_not_reliable_cnt - LOCALIZATION_CNT_REINIT / 2)

  def check_robot_stop_reason(self):

    obs_idx = self.subs.obst_idx.get_object()
    if (obs_idx is None or obs_idx.data == -1):
      self.data.stopped_due_to_obstable_cnt = 0
      return

    twist_raw = self.subs.twist_raw.get_object()
    if (twist_raw is None or \
        not twist_is_almost_zero(twist_raw.twist, STANDSTILL_VX_THR, STANDSTILL_WX_THR)):
      self.data.stopped_due_to_obstable_cnt = 0
      return

    fin_wps = self.subs.final_wps.get_object()
    #ndt_pose = self.subs.ndt_pose.get_object()
    last_cur_pose = self.data.last_cur_pose.get_object()
    if (fin_wps is None or last_cur_pose is None):
      self.data.stopped_due_to_obstable_cnt = 0
      return

    print("Distance to obstacle : {}".format(compute_distance_to_obstacle_on_waypoint(obs_idx.data, fin_wps, last_cur_pose.pose)))
    if (compute_distance_to_obstacle_on_waypoint(obs_idx.data, fin_wps, last_cur_pose.pose) < OBSTACLE_DIST):
      if (self.data.stopped_due_to_obstable_cnt % 3 == 0):
        self.announcer.add_speech_text("Robot stops due to obstacle in front.")
      self.data.stopped_due_to_obstable_cnt += 1

      if (WAIT_BEFORE_AVOID_OBST_SEC < \
        self.data.stopped_due_to_obstable_cnt / STATUS_CHECK_HZ):

        if (is_avoidance_ok_waypoint(obs_idx.data, fin_wps)):
          self.announcer.add_speech_text("Rerouting for obstacle avoidance.")
          header = Header()
          header.stamp = rospy.Time.now()
          self.pubs.astar_avoid.publish(header)
          self.data.stopped_due_to_obstable_cnt = 0
        else:
          self.announcer.add_speech_text("Rerouting is not allowed here. Keep waiting.")
          self.data.stopped_due_to_obstable_cnt = 0

  def run_checking(self):

    rate = rospy.Rate(STATUS_CHECK_HZ)
    while not rospy.is_shutdown():

      # X. Robot stuck reason.
      self.check_robot_stop_reason()

      # X. Localization status.
      self.run_pose_initialize_if_necessary()

      rate.sleep()

  def run_control(self):

    rate = rospy.Rate(CONTROL_LOOP_HZ)
    while not rospy.is_shutdown():

      cur_time = rospy.Time.now()

      # X. Control command.
      twist_st = self.subs.twist_raw.get_object()
      if (twist_st is not None and not self.data.pose_initializing):
        self.pubs.cmd_vel.publish(self.subs.twist_raw.get_object().twist)
      else:
        self.pubs.cmd_vel.publish(create_twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

      # X. Velocity routing.
      odom = self.subs.odom.get_object()
      if (odom is not None):
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = cur_time
        twist_stamped.twist = odom.twist.twist
        self.pubs.cur_vel.publish(twist_stamped)
        if (twist_is_almost_zero(odom.twist.twist, STANDSTILL_VX_THR, STANDSTILL_WX_THR)):
          self.data.standstill_count += 1
        else:
          self.data.standstill_count = 0


      # X. Check pose validity
      if (not self.data.pose_initializing):
        self.check_pose_validity()

      # X. Current pose
      self.publish_curpose(cur_time)

      rate.sleep()

  def check_pose_validity(self):

    # X. When map pose is unstable.
    ndt_pose = self.subs.ndt_pose.get_object()
    amcl_pose = self.subs.amcl_pose.get_object()
    if (ndt_pose is not None and amcl_pose is not None and \
        not pose_is_almost_same(ndt_pose, amcl_pose.pose, \
        LOCALIZATION_TRANS_THRESH, LOCALIZATION_YAW_THRESH)):

      self.data.localization_not_reliable_cnt = \
          min(self.data.localization_not_reliable_cnt + 1, LOCALIZATION_CNT_REINIT)

    # X. When map pose is stable.
    else:

      # X. Extract last valid pose.
      if (ndt_pose is not None and amcl_pose is not None and \
          pose_is_almost_same(ndt_pose, amcl_pose.pose, \
          LOCALIZATION_TRANS_RELIABLE_THRESH, LOCALIZATION_YAW_RELIABLE_THRESH)):

        try:
          transform = self.listener.lookupTransform('map', 'odom', ndt_pose.header.stamp)
          self.data.last_valid_ndt_pose_queue.append([ndt_pose, transform])
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
          pass

        # If reaches stable count, treat the first  one as valid pose.
        if (len(self.data.last_valid_ndt_pose_queue) == LOCALIZATION_RELIABLE_CNT):
          self.data.last_valid_ndt_pose = self.data.last_valid_ndt_pose_queue[0][0]
          self.data.last_valid_tf_odom_to_map = self.data.last_valid_ndt_pose_queue[0][1]

      else:
        self.data.last_valid_ndt_pose_queue.clear()

      self.data.localization_not_reliable_cnt \
          = max(0, self.data.localization_not_reliable_cnt - 1)  

  def publish_curpose(self, cur_time):

    odom = self.subs.odom.get_object()
    if (odom is not None):

      pose_stamped = PoseStamped()
      pose_stamped.header.stamp = cur_time
      pose_stamped.header.frame_id = "map"
      pose_st = self.subs.ndt_pose.get_object()
      if (pose_st is not None and \
          self.data.localization_not_reliable_cnt == 0 and \
          not self.data.pose_initializing):

        # X. Use ndt pose as cur_pose
        pose_stamped.pose = pose_st.pose
        transform = create_tf_transform_from_pose(pose_st.pose, odom.pose.pose)

        # X. Send tf
        self.broadcaster.sendTransform(transform[0], transform[1], cur_time, 'odom', 'map')
        self.pubs.cur_pose.publish(pose_stamped)
        self.data.last_cur_pose.set_object(pose_stamped)

        self.data.ndt_pose_unreliable_cnt = 0

      else:

        # X. Use odometry as cur_pose
        if (self.data.last_valid_tf_odom_to_map is not None):
          trans = self.data.last_valid_tf_odom_to_map[0]
          rot = self.data.last_valid_tf_odom_to_map[1]

          # X. Send tf
          self.broadcaster.sendTransform(trans, rot, cur_time, 'odom', 'map')

          trans_pose = transform_pose(odom.pose.pose, trans, rot)
          pose_stamped.pose = trans_pose
          self.pubs.cur_pose.publish(pose_stamped)
          self.data.last_cur_pose.set_object(pose_stamped)

          self.data.ndt_pose_unreliable_cnt += 1
          if (self.data.ndt_pose_unreliable_cnt % (5 * CONTROL_LOOP_HZ) == 0):
            self.announcer.add_speech_text("Ndt pose is unstable. ")


if __name__ == '__main__':

  rospy.init_node("status_manager", anonymous=True)

  pose_initializer = rospy.get_param('~pose_initializer')
  init_via_gnss = rospy.get_param('~init_via_gnss')
  x = rospy.get_param('~x')
  y = rospy.get_param('~y')
  z = rospy.get_param('~z')
  roll = rospy.get_param('~roll')
  pitch = rospy.get_param('~pitch')
  yaw = rospy.get_param('~yaw')

  xyzrpy = XYZRPY(x, y, z, roll, pitch, yaw)
  config = StatusManagerConfig(pose_initializer, init_via_gnss, xyzrpy)
  mgr = StatusManager(config)
