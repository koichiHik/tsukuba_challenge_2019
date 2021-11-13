#!/usr/bin/env python

# Python STL
import time
import math
import threading

# ROS
import rospy
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, \
  Twist, TwistStamped
from autoware_config_msgs.msg import ConfigNDT, ConfigVoxelGridFilter
from autoware_msgs.msg import NDTStat, Lane
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32, Header

# Original
from text_to_speech import TextToSpeech
from common import LockedObj, pose_is_almost_same, create_twist, \
  XYZRPY, call_pose_initialize_service, DEFAULT_CONFIG_NDT, CONFIG_VOXEL_FILT_SEARCH, \
  CONFIG_VOXEL_FILT_RUN, create_pose_stamped_with_cov, pose_is_not_nan, create_xyzrpy_from_pose, \
  FULL_INIT_REQUEST, twist_is_almost_zero, is_avoidance_ok_waypoint, compute_distance_to_obstacle_on_waypoint

LOCALIZATION_TRANS_THRESH = 2.0
LOCALIZATION_YAW_THRESH = 45.0 / 180.0 * math.pi
LOCALIZATION_CNT_REINIT = 3

GNSS_INIT_CNT = 10

WAIT_BEFORE_AVOID_OBST_SEC = 5
STATUS_CHECK_HZ = 1

class Subscribers:

  def __init__(self):

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

    self.gnss_call_cnt = 0

    # Object detection related.
    self.final_wps = LockedObj()
    self.obst_idx = LockedObj()

    self.final_wps_sub = rospy.Subscriber('final_waypoints', Lane, self.final_wps_callback)
    self.obst_wp_idx_sub = rospy.Subscriber('obstacle_waypoint', Int32, self.obst_wp_idx_callback)


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

  def amcl_pose_callback(self, amcl_pose):
    self.amcl_pose.set_object(amcl_pose)

  def twist_raw_callback(self, twist_stamped):
    self.twist_raw.set_object(twist_stamped)

  def final_wps_callback(self, lane):
    self.final_wps.set_object(lane)

  def obst_wp_idx_callback(self, idx):
    self.obst_idx.set_object(idx)

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
    self.stop_ctrl = False
    self.last_valid_pose = None

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

    # Prepare shared object.
    self.data = SharedData()

    # Publishers & Subscribers
    self.pubs = Publishers()
    self.subs = Subscribers()

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
      while (self.subs.gnss_pose.get_object() is None):
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
    self.pubs.amcl_init.publish(create_pose_stamped_with_cov(init_xyzrpy))

    # X. Notify
    self.announcer.add_speech_text("Pose initialization is done. Start control.")


  def check_localization_reliability(self):

    ndt_pose = self.subs.ndt_pose.get_object()
    amcl_pose = self.subs.amcl_pose.get_object()

    if (ndt_pose is not None and amcl_pose is not None and \
        not pose_is_almost_same(ndt_pose, amcl_pose.pose, 
        LOCALIZATION_TRANS_THRESH, LOCALIZATION_YAW_THRESH)):

      self.data.localization_not_reliable_cnt += 1
      if (self.data.localization_not_reliable_cnt % 3 == 0):
        self.announcer.add_speech_text("Localization reliability is low.")
      
      if (LOCALIZATION_CNT_REINIT <= self.data.localization_not_reliable_cnt):
        self.data.stop_ctrl = True
        self.announcer.add_speech_text( \
          "Reinitialize localization. Robot will stop.")
        if (self.data.last_valid_pose is not None):
          config = StatusManagerConfig(True, False, create_xyzrpy_from_pose(self.data.last_valid_pose.pose))
        else:
          config = StatusManagerConfig(True, True, XYZRPY(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        self.initialize_pose(config)        

        self.data.stop_ctrl = False
        self.data.localization_not_reliable_cnt = 0  

    else:
      self.data.last_valid_pose = ndt_pose
      self.data.localization_not_reliable_cnt = 0

  def check_robot_stop_reason(self):

    DIST_THRESH = 2.0
    VEL_THRESH = 0.01
    W_THRESH = 0.01

    obs_idx = self.subs.obst_idx.get_object()
    if (obs_idx is None or obs_idx.data == -1):
      self.data.stopped_due_to_obstable_cnt = 0
      return

    twist_raw = self.subs.twist_raw.get_object()
    if (twist_raw is None or \
        not twist_is_almost_zero(twist_raw.twist, VEL_THRESH, W_THRESH)):
      self.data.stopped_due_to_obstable_cnt = 0
      return

    fin_wps = self.subs.final_wps.get_object()
    ndt_pose = self.subs.ndt_pose.get_object()
    if (fin_wps is None or ndt_pose is None):
      self.data.stopped_due_to_obstable_cnt = 0
      return

    if (compute_distance_to_obstacle_on_waypoint(obs_idx.data, fin_wps, ndt_pose.pose) < DIST_THRESH):
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

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():

      # X. Robot stuck reason.
      self.check_robot_stop_reason()

      # X. Localization status.
      self.check_localization_reliability()

      rate.sleep()

  def run_control(self):

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():

      cur_time = rospy.Time.now()

      # X. Control command.
      twist_st = self.subs.twist_raw.get_object()
      if (twist_st is not None and not self.data.stop_ctrl):
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

      # X. Current pose 
      pose_st = self.subs.ndt_pose.get_object()
      if (pose_st is not None):
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = cur_time
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose = pose_st.pose
        self.pubs.cur_pose.publish(pose_stamped)

      rate.sleep()

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
