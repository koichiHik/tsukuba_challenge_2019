#!/usr/bin/env python

import argparse
import time
import math
import threading
import copy
import numpy as np

# ROS
import rospy
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from autoware_config_msgs.msg import ConfigNDT, ConfigVoxelGridFilter
from autoware_msgs.msg import NDTStat
import tf
from tf.transformations import quaternion_from_euler, decompose_matrix, quaternion_matrix
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, TwistStamped, Twist
from messages.srv import initialize_pose


def create_config_ndt(pose_xyzrpy):

  cfg = ConfigNDT()
  cfg.header = Header()
  
  # X. Pose
  cfg.x = pose_xyzrpy[0]
  cfg.y = pose_xyzrpy[1]
  cfg.z = pose_xyzrpy[2]
  cfg.roll = pose_xyzrpy[3]
  cfg.pitch = pose_xyzrpy[4]
  cfg.yaw = pose_xyzrpy[5]

  #
  cfg.init_pos_gnss = 0
  cfg.use_predict_pose = 0
  cfg.error_threshold = 0.0
  cfg.resolution = 1.0
  cfg.step_size = 0.1
  cfg.trans_epsilon = 0.01
  cfg.max_iterations = 30

  return cfg

def create_pose_stamped_with_cov(pose_xyzrpy):

  pose_cov = PoseWithCovarianceStamped()
  pose_cov.header.stamp = rospy.Time.now()
  pose_cov.header.frame_id = "map"

  pose_cov.pose.pose.position.x = pose_xyzrpy[0]
  pose_cov.pose.pose.position.y = pose_xyzrpy[1]
  pose_cov.pose.pose.position.z = pose_xyzrpy[2]

  q = quaternion_from_euler(pose_xyzrpy[3], pose_xyzrpy[4], pose_xyzrpy[5])
  pose_cov.pose.pose.orientation.x = q[0]
  pose_cov.pose.pose.orientation.y = q[1]
  pose_cov.pose.pose.orientation.z = q[2]
  pose_cov.pose.pose.orientation.w = q[3]

  return pose_cov

class LockedObj():

  def __init__(self, obj = None):
    self.lock = threading.Lock()
    self.__obj = obj

  def get_object(self):

    self.lock.acquire()
    try:
      obj = copy.deepcopy(self.__obj)
    finally:
      self.lock.release()

    return obj

  def set_object(self, obj):

    self.lock.acquire()
    try:
      self.__obj = copy.deepcopy(obj)
    finally:
      self.lock.release()

def FULL_INIT_REQUEST(x, y, z, roll, pitch, yaw):
  return \
  {'resolution':1.0, 'step_size': 0.1, 'outlier_ratio': 0.55, 'trans_eps': 0.01, \
   'max_itr': 100, 'x_range': 10, 'y_range': 10, 'yaw_range': 2.0 * math.pi, \
   'x_step': 1.0, 'y_step': 1.0, 'yaw_step': 20.0 / 180.0 * math.pi, \
   'x': x, 'y': y, 'z': z, 'roll': roll, 'pitch': pitch, 'yaw': yaw}

class LocalizeManager():

  def __init__(self, pose_init, init_via_gnss, x, y, z, roll, pitch, yaw):

    rospy.init_node('localize_manager', anonymous=True)
    self.cfg_ndt_pub = rospy.Publisher('/config/ndt', ConfigNDT, queue_size=10, latch=True)
    self.cfg_pnts_downsample = rospy.Publisher('config/voxel_grid_filter', ConfigVoxelGridFilter, queue_size=10, latch=True)
    self.fused_pose_pub = rospy.Publisher('fused_pose', PoseStamped, queue_size=10)
    self.current_vel_pub = rospy.Publisher('current_velocity', TwistStamped, queue_size=10)
    self.current_pose_pub = rospy.Publisher('current_pose', PoseStamped, queue_size=10)
    self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    self.mcl3dl_init_pose_pub = rospy.Publisher('/mcl_3dl/initialpose', PoseWithCovarianceStamped, queue_size=10, latch=True)

    # X. Update voxel filter config.
    self.update_voxel_grid_filter_config(2.0, 80.0)

    # X. Prepare locked object.
    self.locked_odom = LockedObj()
    self.locked_ndt_pose = LockedObj()
    self.locked_gnss_pose_xyzrpy = LockedObj([0.0] * 6)

    # X. Init Pose Candidate
    init_pose_xyzrpy = self.create_init_pose_xyzrpy(pose_init, init_via_gnss, list([x, y, z, roll, pitch, yaw]))

    # X. Update voxel filter config.
    self.update_voxel_grid_filter_config(0.5, 80.0)

    # X. Start NDT Localization.
    self.publish_config_ndt(init_pose_xyzrpy)

    # X. Start MCL3D Localization.
    self.publish_initial_pose_for_mcl3d(init_pose_xyzrpy)

    # X. Validity
    self.pose_valid = True

    rospy.Subscriber('odom', Odometry, self.odom_callback)
    rospy.Subscriber('ndt_stat', NDTStat, self.ndt_stat_callback)
    rospy.Subscriber('ndt_pose', PoseStamped, self.ndt_pose_callback)
    rospy.Subscriber('twist_raw', TwistStamped, self.twist_raw_callback)

    rospy.spin()

  def create_init_pose_xyzrpy(self, pose_init, init_via_gnss, pose_xyzrpy):

    # X. Init Pose Candidate
    init_pose_xyzrpy = pose_xyzrpy

    self.gnss_received  = False
    rospy.Subscriber('gnss_pose_local', PoseStamped, self.gnss_pose_local_callback) 
    
    # X. Use gnss for initialize.
    if (init_via_gnss):
      # X. Wait update from gnss
      while (not self.gnss_received and not rospy.is_shutdown()):
        rospy.logwarn('[Manager] Wait gnss data to be received.')
        time.sleep(0.5)
      init_pose_xyzrpy = self.locked_gnss_pose_xyzrpy.get_object()

    # X. Call pose initializer.
    if (pose_init):
      rospy.logwarn('[Manager] Init pose request {}'.format(init_pose_xyzrpy))
      req = FULL_INIT_REQUEST(init_pose_xyzrpy[0], init_pose_xyzrpy[1], init_pose_xyzrpy[2], \
                              init_pose_xyzrpy[3], init_pose_xyzrpy[4], init_pose_xyzrpy[5])
      init_pose_xyzrpy = self.call_pose_initialize_service(req)

    return init_pose_xyzrpy

  def update_voxel_grid_filter_config(self, leaf_size, measurement_range):
    cfg_vox_filt = ConfigVoxelGridFilter()
    cfg_vox_filt.voxel_leaf_size = leaf_size
    cfg_vox_filt.measurement_range = measurement_range
    self.cfg_pnts_downsample.publish(cfg_vox_filt)

  def publish_config_ndt(self, pose_xyzrpy):
    
    # X. Start NDT Localization.
    config_ndt = create_config_ndt(pose_xyzrpy)
    self.cfg_ndt_pub.publish(config_ndt)

  def publish_initial_pose_for_mcl3d(self, pose_xyzrpy):
    pose_stamped = create_pose_stamped_with_cov(pose_xyzrpy)
    self.mcl3dl_init_pose_pub.publish(pose_stamped)

  def call_pose_initialize_service(self, req):

    rospy.wait_for_service('/initialize_pose')
    try:
      init_pose_srv = rospy.ServiceProxy('/initialize_pose', initialize_pose)
      resp = init_pose_srv(**req)
    except rospy.ServiceException, e:
      rospy.logwarn("Service call failed: {}".format(e))

    return list([resp.x, resp.y, resp.z, resp.roll, resp.pitch, resp.yaw])

  def twist_raw_callback(self, twist_stamped):
    self.cmd_vel_pub.publish(twist_stamped.twist)

  def ndt_stat_callback(self, ndt_stat):
    if (0.5 < ndt_stat.score):
      rospy.logwarn('High ndt score : {}'.format(ndt_stat.score))

  def odom_callback(self, odom):

    self.locked_odom.set_object(odom)
    twist_stamped = TwistStamped()
    twist_stamped.header = odom.header
    twist_stamped.twist = odom.twist.twist
    self.current_vel_pub.publish(twist_stamped)

    if (self.pose_valid):
      ndt_pose = self.locked_ndt_pose.get_object()
      if (ndt_pose is not None):
        self.current_pose_pub.publish(ndt_pose)

  def ndt_pose_callback(self, ndt_pose):
    self.locked_ndt_pose.set_object(ndt_pose)

  def gnss_pose_local_callback(self, pose):

    gnss_pose = self.locked_gnss_pose_xyzrpy.get_object()
    if (not np.isnan(pose.pose.position.x) and 
        not np.isnan(pose.pose.position.y) and 
        not np.isnan(pose.pose.position.z)):
      gnss_pose[0] = pose.pose.position.x
      gnss_pose[1] = pose.pose.position.y
      gnss_pose[2] = pose.pose.position.z
      gnss_pose[3] = 0
      gnss_pose[4] = 0    
      gnss_pose[5] = 0

    self.locked_gnss_pose_xyzrpy.set_object(gnss_pose)
    self.gnss_received = True

if __name__ == '__main__':

  parser = argparse.ArgumentParser()
  parser.add_argument('-pose_initializer', type=int)
  parser.add_argument('-init_via_gnss', type=int)
  parser.add_argument('-x', type=float)
  parser.add_argument('-y', type=float)
  parser.add_argument('-z', type=float)
  parser.add_argument('-roll', type=float)
  parser.add_argument('-pitch', type=float)
  parser.add_argument('-yaw', type=float)
  args = parser.parse_args(rospy.myargv()[1:])

  mgr = LocalizeManager(args.pose_initializer, args.init_via_gnss, args.x, args.y, args.z, args.roll, args.pitch, args.yaw)
