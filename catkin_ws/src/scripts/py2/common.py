#!/usr/bin/env python

# Python
import threading
import copy
import numpy as np
import math

# ROS
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist, Pose, PoseStamped, PoseWithCovarianceStamped
import rospy
from autoware_config_msgs.msg import ConfigNDT, ConfigVoxelGridFilter
from autoware_msgs.msg import Lane
from messages.srv import initialize_pose
from std_msgs.msg import Header

class XYZRPY:

  def __init__(self, x, y, z, roll, pitch, yaw):
    self.x = x
    self.y = y
    self.z = z
    self.roll = roll
    self.pitch = pitch
    self.yaw = yaw

class LockedObj:

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

def normalize_angle_pi2pi(angle):

  tmp_angle = angle
  while True:

    if (tmp_angle < -math.pi):
      tmp_angle = tmp_angle + math.pi 
    elif(math.pi <= tmp_angle):
      tmp_angle = tmp_angle - math.pi
    else:
      break

  return tmp_angle
      
def pose_is_almost_same(pose_st1, pose_st2, th_trans_diff, th_yaw_diff):

  np_pose1 = np.array( \
    [pose_st1.pose.position.x, pose_st1.pose.position.y, pose_st1.pose.position.z], \
    dtype=np.float64)
  np_pose2 = np.array( \
    [pose_st2.pose.position.x, pose_st2.pose.position.y, pose_st2.pose.position.z], \
    dtype=np.float64)

  diff_norm = np.linalg.norm(np_pose1 - np_pose2)
  if (th_trans_diff < diff_norm):
    return False

  np_q1 = np.array( \
    [pose_st1.pose.orientation.x, pose_st1.pose.orientation.y, \
    pose_st1.pose.orientation.z, pose_st1.pose.orientation.w], dtype=np.float64)
  np_q2 = np.array( \
    [pose_st2.pose.orientation.x, pose_st2.pose.orientation.y, \
    pose_st2.pose.orientation.z, pose_st2.pose.orientation.w], dtype=np.float64)

  np_eul1 = euler_from_quaternion(np_q1)
  np_eul2 = euler_from_quaternion(np_q2)

  # X. Yaw angle check.
  n_yaw1 = normalize_angle_pi2pi(np_eul1[2])
  n_yaw2 = normalize_angle_pi2pi(np_eul2[2])

  yaw_diff = abs(normalize_angle_pi2pi(n_yaw1 - n_yaw2))
  if (th_yaw_diff < yaw_diff):
    return False

  return True

def create_twist(vx, vy, vz, wx, wy, wz):

  twist = Twist()
  twist.linear.x = vx
  twist.linear.y = vy
  twist.linear.z = vz

  twist.angular.x = wx
  twist.angular.y = wy
  twist.angular.z = wz

  return twist

def call_pose_initialize_service(req):

  rospy.wait_for_service('/initialize_pose')
  try:
    init_pose_srv = rospy.ServiceProxy('/initialize_pose', initialize_pose)
    resp = init_pose_srv(**req)
  except rospy.ServiceException, e:
    rospy.logwarn("Service call failed: {}".format(e))

  return XYZRPY(resp.x, resp.y, resp.z, resp.roll, resp.pitch, resp.yaw)

def FULL_INIT_REQUEST(xyzrpy):
  return \
  {'resolution':1.0, 'step_size': 0.1, 'outlier_ratio': 0.55, 'trans_eps': 0.01, \
   'max_itr': 100, 'x_range': 10, 'y_range': 10, 'yaw_range': 2.0 * math.pi, \
   'x_step': 1.0, 'y_step': 1.0, 'yaw_step': 20.0 / 180.0 * math.pi, \
   'x': xyzrpy.x, 'y': xyzrpy.y, 'z': xyzrpy.z, 'roll': xyzrpy.roll, 'pitch': xyzrpy.pitch, 'yaw': xyzrpy.yaw}

def DEFAULT_CONFIG_NDT(xyzrpy):

  cfg = ConfigNDT()
  cfg.header = Header()
  
  # X. Pose
  cfg.x = xyzrpy.x
  cfg.y = xyzrpy.y
  cfg.z = xyzrpy.z
  cfg.roll = xyzrpy.roll
  cfg.pitch = xyzrpy.pitch
  cfg.yaw = xyzrpy.yaw

  #
  cfg.init_pos_gnss = 0
  cfg.use_predict_pose = 0
  cfg.error_threshold = 0.0
  cfg.resolution = 1.0
  cfg.step_size = 0.1
  cfg.trans_epsilon = 0.01
  cfg.max_iterations = 30

  return cfg

def CONFIG_VOXEL_FILT_SEARCH():
  cfg_vox_filt = ConfigVoxelGridFilter()  
  cfg_vox_filt.voxel_leaf_size = 2.0
  cfg_vox_filt.measurement_range = 80.0
  return cfg_vox_filt

def CONFIG_VOXEL_FILT_RUN():
  cfg_vox_filt = ConfigVoxelGridFilter()  
  cfg_vox_filt.voxel_leaf_size = 0.5
  cfg_vox_filt.measurement_range = 80.0
  return cfg_vox_filt

def create_pose_stamped_with_cov(pose_xyzrpy):

  pose_cov = PoseWithCovarianceStamped()
  pose_cov.header.stamp = rospy.Time.now()
  pose_cov.header.frame_id = "map"

  pose_cov.pose.pose.position.x = pose_xyzrpy.x
  pose_cov.pose.pose.position.y = pose_xyzrpy.y
  pose_cov.pose.pose.position.z = pose_xyzrpy.z

  q = quaternion_from_euler(pose_xyzrpy.roll, pose_xyzrpy.pitch, pose_xyzrpy.yaw)
  pose_cov.pose.pose.orientation.x = q[0]
  pose_cov.pose.pose.orientation.y = q[1]
  pose_cov.pose.pose.orientation.z = q[2]
  pose_cov.pose.pose.orientation.w = q[3]

  return pose_cov

def create_xyzrpy_from_pose(pose):
  q = np.array( \
    [pose.orientation.x, pose.orientation.y, \
     pose.orientation.z, pose.orientation.w], dtype=np.float64)
  eulers = euler_from_quaternion(q)
  return XYZRPY(pose.position.x, pose.position.y, pose.position.z, eulers[0], eulers[1], eulers[2])

def pose_is_not_nan(pose):

  if (not np.isnan(pose.position.x) and \
      not np.isnan(pose.position.y) and \
      not np.isnan(pose.position.z) and \
      not np.isnan(pose.orientation.x) and \
      not np.isnan(pose.orientation.y) and \
      not np.isnan(pose.orientation.z) and \
      not np.isnan(pose.orientation.w)):
    return True

  return False

def twist_is_almost_zero(twist, v_th, w_th):
  v = np.array([twist.linear.x, twist.linear.y, twist.linear.z], dtype=np.float64)
  w = np.array([twist.angular.x, twist.angular.y, twist.angular.z], dtype=np.float64)

  if (np.linalg.norm(v) < v_th and np.linalg.norm(w) < w_th):
    return True

  return False

def compute_distance_to_obstacle_on_waypoint(obs_idx, lane, pose):

  wp = lane.waypoints[obs_idx].pose.pose
  np_wp = np.array([wp.position.x, wp.position.y, wp.position.z])
  np_pose = np.array([pose.position.x, pose.position.y, pose.position.z])

  return np.linalg.norm(np_pose - np_wp)

def is_avoidance_ok_waypoint(obs_idx, lane):
  wp = lane.waypoints[obs_idx]
  return wp.change_flag == 1
