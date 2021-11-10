#!/usr/bin/env python

import argparse
import time
import math
import random
import threading
import copy
import numpy as np

# ROS
import rospy
from geometry_msgs.msg import Pose, PoseStamped
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


def create_config_ndt(x, y, z, roll, pitch, yaw):

  cfg = ConfigNDT()
  cfg.header = Header()
  
  # X. Pose
  cfg.x = x
  cfg.y = y
  cfg.z = z
  cfg.roll = roll
  cfg.pitch = pitch
  cfg.yaw = yaw

  #
  cfg.init_pos_gnss = 0
  cfg.use_predict_pose = 0
  cfg.error_threshold = 0.0
  cfg.resolution = 1.0
  cfg.step_size = 0.1
  cfg.trans_epsilon = 0.01
  cfg.max_iterations = 30

  return cfg

class LockedObj():

  def __init__(self):
    self.lock = threading.Lock()
    self.__obj = None

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

class LocalizeManager():

  def __init__(self, x, y, z, roll, pitch, yaw):

    rospy.init_node('localize_manager', anonymous=True)
    self.cfg_ndt_pub = rospy.Publisher('/config/ndt', ConfigNDT, queue_size=10, latch=True)
    self.cfg_pnts_downsample = rospy.Publisher('config/voxel_grid_filter', ConfigVoxelGridFilter, queue_size=10, latch=True)
    self.fused_pose_pub = rospy.Publisher('fused_pose', PoseStamped, queue_size=10)
    self.current_vel_pub = rospy.Publisher('current_velocity', TwistStamped, queue_size=10)
    self.current_pose_pub = rospy.Publisher('current_pose', PoseStamped, queue_size=10)
    self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # X. Voxel filter config.
    cfg_vox_filt = ConfigVoxelGridFilter()
    cfg_vox_filt.voxel_leaf_size = 2.0
    cfg_vox_filt.measurement_range = 40
    self.cfg_pnts_downsample.publish(cfg_vox_filt)

    # X.
    x, y, z, roll, pitch, yaw = self.call_pose_initialize_service()
    cur_pose = list([x, y, z, roll, pitch, yaw])
    self.locked_cur_pose = LockedObj()
    self.locked_cur_pose.set_object(cur_pose)
    config_ndt = create_config_ndt( \
      cur_pose[0], cur_pose[1], cur_pose[2], \
      cur_pose[3], cur_pose[4], cur_pose[5])
    self.cfg_ndt_pub.publish(config_ndt)
    random.seed(0)

    # X. Odom
    self.locked_odom = LockedObj()

    # X. Cur pose
    self.locked_pose = LockedObj()

    # X. Tf broadcaster
    self.br = tf.TransformBroadcaster()

    # X. Validity
    self.pose_valid = False

    rospy.Subscriber('odom', Odometry, self.odom_callback)
    rospy.Subscriber('ndt_stat', NDTStat, self.ndt_stat_callback)
    rospy.Subscriber('gnss_pose_local', PoseStamped, self.gnss_pose_local_callback)
    rospy.Subscriber('ndt_pose', PoseStamped, self.ndt_pose_callback)
    rospy.Subscriber('twist_raw', TwistStamped, self.twist_raw_callback)

    rospy.spin()

  def call_pose_initialize_service(self):

    x_range = 10
    y_range = 10
    yaw_range = 2 * math.pi
    x_step = 1.0
    y_step = 1.0
    yaw_step = 20.0 / 180.0 * math.pi
    x = 0.0
    y = 0.0
    z = 0.0
    roll = 0.0
    pitch = 0.0
    yaw = 0.0

    srv = {'resolution':1.0, 'step_size': 0.1, 'outlier_ratio': 0.55, 
           'trans_eps': 0.01, 'max_itr':10, 'x_range': x_range, 'y_range': y_range,
           'yaw_range': yaw_range, 'x_step': x_step, 'y_step': y_step, 'yaw_step': yaw_step, 
           'x': x, 'y': y, 'z': z, 'roll': roll, 'pitch': pitch, 'yaw': yaw}
    rospy.wait_for_service('/initialize_pose')
    try:
      init_pose_srv = rospy.ServiceProxy('/initialize_pose', initialize_pose)
      resp = init_pose_srv(**srv)
    except rospy.ServiceException, e:
      print("Service call failed: {}", e)

    print(resp)

    return resp.x, resp.y, resp.z, resp.roll, resp.pitch, resp.yaw


  def twist_raw_callback(self, twist_stamped):
    self.cmd_vel_pub.publish(twist_stamped.twist)

  def ndt_stat_callback(self, ndt_stat):

    pass

    if (0.5 < ndt_stat.score and not self.pose_valid):
      self.pose_valid = False
      cur_pose = self.locked_cur_pose.get_object()
      x, y, z, yaw = cur_pose[0], cur_pose[1], cur_pose[2], cur_pose[5]

      print('High ndt score : {}'.format(ndt_stat.score))
      print('x: {}, y: {}, z: {}'.format(x, y, z))

      dpose = self.dpose_list[random.randint(0, len(self.dpose_list))]
      config_ndt = create_config_ndt( \
        x + dpose[0], y + dpose[1], 0, \
        0, 0, yaw + dpose[2])
      self.cfg_ndt_pub.publish(config_ndt)
    elif(100.0 < ndt_stat.score):
      self.pose_valid = False
    else:
      self.pose_valid = True


  def odom_callback(self, odom):
    self.locked_odom.set_object(odom)
    twist_stamped = TwistStamped()
    twist_stamped.header = odom.header
    twist_stamped.twist = odom.twist.twist
    self.current_vel_pub.publish(twist_stamped)

    if (self.pose_valid):
      ndt_pose = self.locked_pose.get_object()
      if (ndt_pose is not None):
        self.current_pose_pub.publish(ndt_pose)


  def ndt_pose_callback(self, ndt_pose):
    print("Ndt Pose Received.\n")

    # X. Convert to matrix.
    odom = self.locked_odom.get_object()

    #odom = Odometry()
    #odom.pose.pose.position.x = 0.0
    #odom.pose.pose.position.y = 0.0
    #odom.pose.pose.position.z = 0.0
    #odom.pose.pose.orientation.x = 0.0
    #odom.pose.pose.orientation.y = 0.0
    #odom.pose.pose.orientation.z = 0.0
    #odom.pose.pose.orientation.w = 1.0

    T_odom = quaternion_matrix( \
      np.array([odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, \
                odom.pose.pose.orientation.z, odom.pose.pose.orientation.w], dtype=np.float64))
    T_odom[0,3] = odom.pose.pose.position.x
    T_odom[1,3] = odom.pose.pose.position.y
    T_odom[2,3] = odom.pose.pose.position.z

    T_map = quaternion_matrix( \
      np.array([ndt_pose.pose.orientation.x, ndt_pose.pose.orientation.y, \
            ndt_pose.pose.orientation.z, ndt_pose.pose.orientation.w], dtype=np.float64))
    T_map[0,3] = ndt_pose.pose.position.x
    T_map[1,3] = ndt_pose.pose.position.y
    T_map[2,3] = ndt_pose.pose.position.z

    T_map_to_odom = np.matmul(T_map, np.linalg.inv(T_odom))

    _, _, angles, translate, _ = decompose_matrix(T_map_to_odom)
    q_map_to_odom = quaternion_from_euler(angles[0], angles[1], angles[2])

    t = TransformStamped()
    t.transform.translation.x = translate[0]
    t.transform.translation.y = translate[1]
    t.transform.translation.z = translate[2]
    t.transform.rotation.x = q_map_to_odom[0]
    t.transform.rotation.y = q_map_to_odom[1]
    t.transform.rotation.z = q_map_to_odom[2]
    t.transform.rotation.w = q_map_to_odom[3]

    pose_transformed = tf2_geometry_msgs.do_transform_pose(odom.pose, t)

    if (self.pose_valid):
      self.locked_pose.set_object(ndt_pose)

    #print("Odom {}".format(odom.pose.pose.position))
    #print("Map {}".format(ndt_pose.pose.position))
    #print("Transformed {}".format(pose_transformed.pose.position))

    #self.br.sendTransform( \
    #  (translate[0], translate[1], translate[2]), q_map_to_odom, \
    #  rospy.Time.now(), "odom", "map")

  def gnss_pose_local_callback(self, pose):
    cur_pose = self.locked_cur_pose.get_object()
    if (not np.isnan(pose.pose.position.x) and 
        not np.isnan(pose.pose.position.y) and 
        not np.isnan(pose.pose.position.z)):
      cur_pose[0] = pose.pose.position.x
      cur_pose[1] = pose.pose.position.y
      cur_pose[2] = pose.pose.position.z

    # X. Assign 0 to orientation.
    cur_pose[3] = 0
    cur_pose[4] = 0    
    cur_pose[5] = 0
    self.locked_cur_pose.set_object(cur_pose)

  def create_dpose_list(self):
    dpose_list = list()
    for dx in range(-10, 11):
      for dy in range(-10, 11):
        for dyaw in range(36):
          dpose_list.append([dx, dy, 10.0 * dyaw / 180.0 * math.pi])
    return dpose_list

if __name__ == '__main__':

  parser = argparse.ArgumentParser()
  parser.add_argument('-x', type=float)
  parser.add_argument('-y', type=float)
  parser.add_argument('-z', type=float)
  parser.add_argument('-roll', type=float)
  parser.add_argument('-pitch', type=float)
  parser.add_argument('-yaw', type=float)
  args = parser.parse_args(rospy.myargv()[1:])

  mgr = LocalizeManager(args.x, args.y, args.z, args.roll, args.pitch, args.yaw)


  #ndt.setResolution(1.0);
  #ndt.setStepSize(0.1);
  #ndt.setOulierRatio(0.55);
  #ndt.setTransformationEpsilon(0.01);
  #ndt.setMaximumIterations(30);