#!/usr/bin/env python

# STL
import sys
import os
import argparse
import numpy as np
import json

# Autoware
from autoware_msgs.srv import String, StringResponse

# ROS
import rospy
from geometry_msgs.msg import Pose, PoseStamped
from tf.transformations import quaternion_multiply, quaternion_matrix

class Wld2Map:

  def __init__(self, world_to_map_json):

    with open(world_to_map_json, 'r') as f:
      trans_json = json.load(f)

    self.gnss_cog = \
      np.array([trans_json['gnss_cog']['x'], \
                trans_json['gnss_cog']['y'], \
                trans_json['gnss_cog']['z']],dtype=np.float64)

    self.map_cog = \
      np.array([trans_json['map_cog']['x'], \
                trans_json['map_cog']['y'], \
                trans_json['map_cog']['z']],dtype=np.float64)

    self.quat = \
      np.array([trans_json['quat']['x'] ,\
                trans_json['quat']['y'] ,\
                trans_json['quat']['z'] ,\
                trans_json['quat']['w']],dtype=np.float64)

    self.R = quaternion_matrix(self.quat)[:3,:3]

class PoseTransformer:

  def __init__(self):

    self.wld_2_map = None

    rospy.init_node('gnss_pose_transformer')
    rospy.Service('load_world_to_map_json', String, self.load_world_2_map_json)
    rospy.Subscriber('gnss_pose', PoseStamped, self.pose_callback)

    self.pub = rospy.Publisher('gnss_pose_local', PoseStamped, queue_size=100)

    rospy.spin()

  def load_world_2_map_json(self, req):
    self.wld_2_map = Wld2Map(req.str)
    return StringResponse()

  def pose_callback(self, pose_stamped):

    if (self.wld_2_map is None):
      return

    src_trans = \
      np.array( \
        [pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z], \
        dtype=np.float64)
    src_quat = \
      np.array( \
        [pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y, \
         pose_stamped.pose.orientation.z, pose_stamped.pose.orientation.w], \
        dtype=np.float64)

    new_trans = np.matmul(self.wld_2_map.R, (src_trans - self.wld_2_map.gnss_cog)) + self.wld_2_map.map_cog
    new_quat = quaternion_multiply(self.wld_2_map.quat, src_quat)

    new_pose = PoseStamped()
    new_pose.header = pose_stamped.header

    new_pose.pose.position.x = new_trans[0]
    new_pose.pose.position.y = new_trans[1]
    new_pose.pose.position.z = new_trans[2]
    new_pose.pose.orientation.x = new_quat[0]
    new_pose.pose.orientation.y = new_quat[1]
    new_pose.pose.orientation.z = new_quat[2]
    new_pose.pose.orientation.w = new_quat[3]

    self.pub.publish(new_pose)


if __name__ == '__main__':

  # X. Run subscriber.
  transformer = PoseTransformer()
