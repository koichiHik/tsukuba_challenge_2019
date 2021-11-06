#!/usr/bin/env python

# STL
import sys
import os
import argparse
import numpy as np
import json

# ROS
import rospy
from geometry_msgs.msg import Pose, PoseStamped
from tf.transformations import quaternion_multiply, quaternion_matrix

class PoseTransformer:

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

    rospy.init_node('gnss_pose_transformer')
    rospy.Subscriber('gnss_pose', PoseStamped, self.pose_callback)
    self.pub = rospy.Publisher('gnss_pose_local', PoseStamped, queue_size=100)

    rospy.spin()
    
  def pose_callback(self, pose_stamped):

    src_trans = \
      np.array( \
        [pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z], \
        dtype=np.float64)
    src_quat = \
      np.array( \
        [pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y, \
         pose_stamped.pose.orientation.z, pose_stamped.pose.orientation.w], \
        dtype=np.float64)

    new_trans = np.matmul(self.R, (src_trans - self.gnss_cog)) + self.map_cog
    new_quat = quaternion_multiply(self.quat, src_quat)

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

  parser = argparse.ArgumentParser()
  parser.add_argument('-world_to_map_json', type=str, required=True)
  args = parser.parse_args(rospy.myargv()[1:])

  # X. Run subscriber.
  transformer = PoseTransformer(args.world_to_map_json)
