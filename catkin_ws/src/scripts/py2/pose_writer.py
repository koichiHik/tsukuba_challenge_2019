#!/usr/bin/env python

# Python STL
import json
from collections import OrderedDict
import time
import numpy as np
import argparse

# ROS
import rospy
import message_filters
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix

# 3rd party
from googleearthplot.googleearthplot import googleearthplot

def create_dict_elem_from_pose_stamped(p):

  ts = p.header.stamp.to_sec()
  x = p.pose.position.x
  y = p.pose.position.y
  z = p.pose.position.z

  qx = p.pose.orientation.x
  qy = p.pose.orientation.y
  qz = p.pose.orientation.z
  qw = p.pose.orientation.w

  return OrderedDict([('ts',ts),('x',x),('y',y),('z',z),('qx',qx),('qy',qy),('qz',qz),('qw',qw)])

class MapWorldTransformSolver:

  def __init__(self, localized_pose_filepath, \
      pose_pair_filepath, kml_filepath):

    # X. Init node.
    rospy.init_node('map_world_transform_solver')

    # X. Prepare localization subscriber
    rospy.Subscriber('ndt_pose', PoseStamped, self.localized_pose_callback)
    self.loc_pose_list = list()

    # X. Prepare fix subscriber
    kml_fileobj = open(kml_filepath, 'w')
    rospy.Subscriber('fix', NavSatFix, self.fix_callback)
    gep = googleearthplot()
    self.fix_list = list()

    # X. Prepare time synchronizer.
    gnss_pose_sub = message_filters.Subscriber('gnss_pose', PoseStamped)
    localized_pose_sub = message_filters.Subscriber('ndt_pose', PoseStamped)
    ts = message_filters.ApproximateTimeSynchronizer(\
      [gnss_pose_sub, localized_pose_sub], 10, 0.2)
    ts.registerCallback(self.sync_callback)
    self.pose_pair_list = list()

    # X. Collect data.
    rospy.spin()

    # X. Output to file
    with open(localized_pose_filepath, 'w') as f:
      json.dump(self.loc_pose_list, f, indent=4)

    with open(pose_pair_filepath, 'w') as f:
      json.dump(self.pose_pair_list, f, indent=4)

    for fix in self.fix_list:
      gep.PlotPoints(fix[0], fix[1], "")
    gep.GenerateKMLFile(filepath=kml_filepath)

  def sync_callback(self, gnss_pose, localized_pose):
    print('Receive gnss pose and localized pose')
    gnss_elem = create_dict_elem_from_pose_stamped(gnss_pose)
    localized_elem = create_dict_elem_from_pose_stamped(localized_pose)
    elem = OrderedDict({'gnss':gnss_elem, 'localized':localized_elem})
    self.pose_pair_list.append(elem)

  def localized_pose_callback(self, localized_pose):
    print('Receive localized pose')
    elem = create_dict_elem_from_pose_stamped(localized_pose)
    self.loc_pose_list.append(elem)

  def fix_callback(self, fix):
    print('Recieve fix data')
    self.fix_list.append([fix.latitude, fix.longitude, fix.altitude])

if __name__ == '__main__':

  parser = argparse.ArgumentParser()
  parser.add_argument('-localized_pose_filepath', type=str, required=True)
  parser.add_argument('-pose_pair_filepath', type=str, required=True)
  parser.add_argument('-kml_filepath', type=str, required=True)
  args = parser.parse_args(rospy.myargv()[1:])

  obj = MapWorldTransformSolver( \
        args.localized_pose_filepath, \
        args.pose_pair_filepath, \
        args.kml_filepath)
