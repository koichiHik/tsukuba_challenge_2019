#!/usr/bin/env python

# ROS
import rospy
from sensor_msgs.msg import Imu

count = 0
ax_sum = 0
ay_sum = 0
az_sum = 0
wx_sum = 0
wy_sum = 0
wz_sum = 0

def imu_raw_callback(msg):

  global count
  global ax_sum
  global ay_sum
  global az_sum
  global wx_sum
  global wy_sum
  global wz_sum

  count += 1
  ax_sum += msg.linear_acceleration.x
  ay_sum += msg.linear_acceleration.y
  az_sum += msg.linear_acceleration.z

  wx_sum += msg.angular_velocity.x
  wy_sum += msg.angular_velocity.y
  wz_sum += msg.angular_velocity.z

if __name__ == '__main__':

  rospy.init_node('imu_offset_calc')
  rospy.Subscriber('imu', Imu, imu_raw_callback)

  rospy.spin()

  print("Computed Offset")
  print("x : {}".format(ax_sum / count))
  print("y : {}".format(ay_sum / count))
  print("z : {}".format(az_sum / count))
  print("wx : {}".format(wx_sum / count))
  print("wy : {}".format(wy_sum / count))
  print("wz : {}".format(wz_sum / count))