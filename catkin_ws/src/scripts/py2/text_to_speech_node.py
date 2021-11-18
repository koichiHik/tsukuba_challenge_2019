#!/usr/bin/env python

import rospy
import pyttsx3
import threading, Queue
import time
from std_msgs.msg import String

DEFAULT_RATE = 200

class TextToSpeechNode():

  def __init__(self):

    self.engine = pyttsx3.init()
    self.engine.setProperty('voice', 'english+f4')
    self.engine.setProperty('rate', DEFAULT_RATE)

    rospy.init_node('announce')
    rospy.Subscriber('status_message', String, self.message_callback)
    
    rospy.spin()

  def message_callback(self, text):

    self.engine.say(text.data)
    self.engine.runAndWait()


if __name__ == '__main__':

  rospy.init_node("announce")
  node = TextToSpeechNode()



