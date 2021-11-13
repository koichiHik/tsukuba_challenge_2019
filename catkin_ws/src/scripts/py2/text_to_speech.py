#!/usr/bin/env python

import rospy
import pyttsx3
import threading, Queue
import time

DEFAULT_RATE = 200

class TextToSpeech():

  def __init__(self):
    self.__terminate = False
    self.__engine = pyttsx3.init()
    self.__engine.setProperty('voice', 'english+f4')
    self.__engine.setProperty("rate", DEFAULT_RATE)

    self.__text_queue = Queue.Queue()

    self.__thread = threading.Thread(target=self.run)
    self.__thread.start()

  def add_speech_text(self, text, rate=DEFAULT_RATE):
    if (not self.__terminate):
      self.__text_queue.put([text, rate], block=False)

  def run(self):

    while not self.__terminate:

      try:
        text_info = self.__text_queue.get(block=True, timeout=0.5)
        self.__engine.setProperty("rate", text_info[1])
        self.__engine.say(text_info[0])
        self.__engine.runAndWait()
      except Queue.Empty:
        pass
      else:
        self.__text_queue.task_done()        

  def terminate(self):
    self.__terminate = True
    self.__thread.join()


if __name__ == '__main__':

  speecher = TextToSpeech()
  speecher.add_speech_text("Localization reliability is low.")

  speecher.add_speech_text("Robot stops due to obstcle in front.")

  time.sleep(10)

  speecher.terminate()
