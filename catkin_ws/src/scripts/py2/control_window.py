#!/usr/bin/env python

# Wx
#import wxversion
#wxversion.select("4.0.0")
import wx, wx.html

print(wx.__version__)

# Ros
import rospy
from std_msgs.msg import Bool

# STL
import sys
import threading

class ButtonID:

    ENGAGE = 1
    AUTORUN = 2

class Engage:

    class Color:
        DISENGAGED = (255, 0, 0, 255)
        ENGAGED = (0, 0, 255, 255)

    class Label:
        DISENGAGED = "DISENGAGED"
        ENGAGED = "ENGAGED"

class Autorun:

    class Color:
        SUSPENDED = (255, 0, 0, 255)
        AUTORUN = (0, 0, 255, 255)

    class Label:
        SUSPENDED = "SUSPENDED"
        AUTORUN = "AUTORUN"

class Frame(wx.Frame):

    def __init__(self, title):

        # Prepare GUI.
        wx.Frame.__init__(self, None, title=title, pos=(1000,150), size=(200, 800))
        self.Bind(wx.EVT_CLOSE, self.OnClose)
        self.lock = threading.Lock()

        panel = wx.Panel(self)
        box = wx.BoxSizer(wx.VERTICAL)

        # Engage Button
        self.engage_btn = wx.Button(panel, id = ButtonID.ENGAGE, label = Engage.Label.DISENGAGED,  size = (200, 200))
        self.engage_btn.Bind(wx.EVT_BUTTON, self.OnEngage)
        self.engage_btn.SetBackgroundColour(Engage.Color.DISENGAGED)
        box.Add(self.engage_btn, 0, wx.ALL, 10)

        # Autorun Button
        self.autorun_btn = wx.Button(panel, id = ButtonID.AUTORUN, label = Autorun.Label.SUSPENDED,  size = (200, 200))
        self.autorun_btn.Bind(wx.EVT_BUTTON, self.OnAutorun)
        self.autorun_btn.SetBackgroundColour(Autorun.Color.SUSPENDED)
        self.autorun_btn.Disable()
        box.Add(self.autorun_btn, 0, wx.ALL, 10)

        panel.SetSizer(box)
        panel.Layout()

        # Prepare ROS.
        self.engage_pub = rospy.Publisher("control_window_engage_request", Bool, queue_size=10, latch=True)
        self.autorun_pub = rospy.Publisher("control_window_autorun_request", Bool, queue_size=10, latch=True)

        engage_msg = Bool()
        engage_msg.data = False
        self.engage_pub.publish(engage_msg)

        autorun_msg = Bool()
        autorun_msg.data = False
        self.autorun_pub.publish(autorun_msg)

        self.status_mgmt_sub = rospy.Subscriber("status_management_status", Bool, self.OnStatusManagementStatusCallback)

    def OnClose(self, event):
        dlg = wx.MessageDialog(self, 
            "Do you really want to close this application?",
            "Confirm Exit", wx.OK|wx.CANCEL|wx.ICON_QUESTION)
        result = dlg.ShowModal()
        dlg.Destroy()
        if result == wx.ID_OK:
            self.Destroy()

    def OnEngage(self, event):

        self.lock.acquire()

        engage_msg = Bool()
        last_engage_status = self.engage_btn.GetLabel()
        if (last_engage_status == Engage.Label.DISENGAGED):
            self.engage_btn.SetLabel(Engage.Label.ENGAGED)
            self.engage_btn.SetBackgroundColour(Engage.Color.ENGAGED)
            
            self.autorun_btn.Enable()
            # self.autorun_btn.SetLabel(Autorun.Label.SUSPENDED)
            # self.autorun_btn.SetBackgroundColour(Autorun.Color.SUSPENDED)

            engage_msg.data = True
        else:
            self.engage_btn.SetLabel(Engage.Label.DISENGAGED)
            self.engage_btn.SetBackgroundColour(Engage.Color.DISENGAGED)

            self.autorun_btn.SetLabel(Autorun.Label.SUSPENDED)
            self.autorun_btn.SetBackgroundColour(Autorun.Color.SUSPENDED)
            self.autorun_btn.Disable()

            engage_msg.data = False

        self.engage_pub.publish(engage_msg)

        self.lock.release()


    def OnAutorun(self, event):

        self.lock.acquire()

        autorun_msg = Bool()
        last_autorun_status = self.autorun_btn.GetLabel()
        if (last_autorun_status == Autorun.Label.SUSPENDED):
            self.autorun_btn.SetLabel(Autorun.Label.AUTORUN)
            self.autorun_btn.SetBackgroundColour(Autorun.Color.AUTORUN)

            autorun_msg.data = True
        else:
            self.autorun_btn.SetLabel(Autorun.Label.SUSPENDED)
            self.autorun_btn.SetBackgroundColour(Autorun.Color.SUSPENDED)

            autorun_msg.data = False

        self.autorun_pub.publish(autorun_msg)

        self.lock.release()

    def OnStatusManagementStatusCallback(self, data):
        wx.CallAfter(self.ReflectStatusManagementResult, data)

    def ReflectStatusManagementResult(self, data):

        self.lock.acquire()
        print('OnStatusManagementStatusCallback')
        if (not data.data):
            self.autorun_btn.SetLabel(Autorun.Label.SUSPENDED)
            self.autorun_btn.SetBackgroundColour(Autorun.Color.SUSPENDED)

            autorun_msg = Bool()
            autorun_msg.data = False
            self.autorun_pub.publish(autorun_msg)
        
        self.lock.release()

def main():

    rospy.init_node('control_window', anonymous=True)

    app = wx.App(redirect=False)   # Error messages go to popup window
    top = Frame("Auto Drive")
    top.Show()
    app.MainLoop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


