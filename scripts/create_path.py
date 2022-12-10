#! /usr/bin/env python3
import numpy as np
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import time
from ar_track_alvar_msgs.msg import AlvarMarker
import roslib
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest
#roslib.load_manifest('path')

class AR_path_Node():
    def __init__(self):
        self.ar_maker_sub = rospy.Subscriber('ar_pose_marker ',AlvarMarker,self.ar_pose_callback)
        self.srv = rospy.Service('test_speak', SetBool, self.callback_srv)
        self.cmd_vel_pub =rospy.Publisher("cmd_vel",Twist,1)
        self.srv = rospy.Service('test_speak', SetBool, self.callback_srv)
        self.cmd_vel_pub =rospy.Publisher("cmd_vel",Twist,1)
        self.ar = AlvarMarker()
        self.vel = Twist()
        self.ar_pose_x =0
        self.ar_pose_z =0
        self.ar_id =69
        self.target_pose_x
        #self.target_pose_y
        self.target_pose_z
        self.target_id
        self.x_thr = 0.1
        self.z_thr = 0.2
        # self.pub = rospy.Publisher('topic name', String, queue_size=1)

    def ar_pose_callback(self,data):
        self.ar.pose.pose.orientation.w
        self.ar_pose = self.data.id
        self.ar_pose_x = self.data.pose.pose.position.x
        self.ar_pose_z = self.data.pose.pose.position.z
        self.ar_pose_z = self.data.pose.pose.orientation
        self.detect_flg =True

    def callback_srv(self,data):
        resp = SetBoolResponse()
        if data.data == True:
            play_sound = self.auto_sound.play()
            play_sound.wait_done()

    def path_function(self):
        a = (self.target_pose_z - 0) / (self.target_pose_x - 0)
    
    def track_ar(self):
        if (self.track_flg and abs(self.ar_pose_x) >= self.z_thr and abs(self.ar_pose_z) >= self.x_thr):
            self.vel.linear.x = 0.2 * (self.ar_pose_z - self.z_thr)
            self.vel.angular.z = 0.2 * self.ar_pose_x
            self.cmd_vel_pub.publish(self.vel)
        else:
            self.track_flg = False
    # def track_ori(self):
        
if __name__ == '__main__':
    rospy.init_node('test_node')

    time.sleep(1.0)
    ar_path_node = AR_path_Node()
    print("ready")
    while not rospy.is_shutdown():
        ar_path_node     
        rospy.sleep(1.0)
