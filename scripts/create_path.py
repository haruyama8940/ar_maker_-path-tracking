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
        self.srv = rospy.Service('move', SetBool, self.callback_srv)
        self.cmd_vel_pub =rospy.Publisher("cmd_vel",Twist,1)
        self.ar = AlvarMarker()
        self.vel = Twist()
        self.ar_pose_x =0
        self.ar_pose_z =0
        self.ar_id =69
        self.target_pose_x
        #self.target_pose_y
        self.target_pose_z
        self.x_thr = 0.1
        self.z_thr = 0.2
        self.detect_target_ar_flg =False
        self.move_flg =False
        # self.pub = rospy.Publisher('topic name', String, queue_size=1)
        self.current_num =0
        self.target_id_list = [0,1,2,3,4]
        self.target_action_list = ["right","left","left","left","right"]
        self.target_id  =self.target_id_list[self.current_num]
        self.target_action = self.target_action_list[self.current_num]

    def ar_pose_callback(self,data):
        self.ar.pose.pose.orientation.w
        self.ar_id = data.id
        self.ar_pose_x = data.pose.pose.position.x
        self.ar_pose_z = data.pose.pose.position.z
        self.ar_pose_z = data.pose.pose.orientation
        if self.ar_id == self.target_id:
            self.detect_target_ar_flg = True

    def callback_srv(self,data):
        resp = SetBoolResponse()
        self.move_flg = data.data
        resp.message = "catch move_flg !!"
        resp.success = True
        return resp
    def read_list(self):
        if self.current_num >= len(self.target_id_list):
            self.current_num = 0
        self.target_action = self.target_action_list[self.current_num]
        self.target_action = self.target_id_list[self.current_num]

    def path_function(self):
        a = (self.target_pose_z - 0) / (self.target_pose_x - 0)
    
    def track_ar(self):
        if self.move_flg:
            if (self.track_flg):
                if (abs(self.ar_pose_x) >= self.z_thr and abs(self.ar_pose_z) >= self.x_thr):
                    self.vel.linear.x = 0.2 * (self.ar_pose_z - self.z_thr)
                    self.vel.angular.z = 0.2 * self.ar_pose_x
                    self.cmd_vel_pub.publish(self.vel)
                    print("move for target AR_marker")
                else:
                    self.track_flg = False
                    print("reach target AR_marker !!")
                    self.current_num = self.current_num + 1
                    self.read_list()
                    if self.target_id ==3 or self.target_id == 0:
                        self.move_flg = False
            else:
                self.detect_target_ar_flg = False
    
    def search_sr(self):
        if self.move_flg:
            if self.detect_target_ar_flg==False:
                print("search AR_marker")
                if self.target_action == "right":
                    self.vel.angular.z = 0.2
                if self.target_action == "left":
                    self.vel.angular.z  = -0.2
                self.cmd_vel_pub.publish(self.vel)
            else :
                print("detect AR_marker")
                self.track_flg =True
    # def track_ori(self):
        
if __name__ == '__main__':
    rospy.init_node('track_ar_node')

    time.sleep(1.0)
    ar_path_node = AR_path_Node()
    print("ready")
    while not rospy.is_shutdown():
        ar_path_node     
        rospy.sleep(1.0)
