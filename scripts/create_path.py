#! /usr/bin/env python3
import numpy as np
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import time
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
import roslib
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest
#roslib.load_manifest('path')

class AR_path_Node():
    def __init__(self):
        self.ar_maker_sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers,self.ar_pose_callback)
        self.srv = rospy.Service("move", SetBool, self.callback_srv)
        self.cmd_vel_pub =rospy.Publisher("/cmd_vel",Twist,queue_size=10)
        self.ar = AlvarMarkers()
        self.vel = Twist()
        self.ar_pose_x =0
        self.ar_pose_z =0
        self.ar_id =69
        self.target_pose_x =0
        #self.target_pose_y
        self.target_pose_z =0
        self.x_thr = 0.1
        self.z_thr = 0.1
        self.detect_target_ar_flg =False
        self.move_flg =True
        self.track_flg =False
        # self.pub = rospy.Publisher('topic name', String, queue_size=1)
        self.current_num =0
        self.target_id_list = [0,1,2,3,4]
        self.target_action_list = ["right","left","left","left","right"]
        self.target_id  =self.target_id_list[self.current_num]
        self.target_action = self.target_action_list[self.current_num]

    def ar_pose_callback(self,data):
        print("target_id" ,self.target_id)
        self.ar_maker = self.ar.markers

        self.ar_maker_data = data.markers
        if len(self.ar_maker_data) ==0:
            self.detect_target_ar_flg=False
        
        for m in self.ar_maker_data:
            self.ar_id = m.id
            self.ar_pose_x = m.pose.pose.position.x
            self.ar_pose_z = m.pose.pose.position.z
            self.ar_pose_w = m.pose.pose.orientation.w
            print("detect id",self.ar_id)
        if self.ar_id == self.target_id:
            print("target_maker")
            self.detect_target_ar_flg = True
        else :
            self.detect_target_ar_flg =False

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
        self.target_id = self.target_id_list[self.current_num]
        print("target_id" ,self.target_id,"target_action" , self.target_action)
    # def path_function(self):
    #     a = (self.target_pose_z - 0) / (self.target_pose_x - 0)
    
    def track_ar(self):
        print("track")
        if self.move_flg:
            if (self.track_flg):
                print(type(self.ar_pose_x))
                if (abs(self.ar_pose_x) >= self.z_thr and abs(self.ar_pose_z) >= self.x_thr):
                    self.vel.linear.x = 0.1 * (self.ar_pose_z - self.x_thr)
                    self.vel.angular.z = -0.1 * (self.ar_pose_x -0)
                    self.cmd_vel_pub.publish(self.vel)
                    print("move for target AR_marker")
                else:
                    self.track_flg = False
                    print("reach target AR_marker !!")
                    self.current_num = self.current_num + 1
                    self.read_list()
                    if self.target_id ==3 or self.target_id == 0:
                        print("wait")
                        self.move_flg = False
            else:
                self.detect_target_ar_flg = False
    
    def search_sr(self):
        print("search")
        if self.move_flg:
            if self.detect_target_ar_flg==False:
                print("search AR_marker")
                if self.target_action == "right":
                    self.vel.linear.x = 0.0
                    self.vel.angular.z = 0.2
                if self.target_action == "left":
                    self.vel.linear.x =0.0
                    self.vel.angular.z  = -0.2
                self.cmd_vel_pub.publish(self.vel)
            if self.detect_target_ar_flg:
                print("detect AR_marker")
                self.track_flg =True
    # def track_ori(self):
    def loop(self):
        self.read_list()
        self.search_sr()
        self.track_ar()
        
if __name__ == '__main__':
    rospy.init_node('track_ar_node')
    r=rospy.Rate(1)
    ar_path_node = AR_path_Node()
    print("ready")
    while not rospy.is_shutdown():
        ar_path_node.loop()
        r.sleep()
