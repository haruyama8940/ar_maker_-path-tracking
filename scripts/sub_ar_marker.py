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

class AR_sub_Node():
    def __init__(self):
        self.ar_maker_sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers,self.ar_pose_callback)
        self.ar = AlvarMarkers()
        self.vel = Twist()
        self.detect_target_ar_flg =False
        self.move_flg =True
        self.track_flg =False

    def ar_pose_callback(self,data):
        self.ar_maker = self.ar.markers

        self.ar_maker_data = data.markers
        if len(self.ar_maker_data) ==0:
            self.detect_target_ar_flg=False
            print("maigo")
            # self.search_ar()
        
        for m in self.ar_maker_data:
            self.ar_id = m.id
            self.ar_pose_x = m.pose.pose.position.x
            self.ar_pose_z = m.pose.pose.position.z
            self.ar_pose_w = m.pose.pose.orientation.w
            print("detect id",self.ar_id)
        # if self.ar_id == self.target_id:
        #     print("target_maker")
        #     self.detect_target_ar_flg = True
        # else :
        #     self.detect_target_ar_flg =False
        #     self.search_ar()

    # def callback_srv(self,data):
    #     resp = SetBoolResponse()
    #     self.move_flg = data.data
    #     resp.message = "catch move_flg !!"
    #     resp.success = True
    #     return resp
        
if __name__ == '__main__':
    rospy.init_node('track_ar_node')
    r=rospy.Rate(1)
    ar_sub_node = AR_sub_Node()
    print("ready")
    while not rospy.is_shutdown():
        ar_sub_node()
        r.sleep()
