import hello_misc as hm

import rospy
import logging
import tf
import tf2_ros
import time
from math import pi, sqrt, atan2
import json 
import os 

from std_srvs.srv import Trigger

import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, PoseStamped
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

import actionlib
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction

class ArucoNavigationNode(hm.HelloNode):
    def __init__(self):

        super().__init__()
        
        self.joint_state = None 
        self.file_path = rospy.get_param('/file_path')

        #Attempt to access current saved poses in saved_poses.json, if fail set to empty
        try:
            saved_file = open(self.file_path + '/saved_poses.json')
            self.pose_dict = json.load(saved_file)
            saved_file.close()
        except:
            self.pose_dict = {}
        
        self.main() 
    
    def pose_to_list(self, msg):
        """
        Converts PoseStamped message into a list (to be later saved as dictionary entry)
        input: pose - PostStamped msg
        """
        return [msg.header.frame_id, msg.pose.position.x, msg.pose.position.y,
              msg.pose.position.z, msg.pose.orentation.x, msg.pose.orientation.y, 
              msg.pose.orientation.z, msg.pose.orientation.w]
    

    
