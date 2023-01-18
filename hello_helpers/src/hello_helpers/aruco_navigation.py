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


class Pose():
    """
    Stores and converts PoseStamped messages 
    """
    def __init__(self, **kwargs):
        self.id = kwargs.get("id")
        #Positional attributes
        self.pos_x = kwargs.get("pos_x")
        self.pos_y = kwargs.get("pos_y")
        self.pos_z = kwargs.get("pos_z")
        #Orientation attributes
        self.ori_x = kwargs.get("ori_x")
        self.ori_y = kwargs.get("ori_y")
        self.ori_z = kwargs.get("ori_z")
        self.ori_w = kwargs.get("ori_w")

    def toJson
    




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
        Converts PoseStamped message into a Pose object (to be later saved as dictionary entry)
        input: pose - PostStamped msg
        """
        return Pose(id = msg.header.frame_id, pos_x = msg.pose.position.x, pos_y =msg.pose.position.y,
              pos_z = msg.pose.position.z, ori_x = msg.pose.orentation.x, ori_y = msg.pose.orientation.y, 
              ori_z = msg.pose.orientation.z, ori_w = msg.pose.orientation.w)
    

    
