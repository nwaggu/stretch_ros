#!/usr/bin/env python3

# Import modules
import rospy
import time
import numpy as np
import cv2
import matplotlib.pyplot as plt

def cat_points():
  
# Reading image
    font = cv2.FONT_HERSHEY_COMPLEX
    img2 = cv2.imread("/home/hello-robot/Downloads/cat.png", cv2.IMREAD_COLOR)
    whites = 255 * np.ones((np.shape(img2))) 
    print((np.shape(img2)))  
    # Reading same image in another 
    # variable and converting to gray scale.
    img = cv2.imread("/home/hello-robot/Downloads/cat.png", cv2.IMREAD_GRAYSCALE)
    
    # Converting image to a binary image
    # ( black and white only image).
    _, threshold = cv2.threshold(img, 110, 255, cv2.THRESH_BINARY)
    
    # Detecting contours in image.
    contours, _= cv2.findContours(threshold, cv2.RETR_TREE,
                                cv2.CHAIN_APPROX_SIMPLE)
    
    # Going through every contours found in the image.
    pointlist = []
    xlist = []
    ylist = []
    for cnt in contours :
    
        approx = cv2.approxPolyDP(cnt, 0.004 * cv2.arcLength(cnt, True), True)
    
        # draws boundary of contours.
        cv2.drawContours(whites, [approx], 0, (0, 0, 255), 3) 
    
        # Used to flatted the array containing
        # the co-ordinates of the vertices.
        n = approx.ravel() 
        i = 0
        
        for j in n :
            if(i % 2 == 0):
                y = n[i]
                y = np.round(20 + y*3/22, 3)
                ylist.append(y)
                x = n[i + 1]
                x = np. round(50 - 3*x/23, 3)
                xlist.append(x)
                pointlist.append([x*0.01,y*0.01])
                
                    
            i = i + 1
    print(pointlist)
    plt.scatter(xlist, ylist)
    plt.show()
    #cv2.imshow("namdi", whites)
    return pointlist

# Import the FollowJointTrajectoryGoal from the control_msgs.msg package to
# control the Stretch robot
from control_msgs.msg import FollowJointTrajectoryGoal

# Import JointTrajectoryPoint from the trajectory_msgs package to define
# robot trajectories
from trajectory_msgs.msg import JointTrajectoryPoint

#pose = {'gripper_aperture': 0.125}
#                self.move_to_pose(pose)

# Import hello_misc script for handling trajectory goals with an action client
import hello_helpers.hello_misc as hm

class MultiPointCommand(hm.HelloNode):
	"""
	A class that sends multiple joint trajectory goals to the stretch robot.
	"""

	def __init__(self, shape):
		"""
		Function that initializes the inhereted hm.HelloNode class.
		:param self: The self reference.
		"""
		converted_shape = []
		prev_point = [0.2,0]
		for point in shape:
			trajectory = JointTrajectoryPoint()
			trajectory.positions = [point[0], point[1],1.7]
			trajectory.velocities = [0.1,0.1,0]
			trajectory.accelerations = [0.1,0.1,0]
			converted_shape.append(trajectory)
		self.path = converted_shape
		hm.HelloNode.__init__(self)
		#pose = {'gripper_aperture': 0.125}
		#self.move_to_pose(pose)

	def issue_multipoint_command(self):
		"""
		Function that makes an action call and sends multiple joint trajectory goals
		to the joint_lift, wrist_extension, and joint_wrist_yaw.
		:param self: The self reference.
		"""

		# Set trajectory_goal as a FollowJointTrajectoryGoal and define
		# the joint names as a list

		trajectory_goal = FollowJointTrajectoryGoal()
		trajectory_goal.trajectory.joint_names = ['wrist_extension','joint_lift', 'joint_wrist_yaw']

		# Then trajectory_goal.trajectory.points is defined by a list of the joint
		# trajectory points
		trajectory_goal.trajectory.points = self.path

		# Specify the coordinate frame that we want (base_link) and set the time to be now
		trajectory_goal.trajectory.header.stamp = rospy.Time(0.0)
		trajectory_goal.trajectory.header.frame_id = 'base_link'

		# Make the action call and send the goal. The last line of code waits
		# for the result before it exits the python script
		self.trajectory_client.send_goal(trajectory_goal)
		rospy.loginfo('Sent list of goals = {0}'.format(trajectory_goal))
		self.trajectory_client.wait_for_result()

	def main(self):
		"""
		Function that initiates the multipoint_command function.
		:param self: The self reference.
		"""
		# The arguments of the main function of the hm.HelloNode class are the
		# node_name, node topic namespace, and boolean (default value is true)
		hm.HelloNode.main(self, 'multipoint_command', 'multipoint_command', wait_for_first_pointcloud=False)
		rospy.loginfo('issuing multipoint command...')
		self.issue_multipoint_command()
		time.sleep(2)

if __name__ == '__main__':
    try:
		
        # Instanstiate a `MultiPointCommand()` object and execute the main() method
        pointlist = cat_points()
        #pointlist = [[0.5,0.2],[0.2,0.2],[0.2,0.5],[0.5,0.5],[0.5,0.2]]
        node = MultiPointCommand(shape=pointlist)
        node.main()
    except KeyboardInterrupt:
        rospy.loginfo('interrupt received, so shutting down')
