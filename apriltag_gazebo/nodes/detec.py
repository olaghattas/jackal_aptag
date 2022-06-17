#!/usr/bin/env python

#import the dependencies
 
import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
import time
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
import numpy as np

class VelocityController:

    def __init__(self, Kp=3):
        '''
        '''
        self.Kp = Kp

    def proportional_control(self, error):
		return self.Kp * error

	def integral_control(self, error, dt):
        return self.Ki * error * dt

    def derivative_control(self, error, previous_error, dt):
        return self.Kd * (error - previous_error)/dt

	
	def velocity_control(self, error, dt, prev_error , vel):
		max_vel = 4 
        mv_p = self.proportional_control(error)
        mv_i = self.integral_control(error, dt)
        mv_d = self.derivative_control(error, prev_err, dt)

        desired_vel = np.clip( mv_p + mv_i + mv_d, -max_vel, max_vel)
        return desired_vel


class Jackal:
	def __init__(self,prev_time = None, current_time= None):
		self.pub=rospy.Publisher('/cmd_vel',Twist,queue_size=1)
		self.sub_img_detec =  rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.Callback_detection)
		self.pose_x = None
		self.pose_y = None
		self.pose_z = None
		self.ang_pose_z= None
		self.pose_flag = False
		self.vel = Twist()
		self.controller = VelocityController()
		self.prev_error = 0
		self.ap_id = [None]
		self.det_id = None
		self.tfBuffer = tf2_ros.Buffer()
		self.listener = tf2_ros.TransformListener(self.tfBuffer)

        

	def Callback_detection(self,msg):
		if msg.detections and msg.detections[0].id not in self.ap_id:
			rospy.loginfo('withco',msg.detections[0])
			transform = self.tfBuffer.lookup_transform("base_link","front_realsense_gazebo" , rospy.Time(0), rospy.Duration(1.0))
			pose_transformed = tf2_geometry_msgs.do_transform_pose(msg.detections[0].pose.pose, transform)
			rospy.loginfo(pose_transformed)
			self.pose_x= pose_transformed.pose.position.x
			self.pose_y= pose_transformed.pose.position.y
			self.pose_z= pose_transformed.pose.position.z
			self.ang_pose_z= pose_transformed.pose.orientation.z
			self.pose_flag  = True
			self.det_id = msg.detections[0].id
			
			
		else:
			self.pose_x = None
			self.pose_y = None
			self.pose_z = None
			self.ang_pose_z= None
			self.pose_flag = False

	def spin(self):
		# adjust the velocity message
		self.vel.angular.z=1
		#publish it
		self.pub.publish(self.vel)

				
	def dist(self):
		return np.linalg.norm([self.pose_x, self.pose_y, self.pose_z])

	def move_towards_tag(self):
		if self.pose_x is not None and self.pose_y is not None and self.pose_z is not None:
			dist_to_goal = self.dist()
			while dist_to_goal > 0.65:
			
				current_error = self.ang_pose_z
				if current_error is None : 
					break
				self.vel.linear.x = 2
				self.vel.angular.z = self.controller.proportional_control(current_error)
				self.pub.publish(self.vel)

				if self.pose_x is not None and self.pose_y is not None and self.pose_z is not None:
					dist_to_goal = self.dist()
			
			self.update()

	def update(self):
	
		self.vel.linear.x = 0
		self.ap_id[0] = self.det_id

if __name__=="__main__":
    
	#initialise the node
	rospy.init_node("ap_tag", anonymous=True)
	jack = Jackal()
	#while the node is still on
	r = rospy.Rate(5)
	while not rospy.is_shutdown():
		if not jack.pose_flag:
			jack.spin()
		else:
			jack.move_towards_tag()
		r.sleep()

	

	