#!/usr/bin/env python

#import the dependencies
import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
import time
import tf2_ros, tf.transformations 
from apriltag_ros.msg import AprilTagDetection
from apriltag_ros.msg import AprilTagDetectionArray


class Jackal:
	def __init__(self):
		self.pub=rospy.Publisher('/cmd_vel',Twist,queue_size=1)
		self.sub_img_detec =  rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.Callback)
		self.pose_x = None
		self.pose_z= None
		self.pose_flag = False
		self.tfBuffer = tf2_ros.Buffer()
		self.listener = tf2_ros.TransformListener(self.tfBuffer)
		self.target_frame = base_link
	def Callback(self,msg):
		if msg.detections:
			self.pose_x= msg.detections[0].pose.pose.pose.position.x
			self.pose_z= msg.detections[0].pose.pose.pose.orientation.z
			self.pose_flag  = True
			transform = self.tfBuffer.lookup_transform(target_frame, pose_stamped.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
			pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
		else:
			self.pose_x = None
			self.pose_z= None
			self.pose_flag = False

	def detect(self):
	    #while the node is still on
	    while not rospy.is_shutdown():
		# adjust the velocity message
			vel=Twist()
			vel.angular.z=1
			#publish it
			self.pub.publish(vel)
			if self.pose_flag:
				break


if __name__=="__main__":
    
	#initialise the node
	rospy.init_node("ap_tag", anonymous=True)
	det = Jackal()
	det.detect()
	rospy.spin()
	
	#initiate object


