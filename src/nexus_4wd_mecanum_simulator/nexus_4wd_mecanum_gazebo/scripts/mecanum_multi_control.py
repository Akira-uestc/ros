#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 协同制导
import os
import cv2
import time
import numpy as np
import math
# from cir2 import findcir
import rospy
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import tf
from std_msgs.msg import Int32
from gazebo_msgs.msg import ModelState
import time

set_pose = [	0,10,
				-3,-10,
	   			3,-10,
	   			0,-9]
set_angle_target = [0,0,0]
set_angle = [0,0,math.pi/2]
wp = 4
leader_angle_except = [0,-10,10]
f = 5
leader_vel = 0.5
follower_vel = 0.55
dh_q3 = 0
h_q = [0,0,0,0,0]
dh_q = [0,0,0,0]
kp1 = 0.4
kp2 = 0.03
kd = 0.3




class multi_control():
	def __init__(self):
		#设置循环的频率
		rate = rospy.Rate(f)
		self.command = 0
		q1_tmp = 0
		reset_flag = 1
		car0_vel = Twist()
		car1_vel = Twist()
		car2_vel = Twist()
		car3_vel = Twist()
		car0_vel_pub = rospy.Publisher('/tb3_0/cmd_vel', Twist, queue_size=10)
		car1_vel_pub = rospy.Publisher('/tb3_1/cmd_vel', Twist, queue_size=10)
		car2_vel_pub = rospy.Publisher('/tb3_2/cmd_vel', Twist, queue_size=10)
		car3_vel_pub = rospy.Publisher('/tb3_3/cmd_vel', Twist, queue_size=10)
		command_sub = rospy.Subscriber('/command', Int32 , self.command_callback)
		pose_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
		car1_pose = ModelState()
		car1_pose.model_name = 'tb3_1'
		car2_pose = ModelState()
		car2_pose.model_name = 'tb3_2'
		car3_pose = ModelState()
		car3_pose.model_name = 'tb3_3'
		car0_pose = ModelState()
		car0_pose.model_name = 'tb3_0'
		# Main while loop.
		while not rospy.is_shutdown():
			car0_position = rospy.wait_for_message('/tb3_0/odom', Odometry, timeout=5)
			car1_position = rospy.wait_for_message('/tb3_1/odom', Odometry, timeout=5)
			car2_position = rospy.wait_for_message('/tb3_2/odom', Odometry, timeout=5)
			car3_position = rospy.wait_for_message('/tb3_3/odom', Odometry, timeout=5)
			car0_x = car0_position.pose.pose.position.x
			car0_y = car0_position.pose.pose.position.y
			car1_x = car1_position.pose.pose.position.x
			car1_y = car1_position.pose.pose.position.y
			car2_x = car2_position.pose.pose.position.x
			car2_y = car2_position.pose.pose.position.y
			car3_x = car3_position.pose.pose.position.x
			car3_y = car3_position.pose.pose.position.y
			# QuatToEuler

			(car0_r, car0_p, car0_yaw) = tf.transformations.euler_from_quaternion(
				[car0_position.pose.pose.orientation.x, car0_position.pose.pose.orientation.y,
				 car0_position.pose.pose.orientation.z, car0_position.pose.pose.orientation.w])
			(car1_r, car1_p, car1_yaw) = tf.transformations.euler_from_quaternion(
				[car1_position.pose.pose.orientation.x, car1_position.pose.pose.orientation.y,
				 car1_position.pose.pose.orientation.z, car1_position.pose.pose.orientation.w])
			(car2_r, car2_p, car2_yaw) = tf.transformations.euler_from_quaternion(
				[car2_position.pose.pose.orientation.x, car2_position.pose.pose.orientation.y,
				 car2_position.pose.pose.orientation.z, car2_position.pose.pose.orientation.w])
			(car3_r, car3_p, car3_yaw) = tf.transformations.euler_from_quaternion(
				[car3_position.pose.pose.orientation.x, car3_position.pose.pose.orientation.y,
				 car3_position.pose.pose.orientation.z, car3_position.pose.pose.orientation.w])

			q1 = math.atan2((car0_y - car1_y), (car0_x - car1_x))
			q2 = math.atan2((car0_y - car2_y), (car0_x - car2_x))
			q_leader = (q1 + q2) / 2
			q3 = math.atan2((car0_y - car3_y), (car0_x - car3_x))
			r1 = math.sqrt(pow(car0_y - car1_y, 2) + pow(car0_x - car1_x, 2))
			r2 = math.sqrt(pow(car0_y - car2_y, 2) + pow(car0_x - car2_x, 2))
			r3 = math.sqrt(pow(car0_y - car3_y, 2) + pow(car0_x - car3_x, 2))

			print(car0_position.header.seq,car0_position.header.stamp)


			if(self.command == 0):
				print('stop')
				car0_vel = Twist()
				car1_vel = Twist()
				car2_vel = Twist()
				car3_vel = Twist()
				car0_vel_pub.publish(car0_vel)
				car1_vel_pub.publish(car1_vel)
				car2_vel_pub.publish(car2_vel)
				car3_vel_pub.publish(car3_vel)
			elif(self.command == 1):
				car1_vel.linear.x = 0.8
				car1_vel.angular.z = wp*(q1-q1_tmp)

				car1_vel_pub.publish(car1_vel)
				print(car1_vel.angular.z)

			elif(self.command == 2):
				#q1 = math.atan2((car0_y-car1_y),(car0_x-car1_x))
				theta1 = q1 - car1_yaw
				car1_vel.linear.x = 0.8
				car1_vel.angular.z = theta1
				car1_vel_pub.publish(car1_vel)
				print(theta1*57.3)
			elif(self.command == 3):
				'''		target		'''
				car0_vel.linear.x = 0.05
				car0_vel.angular.z = 0.01
				car0_vel_pub.publish(car0_vel)

				'''		leader1		'''
				dq1 = f*(q1-q1_tmp)
				theta1 = q1 - car1_yaw
				eta_m_e1 = theta1- math.pi/180*leader_angle_except[1]
				t_go1 = r1/leader_vel
				w1 = dq1 + 10*eta_m_e1/t_go1
				car1_vel.linear.x = leader_vel
				car1_vel.angular.z = w1
				car1_vel_pub.publish(car1_vel)
				#print(theta1 * 57.3,dq1,10*eta_m_e1/t_go1)
				'''		leader2		'''
				dq2 = f*(q2-q2_tmp)
				theta2 = q2 - car2_yaw
				eta_m_e2 = theta2- math.pi/180*leader_angle_except[2]
				t_go2 = r2/leader_vel
				w2 = dq2 + 10*eta_m_e2/t_go2
				car2_vel.linear.x = leader_vel
				car2_vel.angular.z = w2
				car2_vel_pub.publish(car2_vel)
				#print(theta2 * 57.3,dq2,10*eta_m_e2/t_go2)
				'''		follower1		'''
				t_go3 = r3/follower_vel
				e11 = q3 - q_leader + math.pi/180*h_q[1]
				alpha1 = -kp1*e11/t_go3
				dq3 = f*(q3-q3_tmp)
				dq_leader = f*(q_leader-q_leader_tmp)
				e21 = dq3 - dq_leader + dh_q[1]
				VVM1 = alpha1 - kp2*e11 - kd*e21 - 2*follower_vel/r3*dq3
				U1 = -r3 * VVM1
				car3_vel.linear.x = follower_vel
				car3_vel.angular.z = U1/follower_vel
				car3_vel_pub.publish(car3_vel)
				print((q3 - q_leader) * 57.3)

			elif(self.command == 5):
				if reset_flag :
					# Stop
					print('stop')
					car0_vel = Twist()
					car1_vel = Twist()
					car2_vel = Twist()
					car3_vel = Twist()
					car0_vel_pub.publish(car0_vel)
					car1_vel_pub.publish(car1_vel)
					car2_vel_pub.publish(car2_vel)
					car3_vel_pub.publish(car3_vel)
					time.sleep(0.5)
					# reset
					car0_pose.pose.position.x = set_pose[0]
					car0_pose.pose.position.y = set_pose[1]
					(car0_pose.pose.orientation.x, car0_pose.pose.orientation.y, car0_pose.pose.orientation.z,
					 car0_pose.pose.orientation.w) = tf.transformations.quaternion_from_euler(set_angle_target[0],
																							  set_angle_target[1],
																							  set_angle_target[2])
					car1_pose.pose.position.x = set_pose[2]
					car1_pose.pose.position.y = set_pose[3]
					(car1_pose.pose.orientation.x, car1_pose.pose.orientation.y, car1_pose.pose.orientation.z,
					 car1_pose.pose.orientation.w) = tf.transformations.quaternion_from_euler(set_angle[0],
																							  set_angle[1],
																							  set_angle[2])
					car2_pose.pose.position.x = set_pose[4]
					car2_pose.pose.position.y = set_pose[5]
					(car2_pose.pose.orientation.x, car2_pose.pose.orientation.y, car2_pose.pose.orientation.z,
					 car2_pose.pose.orientation.w) = tf.transformations.quaternion_from_euler(set_angle[0],
																							  set_angle[1],
																							  set_angle[2])
					car3_pose.pose.position.x = set_pose[6]
					car3_pose.pose.position.y = set_pose[7]
					(car3_pose.pose.orientation.x, car3_pose.pose.orientation.y, car3_pose.pose.orientation.z,
					 car3_pose.pose.orientation.w) = tf.transformations.quaternion_from_euler(set_angle[0],
																							  set_angle[1],
																							  set_angle[2])
					car0_pose.twist = Twist()
					car1_pose.twist = Twist()
					car2_pose.twist = Twist()
					car3_pose.twist = Twist()
					pose_pub.publish(car0_pose)
					pose_pub.publish(car1_pose)
					pose_pub.publish(car2_pose)
					pose_pub.publish(car3_pose)
					time.sleep(0.5)
					pose_pub.publish(car0_pose)
					pose_pub.publish(car1_pose)
					pose_pub.publish(car2_pose)
					pose_pub.publish(car3_pose)
					reset_flag = 0
				car0_vel = Twist()
				car1_vel = Twist()
				car2_vel = Twist()
				car3_vel = Twist()
				car0_vel_pub.publish(car0_vel)
				car1_vel_pub.publish(car1_vel)
				car2_vel_pub.publish(car2_vel)
				car3_vel_pub.publish(car3_vel)




			if(self.command != 5):
				reset_flag = 1
			q1_tmp = q1
			q2_tmp = q2
			q3_tmp = q3
			q_leader_tmp = q_leader
			rate.sleep()
	def command_callback(self,msg):
		self.command = msg.data
		print("Recieve command  ",msg.data)
if __name__ == '__main__':
	# ROS节点初始化
	rospy.init_node('multi_control', anonymous=True)
	try:
		multi_control()
	except rospy.ROSInterruptException:
		pass


