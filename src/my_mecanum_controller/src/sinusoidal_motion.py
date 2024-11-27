#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import math

# Constants for the sinusoidal motion
AMPLITUDE = 1.0  # Amplitude of the sinusoid (meters)
FREQUENCY = 0.1  # Frequency of the sinusoid (Hz)
SPEED = 0.2  # Linear speed in the x-direction (m/s)

def move_in_sin_wave():
    rospy.init_node('sinusoidal_motion', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    rate = rospy.Rate(10)  # 10 Hz loop rate

    while not rospy.is_shutdown():
        twist = Twist()

        # Time-dependent velocity values
        t = rospy.get_time()

        # Sinusoidal motion: moving along the x-axis with a sinusoidal deviation in the y-axis
        twist.linear.x = SPEED
        twist.linear.y = AMPLITUDE * math.sin(2 * math.pi * FREQUENCY * t)  # Sinusoidal y velocity
        twist.angular.z = 0.0  # No angular velocity

        # Publish the velocity message
        pub.publish(twist)

        # Sleep to maintain the loop rate
        rate.sleep()

if __name__ == '__main__':
    try:
        move_in_sin_wave()
    except rospy.ROSInterruptException:
        pass

