#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class TurtleCircleController:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('turtle_circle_controller', anonymous=True)
        
        # Publisher for turtle velocity commands
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        # Subscriber to get turtle's current pose
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        
        # Current pose of the turtle
        self.current_pose = Pose()
        
        # Circle parameters
        self.radius = 2.0  # meters
        self.linear_speed = 1.0  # m/s
        self.angular_speed = self.linear_speed / self.radius  # rad/s
        
        # Rate for the control loop
        self.rate = rospy.Rate(10)  # 10 Hz
        
        rospy.loginfo("Turtle Circle Controller initialized!")
        rospy.loginfo(f"Circle radius: {self.radius} meters")
        rospy.loginfo(f"Linear speed: {self.linear_speed} m/s")
        rospy.loginfo(f"Angular speed: {self.angular_speed} rad/s")
    
    def pose_callback(self, pose):
        """Callback function to update current turtle pose"""
        self.current_pose = pose
    
    def move_in_circle(self):
        """Make the turtle move in a circle"""
        # Create a Twist message
        vel_msg = Twist()
        
        # Set linear velocity (forward motion)
        vel_msg.linear.x = self.linear_speed
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        
        # Set angular velocity (turning)
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = self.angular_speed
        
        rospy.loginfo("Starting to move in circles...")
        rospy.loginfo(f"Linear velocity: {vel_msg.linear.x} m/s")
        rospy.loginfo(f"Angular velocity: {vel_msg.angular.z} rad/s")
        
        # Keep moving until the node is shut down
        while not rospy.is_shutdown():
            # Publish the velocity command
            self.velocity_publisher.publish(vel_msg)
            
            # Log current position
            rospy.loginfo(f"Turtle position: x={self.current_pose.x:.2f}, y={self.current_pose.y:.2f}, theta={self.current_pose.theta:.2f}")
            
            # Sleep to maintain the control rate
            self.rate.sleep()
    
    def stop_turtle(self):
        """Stop the turtle"""
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        self.velocity_publisher.publish(vel_msg)
        rospy.loginfo("Turtle stopped!")

def main():
    try:
        # Create the turtle controller
        controller = TurtleCircleController()
        
        # Wait a moment for everything to initialize
        rospy.sleep(1)
        
        # Start moving in circles
        controller.move_in_circle()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Turtle Circle Controller stopped!")
    except KeyboardInterrupt:
        rospy.loginfo("Interrupted by user!")
        controller.stop_turtle()

if __name__ == '__main__':
    main() 