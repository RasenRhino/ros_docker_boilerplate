#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

class TurtleTeleopEnhanced:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('turtle_teleop_enhanced', anonymous=True)
        
        # Publisher for turtle velocity commands
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        # Settings
        self.settings = termios.tcgetattr(sys.stdin)
        self.speed = 1.0
        self.turn = 1.0
        
        rospy.loginfo("Enhanced Turtle Teleop initialized!")
        rospy.loginfo("Use WASD keys to control the turtle:")
        rospy.loginfo("  W - Move forward")
        rospy.loginfo("  S - Move backward")
        rospy.loginfo("  A - Turn left")
        rospy.loginfo("  D - Turn right")
        rospy.loginfo("  Q - Quit")
        rospy.loginfo("  R - Reset turtle position")
    
    def get_key(self):
        """Get a single keypress from the user"""
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def reset_turtle(self):
        """Reset turtle to center position"""
        rospy.loginfo("Resetting turtle position...")
        # This would require a service call to turtlesim
        # For now, just log the action
        rospy.loginfo("Turtle reset requested!")
    
    def run(self):
        """Main teleop loop"""
        try:
            while not rospy.is_shutdown():
                key = self.get_key()
                
                if key == 'w':
                    # Move forward
                    self.move_forward()
                elif key == 's':
                    # Move backward
                    self.move_backward()
                elif key == 'a':
                    # Turn left
                    self.turn_left()
                elif key == 'd':
                    # Turn right
                    self.turn_right()
                elif key == 'q':
                    # Quit
                    rospy.loginfo("Quitting teleop...")
                    break
                elif key == 'r':
                    # Reset turtle
                    self.reset_turtle()
                elif key == ' ':
                    # Stop
                    self.stop()
                
        except KeyboardInterrupt:
            rospy.loginfo("Teleop interrupted!")
        finally:
            self.stop()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
    
    def move_forward(self):
        """Move turtle forward"""
        vel_msg = Twist()
        vel_msg.linear.x = self.speed
        vel_msg.angular.z = 0.0
        self.velocity_publisher.publish(vel_msg)
        rospy.loginfo("Moving forward")
    
    def move_backward(self):
        """Move turtle backward"""
        vel_msg = Twist()
        vel_msg.linear.x = -self.speed
        vel_msg.angular.z = 0.0
        self.velocity_publisher.publish(vel_msg)
        rospy.loginfo("Moving backward")
    
    def turn_left(self):
        """Turn turtle left"""
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = self.turn
        self.velocity_publisher.publish(vel_msg)
        rospy.loginfo("Turning left")
    
    def turn_right(self):
        """Turn turtle right"""
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = -self.turn
        self.velocity_publisher.publish(vel_msg)
        rospy.loginfo("Turning right")
    
    def stop(self):
        """Stop the turtle"""
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        self.velocity_publisher.publish(vel_msg)
        rospy.loginfo("Stopped")

def main():
    try:
        teleop = TurtleTeleopEnhanced()
        teleop.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Teleop stopped!")

if __name__ == '__main__':
    main() 