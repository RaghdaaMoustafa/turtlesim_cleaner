#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt

class Turtlebot:
    def __init__(self):

        rospy.init_node("turtlesimbot")
        
        self.current_pose = Pose()
        self.rate = rospy.Rate(1)
        
        self.velocity_publisher=rospy.Publisher('/turtle1/cmd_vel',Twist, queue_size=10)
        self.pose_subscriber=rospy.Subscriber('/turtle1/pose',Pose,self.update_func)

       
    def update_func(self, data):
        self.current_pose=data
    def set_goal_pose(self, goal):
        self.goal_pose = goal



    def distance (self, goal_pose):
        return sqrt(pow((goal_pose.x - self.current_pose.x), 2) +
                        pow((goal_pose.y - self.current_pose.y), 2))

    def linear_vel(self, goal_pose, beta=0):
        beta=rospy.get_param("beta")
        return beta * self.distance(goal_pose)

    def steering_angle(self,goal_pose):
        return atan2(goal_pose.y - self.current_pose.y, goal_pose.x - self.current_pose.x)

    def angular_vel(self, goal_pose, phi=0):
        phi=rospy.get_param("phi")
        return phi * (self.steering_angle(goal_pose) - self.current_pose.theta)
    
    def Go_to(self):
        goal_pose = Pose()
        goal_pose.x=rospy.get_param("goal_pose.x")
        goal_pose.y=rospy.get_param("goal_pose.y")
        distance_tolerance=rospy.get_param("tolerance")

        vel_msg = Twist()
        while self.distance(goal_pose) >= distance_tolerance:
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_pose)

            
            self.velocity_publisher.publish(vel_msg)

            
            self.rate.sleep()

        
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        
        rospy.spin()

if __name__ == '__main__':
    try:
        x = Turtlebot()
        x.Go_to()
    except rospy.ROSInterruptException:
        pass



