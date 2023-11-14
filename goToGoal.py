#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""

@author: abirath
"""
###############################################################################################################################################################################
# Team Vegeta - Abirath Raju, Sreeranj Jayadevan
###############################################################################################################################################################################
import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import Point, Vector3, Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np

logger = rclpy.logging.get_logger('goToGoal')

class goToGoal(Node):
    def __init__(self):

        super().__init__('goToGoal')
        image_qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, durability=QoSDurabilityPolicy.VOLATILE, depth=1)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.callback_odom, image_qos_profile)
        self.position_subscriber = self.create_subscription(Vector3, '/obs_pos', self.callback_pos, image_qos_profile)
        self.odom_subscriber
        self.position_subscriber 
        self._vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.Init = True
        self.Init_pos = Point()
        self.wayPoints = []

        waypoints = [(1.75,0.0), (1.75,1.65), (0.0,1.65)]
        for i in waypoints:
            a = Vector3()
            a.x = i[0]
            a.y = i[1]
            self.wayPoints.append(a)
        self.theta_desired_goal = 0
        self.desired_distance = 0.20
        self.epsilon = 0.1
        self.globalPos = Point()
        self.globalAng = 0
        self.globalObstaclePos = Point()
        self.goal_1_reached = False
        self.goal_2_reached = False
        self.goal_3_reached = False
        self.goal_pos = np.array([1.5,0])
        self.robot_pos = np.zeros(2)
        self.obstacle_pos = np.zeros(2)
        self.switch_count = 0
        self.mode = "goToGoal"
        self.velocity = Twist()
        self.velocity.linear.x = 0.0
        self.velocity.linear.y = 0.0
        self.velocity.linear.z = 0.0
        self.velocity.angular.x = 0.0
        self.velocity.angular.y = 0.0
        self.velocity.angular.z = 0.0

    def determine_goal(self):
        if not self.goal_1_reached and not self.goal_2_reached and not self.goal_3_reached:
            self.goal_pos = np.array([self.wayPoints[0].x, self.wayPoints[0].y])
        if self.goal_1_reached and not self.goal_2_reached and not self.goal_3_reached:
            self.goal_pos = np.array([self.wayPoints[1].x, self.wayPoints[1].y])
        if self.goal_1_reached and self.goal_2_reached and not self.goal_3_reached:
            self.goal_pos = np.array([self.wayPoints[2].x, self.wayPoints[2].y])

    def reached_goal(self):
        if abs(self.wayPoints[0].x - self.robot_pos[0]) < 0.05 and abs(self.wayPoints[0].y - self.robot_pos[1]) < 0.05 and not self.goal_1_reached:
            self.goal_1_reached = True
            print("Reached goal 1!")
            velocity = Twist()
            velocity.linear.x = 0.0
            velocity.angular.z = 0.0
            self._vel_publisher.publish(velocity)
            time.sleep(2.0)

        if abs(self.wayPoints[1].x - self.robot_pos[0]) < 0.1 and abs(self.wayPoints[1].y - self.robot_pos[1]) < 0.1 and not self.goal_2_reached:
            self.goal_2_reached = True
            print("Reached goal 2!")
            velocity = Twist()
            velocity.linear.x = 0.0
            velocity.angular.z = 0.0
            self._vel_publisher.publish(velocity)
            time.sleep(2.0)

        if abs(self.wayPoints[2].x - self.robot_pos[0]) < 0.15 and abs(self.wayPoints[2].y - self.robot_pos[1]) < 0.15 and not self.goal_3_reached:
            self.goal_3_reached = True
            print("Reached goal 3!")
            velocity = Twist()
            velocity.linear.x = 0.0
            velocity.angular.z = 0.0
            self._vel_publisher.publish(velocity)
            time.sleep(2.0)

        if self.goal_1_reached and self.goal_2_reached and self.goal_3_reached:
            velocity = Twist()
            velocity.linear.x = 0.0
            velocity.angular.z = 0.0
            self._vel_publisher.publish(velocity)

    def callback_odom(self,Odom):
        position = Odom.pose.pose.position
        q = Odom.pose.pose.orientation
        orientation = np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

        if self.Init:
            self.Init = False
            self.Init_ang = orientation
            self.globalAng = self.Init_ang
            Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])        
            self.Init_pos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y
            self.Init_pos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y
            self.Init_pos.z = position.z

        Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])        
        self.globalPos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y - self.Init_pos.x
        self.globalPos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y - self.Init_pos.y
        self.globalAng = orientation - self.Init_ang
        self.robot_pos[0] = self.globalPos.x
        self.robot_pos[1] = self.globalPos.y

    def callback_pos(self, pos):
        self.obstacle_pos[0] = self.robot_pos[0] + pos.x
        self.obstacle_pos[1] = self.robot_pos[1] + pos.y

        self.determine_goal()
        print( self.goal_pos)
        print( self.robot_pos)
        self.go()
        self.reached_goal()

    def avoid_obstacle(self):
            distance = np.linalg.norm(self.robot_pos - self.obstacle_pos)
            velocity = Twist()
            x_val = 0.5 * distance
            x_val = min(x_val, 0.2) * (self.robot_pos[0] - self.obstacle_pos[0])/abs((self.robot_pos[0] - self.obstacle_pos[0]))
            velocity.linear.x = x_val
            velocity.angular.z = 0.0
            return velocity

    def goToGoal(self):
            theta_desired_goal = np.arctan2((self.goal_pos[1]-self.robot_pos[1]),(self.goal_pos[0]-self.robot_pos[0]))
            error = theta_desired_goal - self.globalAng
            error = np.arctan2(np.sin(error), np.cos(error))
            z_val = np.arctan2(np.sin(error),np.cos(error))
            if z_val <= 0:
                z_val = max(1.5*z_val, -2.0)
            else:
                z_val = min(1.5*z_val,2.0)
            dist_to_goal = np.linalg.norm(self.robot_pos - self.goal_pos)
            if np.deg2rad(-7) < error < np.deg2rad(3):
                x_val = max(min(0.5 * dist_to_goal, 0.2), 0.1)
                z_val = 0.0
            else:
                x_val = 0.0

            velocity = Twist()
            velocity.linear.x = x_val
            velocity.angular.z = z_val
            return velocity

    def follow_wall_c(self):

            self.heading_vector_cc = np.array([-1 * (self.obstacle_pos[1] - self.robot_pos[1]), self.obstacle_pos[0] - self.robot_pos[0]])
            heading_vector = self.heading_vector_cc
            heading_angle = np.arctan2(heading_vector[1], heading_vector[0])
            print("Heading Angle: ", heading_angle)
            error = heading_angle
            error = np.arctan2(np.sin(error), np.cos(error))
            z_val = 2 * error
            if z_val <= 0:
                z_val = max(z_val, -2.0)
            else:
                z_val = min(z_val,2.0)
            if abs(error) < np.deg2rad(3):
                x_val = np.linalg.norm(heading_vector) * 0.5
            else:
                x_val = 0.0

            velocity = Twist()
            velocity.linear.x = x_val
            velocity.angular.z = z_val

            return velocity
                
    def follow_wall_cc(self):
            self.heading_vector_c = np.array([self.obstacle_pos[1] - self.robot_pos[1], -1*(self.obstacle_pos[0] - self.robot_pos[0])])
            heading_vector = self.heading_vector_c
            heading_angle = np.arctan2(heading_vector[1], heading_vector[0])
            print("Heading Angle: ", heading_angle)
            error = heading_angle
            error = np.arctan2(np.sin(error), np.cos(error))
            z_val = 2 * error
            if z_val <= 0:
                z_val = max(z_val, -2.0)
            else:
                z_val = min(z_val,2.0)
            if abs(error) < np.deg2rad(3):
                x_val = np.linalg.norm(heading_vector) * 0.5
            else:
                x_val = 0.0

            velocity = Twist()
            velocity.linear.x = x_val
            velocity.angular.z = z_val

            return velocity  
                
    def go(self):

            self.obstacle_distance = np.linalg.norm(self.obstacle_pos - self.robot_pos)
            self.heading_vector_goal = self.goal_pos - self.robot_pos
            self.heading_vector_avoid_obstacle = self.robot_pos - self.obstacle_pos
            self.heading_vector_c = np.array([-1 * (self.obstacle_pos - self.robot_pos)[1], (self.obstacle_pos - self.robot_pos)[0]])
            self.heading_vector_cc = np.array([(self.obstacle_pos - self.robot_pos)[1], -1 * (self.obstacle_pos - self.robot_pos)[0]])

            if(self.mode == "goToGoal") and (self.obstacle_distance <= (self.desired_distance)) and (np.dot(self.heading_vector_c, self.heading_vector_goal) > 0):
                self.robot_pos_switch = np.copy(self.robot_pos)
                self.mode = "followWallC"

            if(self.mode == "goToGoal") and (self.obstacle_distance <= (self.desired_distance)) and (np.dot(self.heading_vector_cc, self.heading_vector_goal) > 0):
                self.robot_pos_switch = np.copy(self.robot_pos)
                self.mode = "followWallCC"

            if(self.mode == "followWallC" or self.mode == "followWallCC") and (self.obstacle_distance < (self.desired_distance - self.epsilon)):
                self.mode = "avoidObstacle"

            if (self.mode == "avoidObstacle") and (self.obstacle_distance > self.desired_distance) and (np.dot(self.heading_vector_c, self.heading_vector_goal) > 0):
                self.robot_pos_switch = np.copy(self.robot_pos)
                self.mode = "followWallC"

            if (self.mode == "avoidObstacle") and (self.obstacle_distance > self.desired_distance) and (np.dot(self.heading_vector_cc, self.heading_vector_goal) > 0):
                self.robot_pos_switch = np.copy(self.robot_pos)
                self.mode = "followWallCC"

            if((self.mode == "followWallC")or(self.mode == "followWallCC")) and (np.dot(self.heading_vector_goal, self.heading_vector_avoid_obstacle) > 0) and \
                    (np.linalg.norm(self.robot_pos-self.goal_pos) < (np.linalg.norm(self.robot_pos_switch-self.goal_pos)+0.9)) and\
                    (self.obstacle_distance > (self.desired_distance + self.epsilon)):
                self.mode = "goToGoal"

            if self.mode == "goToGoal":
                velocity = self.goToGoal()
            elif self.mode == "followWallC":
                velocity = self.follow_wall_c()
            elif self.mode == "followWallCC":
                velocity = self.follow_wall_cc()
            elif self.mode == "avoidObstacle":
                velocity = self.avoid_obstacle()

            self._vel_publisher.publish(velocity)

def main():
	rclpy.init()  
	goal_subs = goToGoal() 
	while rclpy.ok():
		rclpy.spin(goal_subs)
	goal_subs.destroy_node()
	rclpy.shutdown()



