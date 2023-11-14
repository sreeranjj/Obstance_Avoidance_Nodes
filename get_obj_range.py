#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""

@author: abirath
"""
###############################################################################################################################################################################
# Team Vegeta - Abirath Raju, Sreeranj Jayadevan
###############################################################################################################################################################################
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, LaserScan 
from std_msgs.msg import Float64
from geometry_msgs.msg import Point, Vector3
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
import sys
import numpy as np
import cv2
from cv_bridge import CvBridge


class getObjectRange(Node):
    def __init__(self):
        super().__init__('get_obj_range')
        print("Inside loop")
        image_qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, durability=QoSDurabilityPolicy.VOLATILE, depth=1)
        self._lidar_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, image_qos_profile)
        self._lidar_subscriber 
        self._angle_publisher = self.create_publisher(Vector3, '/obs_pos', 10)

    def scan_callback(self, scan):
        vector = Vector3()
        vector.z = 0.0
        min_angle = scan.angle_min
        lidar_arr = np.array(scan.ranges)
        lidar_arr = np.where(np.isnan(lidar_arr), 100.0, lidar_arr)
        angle_increments = scan.angle_increment
        min_index = np.argmin(lidar_arr)
        distance = lidar_arr[min_index]
        angle = min_angle + min_index * angle_increments
        vector.x = distance * np.cos(angle)
        vector.y = distance * np.sin(angle)
        self._angle_publisher.publish(vector)

def main():
	rclpy.init()  
	pos_subscriber = getObjectRange() 
	while rclpy.ok():
		rclpy.spin(pos_subscriber)
	pos_subscriber.destroy_node()
	rclpy.shutdown()
