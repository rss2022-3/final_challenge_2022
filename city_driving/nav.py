import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
import numpy as np
from sensor_msgs.msg import Image
import os
import torch
import matplotlib.pyplot as plt
from PIL import Image, ImageDraw
#TODO: import line follower
from stop_detector.stop_detector import SignDetector
from stop_detector.detector import StopSignDetector
from rospy.numpy_msg import numpy_msg
from rospy.time import sleep
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_tools import *
import tf2_ros
import tf
from geometry_msgs.msg import Point
import tf2_geometry_msgs
from utilities.line_color_segmentation import line_color_segmentation
from visual_servoing.msg import ConeLocation, ParkingError
from utilities.controllers import PurePursuit
from utilities.Trajectory import LinearTrajectory


class CityNavigation:

    STOP_TOPIC = "/stop"
    SCAN_TOPIC = "/scan"
    DRIVE_TOPIC = "/drive"
    stop_size_thresh = 40 #TODO: change this - picked this number randomly

    def __init__(self):
        self.sub = rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.scan_callback)
        self.line_sub = rospy.Subscriber("/relative_line", Point, self.line_callback)
        self.stop_pos =rospy.Subscriber("/relative_stop", Point, self.stop_callback)
        self.pub = rospy.Publisher(self.Drive_TOPIC, AckermannDriveStamped, queue_size = 10)
        self.timer = rospy.Timer(rospy.Duration(1), self.time_callback)
        self.img_sub = rospy.Subscriber()
        self.Detector = SignDetector()
        self.alfredo = AckermannDriveStamped()
        #self.image_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.image_callback)
        self.parking_distance = .1 # meters; try playing with this number!
        self.relative_x = 0
        self.relative_y = 0
        self.stopped = False
        self.pursuit = PurePursuit(0.325)
        self.stop_dist = 0.6 #how far away it will stop from stop sign
        self.current_sign_distance = float("inf")
        self.now = 0
    c
    def v_function(self, v_desired, traj):
        #adaptive velocity function
        Lfw, lfw = 0, 0
        v_desired = abs(v_desired)
        if v_desired < 2:
            Lfw = v_desired
        elif v_desired >=2 and v_desired < 6:
            Lfw = 1.5*v_desired
        else:
            Lfw = 12
        return Lfw, lfw

    def scan_callback(self, data):
        pass

    def stop_callback(self, msg):
        self.current_sign_distance = np.sqrt(msg.x**2 + msg.y**2)

    def time_callback(self, timer):
        self.stopped = False

    def line_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos

        dist = np.sqrt(self.relative_x**2 + self.relative_y**2) - self.parking_distance
        heading = np.arctan2(self.relative_y, self.relative_x)

        x_d = np.cos(heading)*dist
        y_d = np.sin(heading)*dist
        #################################

        #generate trajectory 
        traj_knots = np.array([[0,0],
                            [x_d, y_d]])

        if abs(dist) >= 1:
            t_breaks = np.array([0,dist])
        #elif abs(dist) > 0.1:
        #    t_breaks = np.array([0,0.5])
        else:
            t_breaks = np.array([0,1])


        trj_d = LinearTrajectory(t_breaks, traj_knots)
        
        steer, speed = self.pursuit.adaptiveControl(trj_d, self.v_function)
        drive_cmd = self.drive(steer, speed, None, None)

        #################################

        if(self.Detector.stop_found and self.stop_dist > self.current_sign_distance and not self.stopped):
            if self.now == 0:
                self.now = rospy.get_time()
            drive_cmd = self.drive(steer, 0, None, None)
            #rospy.sleep(1)#timer should have a callback function
            self.stopped = True
            self.Detector.stop_registered()
            if rospy.get_time() - self.prev_time > 1:
                self.stopped = True
                drive_cmd = self.drive(steer, speed, None, None)
        
        # make sure we leave the stop sign before resetting
        if not (self.Detector.stop_found and self.current_sign_distance > 0.6):
            self.stopped = False
            self.prev_time = 0
        
        self.pub.publish(drive_cmd)
        

    def drive(self, theta, speed, theta_dot = None,  acceleration = None):
        """
            Takes in steering and speed commands for the car.
            :param theta: steering angle [rad]. right sided
            :type theta: float
            :param theta_dot: steering angular velocity [rad/s]
            :type theta_dot: float
            :param speed: speed in [m/s]
            :type speed: float

            :param speed: speed in [m/s]
            :type speed: float
        """
        ack_msg = AckermannDriveStamped()
        ack_msg.header.stamp = rospy.get_rostime()
        ack_msg.header.frame_id = 'base_link'
        ack_msg.drive.steering_angle = np.clip(theta, -0.34, 0.34)
        ack_msg.drive.speed = speed
        if not theta_dot is None:
            ack_msg.drive.steering_angle_velocity = theta_dot
        else:
            ack_msg.drive.steering_angle_velocity = 0.1
        if not acceleration is None:
            ack_msg.drive.acceleration = acceleration
        return ack_msg

if __name__ == '__main__':
    try:
        rospy.init_node('CityNavigation', anonymous=True)
        CityNavigation()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

        
        
        
