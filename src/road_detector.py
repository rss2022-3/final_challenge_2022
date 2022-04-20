#!/usr/bin/env python

import numpy as np
import rospy

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, PoseArray #geometry_msgs not in CMake file

# import your color segmentation algorithm; call this function in ros_image_callback!
from computer_vision.line_color_segmentation import line_color_segmentation


class RoadDetector():
    """
    A class for applying your cone detection algorithms to the real robot.
    Subscribes to: /zed/zed_node/rgb/image_rect_color (Image) : the live RGB image from the onboard ZED camera.
    Publishes to: /relative_cone_px (ConeLocationPixel) : the coordinates of the cone in the image frame (units are pixels).
    """
    def __init__(self):
        # toggle line follower vs cone parker
        self.LineFollower = False

        # Subscribe to ZED camera RGB frames
        self.traj_pub = rospy.Publisher("/trajectory", PoseArray, queue_size=10)
        self.debug_pub = rospy.Publisher("/debug_img", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.image_callback)
        self.bridge = CvBridge() # Converts between ROS images and OpenCV Images

    def image_callback(self, image_msg):
        # Apply your imported color segmentation function (cd_color_segmentation) to the image msg here
        # From your bounding box, take the center pixel on the bottom
        # (We know this pixel corresponds to a point on the ground plane)
        # publish this pixel (u, v) to the /relative_cone_px topic; the homography transformer will
        # convert it to the car frame.

        image_msg = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        
        image_msg = np.asarray(image_msg)

        bounding_box = line_color_segmentation(image_msg)
        if not bounding_box is None:
            u = (bounding_box[0][0] + bounding_box[1][0])/2
            v = bounding_box[1][1]
            cone_location = ConeLocationPixel()
            cone_location.u = u
            cone_location.v = v
            self.cone_pub.publish(cone_location)

        #image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")

        #debug_msg = self.bridge.cv2_to_imgmsg(image, desired_encoding="bgr8")
        #self.debug_pub.publish(debug_msg)


        point_list = [] # (x,y) in relative frame of car

        traj = PoseArray()
        traj.header = self.make_header("/map")
        for x,y in point_list:
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            traj.poses.append(pose)
        # publish trajectory
        self.traj_pub.publish(traj)

if __name__ == '__main__':
    try:
        rospy.init_node('RoadDetector', anonymous=True)
        RoadDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
