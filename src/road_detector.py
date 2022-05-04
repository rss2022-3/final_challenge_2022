#!/usr/bin/env python

import numpy as np
import rospy

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, PoseArray #geometry_msgs not in CMake file
from std_msgs.msg import Header

# import your color segmentation algorithm; call this function in ros_image_callback!
#from computer_vision.line_color_segmentation import line_color_segmentation
from homography_transformer import HomographyTransformer
from computer_vision.hough_transform import get_trajectory
from utilities.average import RunningAvg


class RoadDetector():
    """
    A class for applying your cone detection algorithms to the real robot.
    Subscribes to: /zed/zed_node/rgb/image_rect_color (Image) : the live RGB image from the onboard ZED camera.
    Publishes to: /relative_cone_px (ConeLocationPixel) : the coordinates of the cone in the image frame (units are pixels).
    """
    def __init__(self):
        # toggle line follower vs cone parker
        self.LineFollower = False
        self.homography = HomographyTransformer()
        self.runningavg = RunningAvg()

        # Subscribe to ZED camera RGB frames
        self.traj_pub = rospy.Publisher("/trajectory", PoseArray, queue_size=10)
        self.debug_pub = rospy.Publisher("/debug_img", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.image_callback)
        self.bridge = CvBridge() # Converts between ROS images and OpenCV Images

    def draw_line(self, img, rho, theta, color, thickness = 1):
        height = img.shape[0]
        bottom = rho/np.cos(theta)
        top = (rho/np.sin(theta)-height)*np.tan(theta)
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a*rho
        y0 = b*rho
        x1 = int(x0 + 1000*(-b))
        y1 = int(y0 + 1000*(a))
        x2 = int(x0 - 1000*(-b))
        y2 = int(y0 - 1000*(a))
        cv2.line(img,(x1,y1),(x2,y2),color,thickness)


    def image_callback(self, image_msg):
        # Apply your imported color segmentation function (cd_color_segmentation) to the image msg here
        # From your bounding box, take the center pixel on the bottom
        # (We know this pixel corresponds to a point on the ground plane)
        # publish this pixel (u, v) to the /relative_cone_px topic; the homography transformer will
        # convert it to the car frame.

        image_msg = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        
        img = np.asarray(image_msg)
        # image_msg = cv2.imread("computer_vision/test_images_input/image (4).png")

        #stack = get_trajectory(image_msg)

        original_height = img.shape[0]
        img = img[int(img.shape[0]/3):, :]
        height = img.shape[0]
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)  

        lower_threshold = 350 #TUNE
        upper_threshold = 500 #TUNE
        edges = cv2.Canny(gray,lower_threshold,upper_threshold,apertureSize = 3) 

        min_intersections = 100 #TUNE                         
        lines = cv2.HoughLines(edges,1,np.pi/180,min_intersections)    

        min_left_theta, max_right_theta = np.Inf, 0
        left, right = None, None
        for i in range(lines.shape[0]):                         
            for rho,theta in lines[i]:
                if rho > 0 and theta < min_left_theta:
                    min_left_theta = theta
                    left = (rho,theta)
                elif rho < 0 and theta > max_right_theta:
                    max_right_theta = theta
                    right = (rho,theta)
                self.draw_line(img, rho, theta, (0,0,255))

        self.draw_line(img, left[0], left[1], (0,255,0))
        self.draw_line(img, right[0], right[1], (0,255,0))

        top_right = right[0]/np.cos(right[1])
        top_left = left[0]/np.cos(left[1])
        bottom_right = (right[0]/np.sin(right[1])-height)*np.tan(right[1])
        bottom_left = (left[0]/np.sin(left[1])-height)*np.tan(left[1])

        top = (top_right + top_left)/2
        bottom = (bottom_right + bottom_left)/2

        cv2.line(img,(int(bottom),height),(int(top),0),(255,0,0),1)

        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(img, encoding='bgr8'))

        stack = np.stack([np.array([bottom,original_height]), np.array([top,original_height-height])])


        self.runningavg.add(stack)
        new_stack = self.runningavg.get()
        if new_stack is not None:
            stack = new_stack
        
        bottom = stack[0, :]
        top = stack[1, :]

        # print(top, bottom, "lalala")
        num_points = 25
        point_list = [] # (x,y) in relative frame of car
        for step in range(num_points):
            uv = bottom + (top-bottom)*(step / float(num_points-1))
            xy = self.homography.transformUvToXy(uv[0], uv[1])
            if xy[0] > 0:
                point_list.append(xy)
                
        # print(point_list[0], point_list[-1])

        #bounding_box = line_color_segmentation(image_msg)
        #if not bounding_box is None:
        #    u = (bounding_box[0][0] + bounding_box[1][0])/2
        #    v = bounding_box[1][1]
        #    cone_location = ConeLocationPixel()
        #    cone_location.u = u
        #    cone_location.v = v
        #    self.cone_pub.publish(cone_location)

        #image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")

        #debug_msg = self.bridge.cv2_to_imgmsg(image, desired_encoding="bgr8")
        #self.debug_pub.publish(debug_msg)
        traj = PoseArray()
        traj.header = self.make_header("map")
        
        for x,y in point_list:
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            traj.poses.append(pose)
        # publish trajectory
        self.traj_pub.publish(traj)
    def make_header(self, frame_id, stamp=None):
        if stamp == None:
            stamp = rospy.Time.now()
        header = Header()
        header.stamp = stamp
        header.frame_id = frame_id
        return header

if __name__ == '__main__':
    try:
        rospy.init_node('RoadDetector', anonymous=True)
        RoadDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
