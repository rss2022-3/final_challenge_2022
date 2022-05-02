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

    def image_callback(self, image_msg):
        # Apply your imported color segmentation function (cd_color_segmentation) to the image msg here
        # From your bounding box, take the center pixel on the bottom
        # (We know this pixel corresponds to a point on the ground plane)
        # publish this pixel (u, v) to the /relative_cone_px topic; the homography transformer will
        # convert it to the car frame.

        image_msg = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        
        image_msg = np.asarray(image_msg)
        # image_msg = cv2.imread("computer_vision/test_images_input/image (4).png")

        stack = get_trajectory(image_msg)
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
        print("POINT LIST: ", point_list)
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


    def get_closest_line(self, img, traj, thresh):
        """Mitigates multiple trajectory error and reduces oscillation
           down the track by eliminating trajectories who's distance
           from the center is too large (i.e. trajectories computed from lines
           in other lanes). 

           Inputs: OpenCV image object, trajectory object after homography,
                   threshold value for maximum distance from desired trajectory.
           
           Outputs: The current trajectory if the mean distance of the portion 
                    of the trajectory to the middle line is less than or equal
                    to the threshold. Otherwise, returns previous trajectory.

        """
        
        ###TODO Take homography of middle lane in order to measure distance
        #to trajectory values.
        ###TODO Determine tresh value experimentally.
        ###TODO declare self.prev_traj variable in __init__ function. 

        width, _ = img.shape
        middle = width/2 

        mini_traj = np.array(traj) #Fix this
        middle_line = np.array(middle) #Fix this 

        mean_diff = np.abs(mini_traj - middle_line)/mini_traj.shape[0]

        if mean_diff > thresh:
            return self.prev_traj
        else:
            return traj



if __name__ == '__main__':
    try:
        rospy.init_node('RoadDetector', anonymous=True)
        RoadDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
