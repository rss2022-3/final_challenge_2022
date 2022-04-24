import rospy
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError

from geometry_msgs.msg import Point 
from std_msgs.msg import String
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from visual_servoing.msg import ConeLocation, ConeLocationPixel

#The following collection of pixel locations and corresponding relative
#ground plane locations are used to compute our homography matrix

# PTS_IMAGE_PLANE units are in pixels
# see README.md for coordinate frame description

######################################################
## DUMMY POINTS -- ENTER YOUR MEASUREMENTS HERE
PTS_IMAGE_PLANE = [[498, 264],
                   [400, 228],
                   [244, 280],
                   [113, 262]] 
######################################################

# PTS_GROUND_PLANE units are in inches
# car looks along positive x axis with positive y axis to left

######################################################
## DUMMY POINTS -- ENTER YOUR MEASUREMENTS HERE
PTS_GROUND_PLANE = [[21.0, -10.0],
                    [32.0, -7.5],
                    [18.5, 7.0],
                    [22.0, 16.5]] 
######################################################

METERS_PER_INCH = 0.0254


class HomographyTransformer:
    def __init__(self):
        self.line_px_sub = rospy.Subscriber("/relative_line_px", Point, self.line_detection_callback)
        self.line_pub = rospy.Publisher("/relative_line", Point, queue_size=10)

        self.stop_sub = rospy.Subscriber("/relative_stop_px", Point, self.stop_detect_callback)

        self.marker_pub = rospy.Publisher("/cone_marker",
            Marker, queue_size=1)

        if not len(PTS_GROUND_PLANE) == len(PTS_IMAGE_PLANE):
            rospy.logerr("ERROR: PTS_GROUND_PLANE and PTS_IMAGE_PLANE should be of same length")

        #Initialize data into a homography matrix

        np_pts_ground = np.array(PTS_GROUND_PLANE)
        np_pts_ground = np_pts_ground * METERS_PER_INCH
        np_pts_ground = np.float32(np_pts_ground[:, np.newaxis, :])

        np_pts_image = np.array(PTS_IMAGE_PLANE)
        np_pts_image = np_pts_image * 1.0
        np_pts_image = np.float32(np_pts_image[:, np.newaxis, :])

        self.h, err = cv2.findHomography(np_pts_image, np_pts_ground)

    def line_detection_callback(self, msg):
        #Extract information from message
        u = msg.x
        v = msg.y

        #Call to main function
        x1, y1 = self.transformUvToXy(u, v)

        #Publish relative xy position of object in real world
        relative_xy_msg = Point()
        relative_xy_msg.x = x1
        relative_xy_msg.y = y1

        self.line_pub.publish(relative_xy_msg)
        self.draw_marker(x1, y1, "map")

    def stop_detect_callback(self, msg):
        homogeneous_point = np.array([[u], [v], [1]])
        xy = np.dot(self.h, homogeneous_point)
        scaling_factor = 1.0 / xy[2, 0]
        homogeneous_xy = xy * scaling_factor
        x = homogeneous_xy[0, 0]
        y = homogeneous_xy[1, 0]
        return x, y


    def transformUvToXy(self, u, v):
        """
        u and v are pixel coordinates.
        The top left pixel is the origin, u axis increases to right, and v axis
        increases down.

        Returns a normal non-np 1x2 matrix of xy displacement vector from the
        camera to the point on the ground plane.
        Camera points along positive x axis and y axis increases to the left of
        the camera.

        Units are in meters.
        """
        homogeneous_point = np.array([[u], [v], [1]])
        xy = np.dot(self.h, homogeneous_point)
        scaling_factor = 1.0 / xy[2, 0]
        homogeneous_xy = xy * scaling_factor
        x = homogeneous_xy[0, 0]
        y = homogeneous_xy[1, 0]
        return x, y

    def draw_marker(self, cone_x, cone_y, message_frame):
        """
        Publish a marker to represent the cone in rviz.
        (Call this function if you want)
        """
        marker = Marker()
        marker.header.frame_id = message_frame
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.scale.x = .2
        marker.scale.y = .2
        marker.scale.z = .2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = .5
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = cone_x
        marker.pose.position.y = cone_y
        self.marker_pub.publish(marker)


if __name__ == "__main__":
    rospy.init_node('homography_transformer')
    homography_transformer = HomographyTransformer()
    rospy.spin()
