import cv2
import rospy

import numpy as np
from sensor_msgs.msg import Image
from detector import StopSignDetector
class SignDetector:
    def __init__(self):
        self.detector = StopSignDetector()
        self.publisher = #TODO
        self.subscriber = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.callback)
        self.stop_found = False
        self.box = []

    def callback(self, img_msg):
        # Process image without CV Bridge
        np_img = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(img_msg.height, img_msg.width, -1)
        bgr_img = np_img[:,:,:-1]
        rgb_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB)
        self.stop_found, self.box = self.detector.predict(bgr_img)
        #TODO: 

    def stop_registered(self):
        self.stop_found = False

    def get_area(self):
        return (self.box[2] - self.box[0])* (self.box[3] - self.box[1])


if __name__=="__main__":
    rospy.init_node("stop_sign_detector")
    detect = SignDetector()
    rospy.spin()
