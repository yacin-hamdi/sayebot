#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np



class LaneFollower(Node):
    def __init__(self):
        super().__init__("lane_follower")
        self.bridge = CvBridge()
        self.image_sub_ = self.create_subscription(Image, 
                                                   "/camera", 
                                                   self.image_callback,
                                                   10)
        
    

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        h, w, _ = frame.shape
        roi = frame[int(h*0.5):, :]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask_white = cv2.inRange(hsv, (0, 0, 200), (180, 40, 255))
        kernel = np.ones((5, 5), np.uint8)
        mask_white = cv2.dilate(mask_white, kernel, iterations=1)
        cv2.imshow("white mask", mask_white)
        cv2.waitKey(1)




def main():
    rclpy.init()
    node = LaneFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

