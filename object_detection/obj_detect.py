import argparse
import cv2
import numpy as np
import torch
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import String

class FaceRecognition(Node):
    def __init__(self):
        self.br = CvBridge()
        self.k = 0.52
        self.strat = time.time()
        super().__init__('minimal_subscriber')
        self.sub_image = self.create_subscription( Image, '/video_frames', self.listener_callback, qos_profile_sensor_data) 
        self.publisher_image = self.create_publisher(Image, '/face_frames', 10)  
        self.start = time.time()
        self.base_frame = None
        self.k = 0.55
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s')  
        
    def listener_callback(self, data):
        current_frame = self.br.imgmsg_to_cv2(data)
        results = self.model(current_frame)
        # Results
        results.print()  # or .show(), .save(), .crop(), .pandas(), etc.
        #self.get_logger().info('Publishing: "%s"' % self.msg)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = FaceRecognition()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
