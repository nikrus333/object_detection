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
        self.graf = GrafWork()
        self.br = CvBridge()
        self.k = 0.52
        self.strat = time.time()
        super().__init__('minimal_subscriber')
        self.sub_image = self.create_subscription( Image, '/camera/rgb/image_color', self.listener_callback, qos_profile_sensor_data) 
        self.sub__depth_image = self.create_subscription( Image, '/camera/depth/image', self.depth_callback, qos_profile_sensor_data) 
        #self.publisher_image = self.create_publisher(Image, '/face_frames', 10)  
        self.start = time.time()
        self.base_frame = None
        self.k = 0.55
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s')

        self.depth_image = None  
        
    def listener_callback(self, data):
        current_frame = self.br.imgmsg_to_cv2(data)
        results = self.model(current_frame)
        # Results
        #results.print()  # or .show(), .save(), .crop(), .pandas(), etc.
        #self.get_logger().info('Publishing: "%s"' % self.msg)
        #print('\n', results.xyxy[0])
        #print(results.pandas().xyxy[0])
        name = results.pandas().xyxy[0]['name']
        #print(len(name))
        for count in range(len(name)):
            print(results.pandas().xyxy[0]['name'][count])
            x_center = int(results.pandas().xyxy[0]['xmax'][count] - results.pandas().xyxy[0]['xmin'][count])
            y_center = int(results.pandas().xyxy[0]['ymax'][count] - results.pandas().xyxy[0]['ymin'][count])
            print(x_center, y_center)
        self.graf.append_graf(name)

        #print(len(self.graf.list_graf))
    def depth_callback(self, data):
        self.depth_image = data


    # NO ROS PART
    def depth_solution(self, pixel_cord):
        CX_DEPTH = 319.5
        FX_DEPTH = 525.0
        CY_DEPTH = 239.5
        FY_DEPTH = 525.0

        current_frame = self.br.imgmsg_to_cv2(self.depth_image)
        height, width = current_frame.shape
        # for i in range(height):
        #     for j in range(width):
        #         z = current_frame[i][j]
        #         x = (j - CX_DEPTH) * z / FX_DEPTH
        #         y = (i - CY_DEPTH) * z / FY_DEPTH
        z = current_frame[pixel_cord[0]][pixel_cord[1]]
        x = (pixel_cord[0] - CX_DEPTH) * z / FX_DEPTH
        y = (pixel_cord[1] - CY_DEPTH) * z / FY_DEPTH

        return (x, y, z)
        



        

class GrafWork():
    def __init__(self) -> None:
        self.list_graf = []
        
        pass
    def append_graf(self, graf):
        graf = graf
        self.list_graf.append(graf)
    def get_graf(self):
        pass
    
    class Graf():
        vertices = {"A", "B", "C", "D", "E", "F"}
        edges = {("A", "D"), ("A", "B"), ("A", "E"), ("A", "F"), ("B", "F"), ("B", "C")}


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = FaceRecognition()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
