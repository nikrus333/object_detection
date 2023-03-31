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
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class FaceRecognition(Node):
    def __init__(self):
        self.graf = GrafWork()
        self.br = CvBridge()
        self.strat = time.time()
        super().__init__('minimal_subscriber')
        self.sub_image = self.create_subscription( Image, '/camera/rgb/image_color', self.listener_callback, qos_profile_sensor_data) 
        self.sub__depth_image = self.create_subscription( Image, '/camera/depth/image', self.depth_callback, qos_profile_sensor_data) 

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        #self.publisher_image = self.create_publisher(Image, '/face_frames', 10)  
        self.start = time.time()
        self.base_frame = None
    
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s')
        
        self.k = 0.52
        self.depth_image = None  
        self.count = 0
        
    def listener_callback(self, data):
        self.count += 1 
        if self.count % 5 == 1:
            try:
                t = self.tf_buffer.lookup_transform('world_1', 'camera', rclpy.time.Time(),   timeout=rclpy.duration.Duration(seconds=1.0))
                coordinate_camera = (t.transform.translation.x, t.transform.translation.y, t.transform.translation.z)
            except:
                coordinate_camera = (0,0,0)
            print(coordinate_camera)

            current_frame = self.br.imgmsg_to_cv2(data)
            results = self.model(current_frame)
            # Results
            #results.print()  # or .show(), .save(), .crop(), .pandas(), etc.
            #self.get_logger().info('Publishing: "%s"' % self.msg)
            #print('\n', results.xyxy[0])
            #print(results.pandas().xyxy[0])
            arr_name = []
            arr_coordinates_object = []
            name = results.pandas().xyxy[0]['name']
            #print(len(name))
            for count in range(len(name)):
                name_obj = results.pandas().xyxy[0]['name'][count]
                x_center = int(results.pandas().xyxy[0]['xmax'][count] - results.pandas().xyxy[0]['xmin'][count])
                y_center = int(results.pandas().xyxy[0]['ymax'][count] - results.pandas().xyxy[0]['ymin'][count])
                x, y, z = self.depth_solution([x_center, y_center])
                arr_name.append(name_obj)
                arr_coordinates_object.append([x, y, z])
                #print(x, y, z)
                
            graf_one = self.graf.Graf(arr_name, arr_coordinates_object, coordinate_camera)
            if graf_one.valid == True:
                pass
                #print(graf_one.get_graf())
            #print(self.graf.list_graf)

        #print(len(self.graf.list_graf))
    def depth_callback(self, data):
        self.depth_image = data


    # NO ROS PART
    def depth_solution(self, pixel_cord):
        CX_DEPTH = 319.5
        FX_DEPTH = 525.0
        CY_DEPTH = 239.5
        FY_DEPTH = 525.0
        try:
            current_frame = self.br.imgmsg_to_cv2(self.depth_image)  
            height, width = current_frame.shape
            z = current_frame[pixel_cord[0]][pixel_cord[1]]
            x = (pixel_cord[0] - CX_DEPTH) * z / FX_DEPTH
            y = (pixel_cord[1] - CY_DEPTH) * z / FY_DEPTH
        except:
            x, y, z = (0,0,0)
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

    def searh_graf(self, list_graf, curent_graf):
        # first match 
        names_current_graf, edges_current_graf = curent_graf
        favorite_graf = []
        best_accurasy = 0
        best_graf = None
        count = 0
        for names_list_graf, _ in list_graf:
            result=list(set(names_current_graf) & set(names_list_graf))
            accuracy = len(result) / len(names_current_graf)
            if accuracy > 0.7:
                favorite_graf.append(list_graf[count])
                if best_accurasy < accuracy:
                    best_accurasy = accuracy
                    best_graf = list_graf[count]
            count += 1
                        
    class Graf():
        # def __new__(cls, item):
        #     if cls.IsValid(item):
        #         return super(SampleObject, cls).__new__(cls)
        #     else:
        #         return None
            
        def __init__(self, arr_name, arr_coordinates, coordinate_camera) -> None:
            self.vertices = arr_name
            self.valid = True
            self.edge = self.create_edges(arr_name, arr_coordinates)
            self.coordinates = coordinate_camera
            print('init')

        def create_edges(self, arr_name, arr_coordinates):
            count = len(arr_name)
            arr_name_copy = arr_name
            edge_dict = {}
            edge_arr = []
            for i in range(len(arr_name)):
                for temp in range(len(arr_name)):
                    distanse = math.sqrt((arr_coordinates[i][0]-arr_coordinates[temp][0])**2 + (arr_coordinates[i][1]-arr_coordinates[temp][1])**2 + (arr_coordinates[i][2]-arr_coordinates[temp][2])**2)
                    if math.isnan(distanse):
                        self.valid = False
                    edge_dict[arr_name[temp]] = distanse

                edge_arr.append({arr_name_copy[i] : edge_dict})
            return edge_arr
                
        def get_graf(self):
            return (self.vertices, self.edge)
    

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = FaceRecognition()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
