import sys
import os
import sensor_msgs.msg as sensor_msgs
import struct
import ctypes
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt
import rclpy
from rclpy.clock import ROSClock
from rclpy.node import Node
import open3d as o3d
import numpy as np
# Own files
import params 
from pcd_image_overlay import create_point_cloud_image_overlay
import pointcloud2_to_pcd_file  

### Class definition
class Img_PCD_Subscriber_Overlay_Publisher(Node):
    glob_cv_image_front = None
    glob_pcd_file = None
    ros_image_point_cloud_overlay = None
    def __init__(self):
        super().__init__('img_pcd_subscriber_overlay_publisher')

        # Subscribe to pcd file
        self.pcd_file = self.create_subscription(
            sensor_msgs.PointCloud2,
            params.sub_topic_pcd,  # Subscribes from pcd_publisher 
            self.sub_callback_pcd,
            1)
        self.pcd_file  # Prevent unused variable warning

        # Subscribe to image file
        self.ros_img_front = self.create_subscription(
            sensor_msgs.Image,
            params.sub_topic_image,  # Subscribes from image publisher
            self.sub_callback_img,
            1)
        self.ros_img_front  # Prevent unused variable warning

        # Publish overlay as an image
        self.overlay_publisher = self.create_publisher(
            sensor_msgs.Image,
            params.pub_topic_overlay,
            1)
        self.publish_timestep = params.overlay_publish_timestep 
        self.timer_overlay = self.create_timer(self.publish_timestep, self.pub_callback)    

        self.bridge = CvBridge()

    # Image subscriber callback function     
    def sub_callback_img(self, Image):
        Img_PCD_Subscriber_Overlay_Publisher.glob_cv_image_front = None
        try: 
            Img_PCD_Subscriber_Overlay_Publisher.glob_cv_image_front  = self.bridge.imgmsg_to_cv2(Image)
        except CvBridgeError as e:
            print(e)
        # print("Subscribed to image.") 

    # Pointcloud2 file subscriber callback function
    def sub_callback_pcd(self, PointCloud2 ):
        # The 'msg', which is of the type PointCloud2 is converted to a pcd and finally to an array.
        # The function read_points2 is ported from the ROS1 package below. 
        # https://github.com/ros/common_msgs/blob/noetic-devel/sensor_msgs/src/sensor_msgs/point_cloud2.py
        Img_PCD_Subscriber_Overlay_Publisher.glob_pcd_file = None
        gen = pointcloud2_to_pcd_file.read_points(PointCloud2, skip_nans=True)
        Img_PCD_Subscriber_Overlay_Publisher.glob_pcd_file = np.array(list(gen))
        # print("Subscribed to pcd.")

    # Overlay publisher callback function
    def pub_callback(self):
        cv_image_point_cloud_overlay = None
        # Overlay image and pointcloud
        cv_image_point_cloud_overlay = create_point_cloud_image_overlay\
            (Img_PCD_Subscriber_Overlay_Publisher.glob_pcd_file, Img_PCD_Subscriber_Overlay_Publisher.glob_cv_image_front)
        # Convert overlay to ros msg image format
        try:
            ros_image_point_cloud_overlay = self.bridge.cv2_to_imgmsg(cv_image_point_cloud_overlay, "rgb8")
        except CvBridgeError as e:
            print(e) 
        ros_image_point_cloud_overlay.header.frame_id = "camera_front_point_cloud_overlay"
        clock = ROSClock()
        timestamp = clock.now()
        ros_image_point_cloud_overlay.header.stamp = timestamp.to_msg()
        # print("Overlay published.")
        self.overlay_publisher.publish(ros_image_point_cloud_overlay)

def main(args = None):
    rclpy.init(args = args)
    img_pcd_sub_overlay_pub = Img_PCD_Subscriber_Overlay_Publisher()
    rclpy.spin(img_pcd_sub_overlay_pub)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
