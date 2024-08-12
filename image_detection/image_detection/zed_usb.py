#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
import cv2
from rclpy.node import Node
from cv_bridge import CvBridge
import numpy as np
from sensor_msgs.msg import Image
from ultralytics import YOLO


class ImageSubscriber(Node):
    def __init__(self,name):
        super().__init__(name)
        self.sub = self.create_subscription(Image,"/camera1/image_raw",self.listener_callback,10)
        self.model = YOLO("/home/jiewang/yolov8/yolov8n-seg.onnx")
        self.cv_bridge = CvBridge()
        self.pub = self.create_publisher(Image,"/perception/traffic_light_recognition/traffic_light/debug/rois",1)
        self.timer = self.create_timer(0.01,self.timer_callback)
    

    def timer_callback(self):
        pass


    def listener_callback(self,data):
        self.get_logger().info("Receiving video frame")
        image = self.cv_bridge.imgmsg_to_cv2(data,'bgr8')
        results = self.model.predict(image,classes=[0,1,2,3,5],stream=True,device='cpu')
        for result in results:
            img = result.plot()
        img_result = self.cv_bridge.cv2_to_imgmsg(img,"bgr8")
        self.pub.publish(img_result)


def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber("zed_image_sub")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown() 