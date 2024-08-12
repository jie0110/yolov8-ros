#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
import cv2
from rclpy.node import Node
from cv_bridge import CvBridge
import numpy as np
from sensor_msgs.msg import Image
import message_filters
from ultralytics import YOLO
import torch


class ImageSubscriber(Node):
    def __init__(self,name):
        super().__init__(name)
        # self.rgb = message_filters.Subscriber(self,Image,"/zed/zed_node/rgb/image_rect_color")
        self.rgb = message_filters.Subscriber(self,Image,"/camera/camera/color/image_raw")
        # self.depth = message_filters.Subscriber(self,Image,"/zed/zed_node/depth/depth_registered")
        self.depth = message_filters.Subscriber(self,Image,"/camera/camera/depth/image_rect_raw")

        self.model = YOLO("/home/jiewang/yolov8/yolov8n-seg.pt")
        self.sub = message_filters.ApproximateTimeSynchronizer([self.rgb,self.depth],1,0.1,allow_headerless=True)
        self.sub.registerCallback(self.listener_callback)
        self.cv_bridge = CvBridge()
        self.pub = self.create_publisher(Image,"/perception/traffic_light_recognition/traffic_light/debug/rois",1)
        # self.timer = self.create_timer(1,self.timer_callback)

    def object_detect(self,color,depth):
        depth_image = np.array(depth, dtype=np.float32)
        depth_image = depth_image * 1000
        depth_image = np.round(depth_image).astype(np.uint16)
        results = self.model.predict(color,classes=[0,1,2,3,5],stream=True,device='cpu')
        for result in results:
        #     # xywh_list = result.boxes.xywh
        #     # for xywh in xywh_list:
        #     #     x=int(xywh[0])
        #     #     y=int(xywh[1])
        #     #     # Z=depth_image[y,x]
        #     #     # cv2.circle(color,(x,y),10,[0,255,0],-1)
        #     #     # cv2.putText(color,str(Z),(x,y),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),2)
            img = result.plot()
        # # cv2.imshow("result",img)
        # # cv2.waitKey(10)
        img_result = self.cv_bridge.cv2_to_imgmsg(img,"bgr8")
        self.pub.publish(img_result)

       

    def listener_callback(self,rgb,dph):
        self.get_logger().info("Receiving video frame")
        color = self.cv_bridge.imgmsg_to_cv2(rgb,'bgr8')
        depth = self.cv_bridge.imgmsg_to_cv2(dph,'32FC1')
        self.object_detect(color,depth)


def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber("zed_image_sub")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown() 