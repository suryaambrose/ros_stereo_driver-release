#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import yaml
import roslib
from sensor_msgs.msg import CameraInfo, Image
import rospy

class CameraInfoPublisher: 
# Callback of the ROS subscriber. 
    def callback(self, data):
        self.cam_info.header = data.header
        self.publish()

    def __init__(self):
        file_name  = rospy.get_param('~file_name', '')
        camera_name  = rospy.get_param('~camera_name', '')
        out_topic_name  = camera_name + "/camera_info"
        in_topic = camera_name + "/image_raw"
        
        self.cam_info = parse_yaml(file_name)

        rospy.Subscriber(in_topic, Image, self.callback)
        self.pub = rospy.Publisher(out_topic_name,CameraInfo, queue_size=2)

    def publish(self):
        '''
        now = rospy.Time.now()
        self.cam_info.header.stamp = now
        '''
        self.pub.publish(self.cam_info)

def parse_yaml(filename):
    stream = file(filename, 'r')
    calib_data = yaml.load(stream)
    cam_info = CameraInfo()
    cam_info.width = calib_data['image_width']
    cam_info.height = calib_data['image_height']
    cam_info.K = calib_data['camera_matrix']['data']
    cam_info.D = calib_data['distortion_coefficients']['data']
    cam_info.R = calib_data['rectification_matrix']['data']
    cam_info.P = calib_data['projection_matrix']['data']
    cam_info.distortion_model = calib_data['distortion_model']
    return cam_info

if __name__ == '__main__':
    rospy.init_node("camera_info_publisher")
    publisher = CameraInfoPublisher()
    rospy.spin()
