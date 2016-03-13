/*********************************************************************
* Copyright 2015 Surya Ambrose

* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0

* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
* implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*********************************************************************/

#include "stereo_driver/deinterlace.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <image_transport/camera_publisher.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <cv_bridge/cv_bridge.h>

namespace stereo_driver {

Deinterlacer::Deinterlacer(ros::NodeHandle nh, ros::NodeHandle priv_nh)
  : nh_(nh), priv_nh_(priv_nh),
    it_(nh_),
    cinfo_manager_(nh) {
  cam_left_pub_ = it_.advertiseCamera("left/image_raw", 1, false);
  cam_right_pub_ = it_.advertiseCamera("right/image_raw", 1, false);
  priv_nh_.getParam("camera_left_info_url",camera_left_info_url_);
  priv_nh_.getParam("camera_right_info_url",camera_right_info_url_);
  sub_ = it_.subscribe("/camera/image_raw", 1, &Deinterlacer::imageCb, this);
}

Deinterlacer::~Deinterlacer() {
}

void Deinterlacer::imageCb(const sensor_msgs::ImageConstPtr& image_msg){

  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(image_msg);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  sensor_msgs::ImagePtr imageL(new sensor_msgs::Image());
  sensor_msgs::ImagePtr imageR(new sensor_msgs::Image());
  sensor_msgs::CameraInfoPtr cinfoL;
  sensor_msgs::CameraInfoPtr cinfoR;

  cv::Mat left_frame, right_frame;
  cv::cvtColor(cv_ptr->image, left_frame, cv::COLOR_YUV2GRAY_YUYV);
  cv::cvtColor(cv_ptr->image, right_frame, cv::COLOR_YUV2GRAY_UYVY);

  imageL->width = image_msg->width;
  imageL->height = image_msg->height;
  imageL->encoding = "mono8";
  imageL->step = imageL->width;
  imageL->data.resize(imageL->step * imageL->height);
  memcpy(&(imageL->data[0]), &(left_frame.data[0]), left_frame.step*left_frame.rows);

  cinfo_manager_.loadCameraInfo(camera_left_info_url_);
  cinfoL = sensor_msgs::CameraInfoPtr(new sensor_msgs::CameraInfo(cinfo_manager_.getCameraInfo()));

  imageL->header.frame_id = "Stereo_Cam";
  imageL->header.stamp = image_msg->header.stamp;
  cinfoL->header.frame_id = "Stereo_Cam";
  cinfoL->header.stamp = image_msg->header.stamp;

  imageR->width = image_msg->width;
  imageR->height = image_msg->height;
  imageR->encoding = "mono8";
  imageR->step = imageR->width;
  imageR->data.resize(imageR->step * imageR->height);
  memcpy(&(imageR->data[0]), &(right_frame.data[0]), right_frame.step*right_frame.rows);

  cinfo_manager_.loadCameraInfo(camera_right_info_url_);
  cinfoR = sensor_msgs::CameraInfoPtr(new sensor_msgs::CameraInfo(cinfo_manager_.getCameraInfo()));

  imageR->header.frame_id = "Stereo_Cam";
  imageR->header.stamp = image_msg->header.stamp;
  cinfoR->header.frame_id = "Stereo_Cam";
  cinfoR->header.stamp = image_msg->header.stamp;

  cam_left_pub_.publish(imageL, cinfoL);
  cam_right_pub_.publish(imageR, cinfoR);
}

};

int main (int argc, char **argv) {

  ros::init(argc, argv, "stereo_camera");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  stereo_driver::Deinterlacer driver(nh, priv_nh);

  ros::spin();

  return 0;
}
