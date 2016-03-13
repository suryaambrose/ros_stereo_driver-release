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

#pragma once

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_publisher.h>
#include <camera_info_manager/camera_info_manager.h>


namespace stereo_driver {

class Deinterlacer {
public:
  Deinterlacer(ros::NodeHandle nh, ros::NodeHandle priv_nh);
  ~Deinterlacer();

private:
  
  void imageCb(const sensor_msgs::ImageConstPtr& image_msg);
  

  ros::NodeHandle nh_, priv_nh_;

  image_transport::ImageTransport it_;
  image_transport::CameraPublisher cam_left_pub_;
  image_transport::CameraPublisher cam_right_pub_;
  image_transport::Subscriber sub_;

  std::string camera_left_info_url_;
  std::string camera_right_info_url_;

  camera_info_manager::CameraInfoManager cinfo_manager_;
};

};
