/******************************************************************************
 * Copyright (c) 2016
 * VoXel Interaction Design GmbH
 *
 * @author Angel Merino Sastre
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 ******************************************************************************/

/** @mainpage Bta ROS driver
 *
 * @section intro_sec Introduction
 *
 * This software defines a interface for working with all ToF cameras from 
 * Bluetechnix GmbH supported by their API.
 *
 * @section install_sec Installation
 *
 * We encorage you to follow the instruction we prepared in:
 *
 * ROS wiki: http://wiki.ros.org/bta_tof_driver
 * Github repository: https://github.com/voxel-dot-at/bta_tof_driver
 *
 */
#ifndef __SENSOR2D_HPP__
#define __SENSOR2D_HPP__


#include <gst/gst.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <glib-object.h>
#include <stdlib.h>
#include <gst/app/gstappsink.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/thread/locks.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <cv_bridge/cv_bridge.h>


#include <ros/console.h>

namespace bta_tof_driver {

	class Sensor2D
	{
		ros::NodeHandle nh_, nh_private_;
		std::string nodeName_;
		camera_info_manager::CameraInfoManager cim_rgb_;
		image_transport::ImageTransport it_;
		image_transport::CameraPublisher pub_rgb_;	
	
		std::string address_;
	
		GstElement *pipeline_;
		GstElement *appsink;
		GMainLoop *loop;
	
		//boost::thread* streaming;

 public:
 
 	Sensor2D(ros::NodeHandle nh_camera, 
 					ros::NodeHandle nh_private, 
 					std::string nodeName);
 	virtual ~Sensor2D();
 	
	void init();
	void stop();
	void getFrame();

	};
}

#endif
