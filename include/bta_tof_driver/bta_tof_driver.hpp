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

#ifndef _BTA_TOF_DRIVER_HPP_
#define _BTA_TOF_DRIVER_HPP_

#include <bta.h>

// ROS communication
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <ros/console.h>
#include <tf/transform_listener.h>

// PCL 
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>

// Standard libs
#include <stdio.h>
#include <time.h>
#include <sstream>
#include <string>

// Dynamic reconfigure
#include <bta_tof_driver/bta_tof_driverConfig.h>
#include <dynamic_reconfigure/server.h>

//static ros::Publisher int_amp,int_dis,int_rgb;

namespace bta_tof_driver {

class BtaRos
{

    typedef bta_tof_driver::bta_tof_driverConfig Config;
    typedef dynamic_reconfigure::Server<Config> ReconfigureServer;

public:

    /**
     *
     * @brief Class constructor.
     *
     * param [in] ros::NodeHandle
     * param [in] ros::NodeHandle
     * param [in] std::string
     *
     */
    BtaRos(ros::NodeHandle nh_camera, ros::NodeHandle nh_private, std::string nodeName);

    /**
     *
     * @brief Class destructor.
     *
     */
    virtual ~BtaRos();

    /**
     *
     * @brief Initializes the device and parameters.
     *
     */
    int initialize();

    /**
     *
     * @brief Helper for connect to the device.
     *
     */
    int connectCamera();

    /**
     *
     * @brief Closes the connection to the device.
     *
     */
    void close();


    /**
     *
     * @brief Publish the data based on set up parameters.
     *
     */
    void publishData();

    //void ampCb(const sensor_msgs::ImagePtr& amp);

    //void disCb(const sensor_msgs::ImagePtr& dis);

    static void BTA_CALLCONV infoEventCb(BTA_EventId eventId, int8_t *msg) {
	ROS_DEBUG("   Callback: infoEvent (%d) %s\n", eventId, msg);
    }

private:
    ros::NodeHandle nh_, nh_private_;
    std::string nodeName_;
    camera_info_manager::CameraInfoManager cim_tof_/*, *cim_rgb*/;
    image_transport::ImageTransport it_;
    image_transport::CameraPublisher pub_amp_, pub_dis_/*, pub_rgb*/;
    tf2_ros::StaticTransformBroadcaster pub_tf;
    geometry_msgs::TransformStamped transformStamped;
    ros::Publisher pub_xyz_;
    //ros::Subscriber sub_amp_, sub_dis_;
    boost::shared_ptr<ReconfigureServer> reconfigure_server_;
    bool config_init_;

    boost::mutex connect_mutex_;

    // Variables needed for config
    uint8_t udpDataIpAddr_[6], udpControlOutIpAddr_[6],
    udpControlInIpAddr_[6], tcpDeviceIpAddr_[6];
    std::string uartPortName_, calibFileName_;

    sensor_msgs::PointCloud2Ptr _xyz;

    BTA_Handle handle_;
    BTA_Config config_;

    /**
     *
     * @brief Callback for rqt_reconfigure. It is called any time we change a
     * parameter in the visual interface
     *
     * @param [in] bta_tof_driver::bta_tof_driverConfig
     * @param [in] uint32_t
     *
     */
    void callback(bta_tof_driver::bta_tof_driverConfig &config, uint32_t level);

    /**
     *
     * @brief Reads configuration from the server parameters
     *
     */
    void parseConfig();

    /**
     *
     * @brief Returns the size of the data based in BTA_DataFormat
     *
     */
    size_t getDataSize(BTA_DataFormat dataFormat);

    /**
     *
     * @brief Returns the data encoding flat based in BTA_DataFormat
     *
     */
    std::string getDataType(BTA_DataFormat dataFormat);

    /**
     * @brief Gives the conversion value to meters from the BTA_Unit
     * @param unit
     * @return the value to multiply to the data.
     */
    float getUnit2Meters(BTA_Unit unit);

    //Test of using camera callback
    /*static void BTA_CALLCONV frameArrived(BTA_Frame *frame) {

	ROS_DEBUG("   Callback: frameArrived FrameCounter %d\n", frame->frameCounter);

	BTA_Status status;
	uint16_t *distances;
	BTA_DataFormat dataFormat;
	BTA_Unit unit;
	uint16_t xRes, yRes;

	sensor_msgs::ImagePtr amp (new sensor_msgs::Image);
	sensor_msgs::ImagePtr dis (new sensor_msgs::Image);

	status = BTAgetDistances(frame, (void **)&distances, &dataFormat, &unit, &xRes, &yRes);
	if (status == BTA_StatusOk) {
		if (dataFormat == BTA_DataFormatUInt16) {
		    if (unit == BTA_UnitMillimeter) {
				dis->header.seq = frame->frameCounter;
				dis->header.stamp.sec = frame->timeStamp;
				dis->height = yRes;
				dis->width = xRes;
				dis->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
				dis->step = yRes*sizeof(uint16_t);
				dis->data.resize(xRes*yRes*sizeof(uint16_t));
				memcpy ( &dis->data[0], distances, xRes*yRes*sizeof(uint16_t) );
		    }
		}
	}

	uint16_t *amplitudes;
	status = BTAgetAmplitudes(frame, (void **)&amplitudes, &dataFormat, &unit, &xRes, &yRes);
	if (status == BTA_StatusOk) {
		if (dataFormat == BTA_DataFormatUInt16) {
		    if (unit == BTA_UnitUnitLess) {
			dis->header.seq = frame->frameCounter;
				amp->header.stamp.sec = frame->timeStamp;
				amp->height = yRes;
				amp->width = xRes;
				amp->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
				amp->step = yRes*sizeof(uint16_t);
				amp->data.resize(xRes*yRes*sizeof(uint16_t));
				memcpy ( &amp->data[0], amplitudes, xRes*yRes*sizeof(uint16_t) );
		    }
		}
	}

	/ *
	 * Publishing the messages
	 * /
	//int_amp.publish(amp);
	//int_dis.publish(dis);

    };*/

};
}

#endif //_BTA_TOF_DRIVER_HPP_

