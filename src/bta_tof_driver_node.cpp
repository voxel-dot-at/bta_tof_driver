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

#include <bta_tof_driver/bta_tof_driver.hpp>
	
/**
 *
 * @brief Main function
 *
 * @param [in] int
 * @param [in] char *
 *
 */
int main(int argc, char *argv[]) {
	ROS_INFO("Starting bta_tof_driver...");
	ros::init (argc, argv, "bta_tof_driver");
	ros::NodeHandle nh, nh_private("~");
		
	bta_tof_driver::BtaRos camera(nh, nh_private,ros::this_node::getName());

	camera.initialize();

	return 0;
}


