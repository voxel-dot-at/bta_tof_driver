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

#include <bta_tof_driver/sensor2D.hpp>
#include <nodelet/nodelet.h>
#include <boost/thread.hpp>

namespace bta_tof_driver {

	class Sensor2DNodelet : public nodelet::Nodelet {

	public:
		 Sensor2DNodelet() : 
		 	nodelet::Nodelet(),
		 	lp_(NULL),
		 	stream_thread_(NULL)
		 {
		 };
		 
		virtual ~Sensor2DNodelet() 
		{
			stream_thread_->join();
		};
	
	private:
		virtual void onInit() 
		{
			NODELET_WARN_STREAM("Initializing nodelet..." << getName());
		
			lp_.reset(new bta_tof_driver::Sensor2D(getNodeHandle(), getPrivateNodeHandle(), getName()));
			stream_thread_.reset(new boost::thread(boost::bind(&Sensor2D::init, lp_.get())));
		};
			boost::scoped_ptr<bta_tof_driver::Sensor2D> lp_;
			boost::scoped_ptr<boost::thread> stream_thread_;
	};
	
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(bta_tof_driver::Sensor2DNodelet, nodelet::Nodelet);


