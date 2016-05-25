/******************************************************************************
 * Copyright (c) 2014
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
 * ROS wiki: http://wiki.ros.org/bta_ros
 * Github repository: https://github.com/voxel-dot-at/bta_ros
 *
 */
#include <bta_ros/sensor2D.hpp>

namespace bta_ros 
{
	Sensor2D::Sensor2D(ros::NodeHandle nh_camera, 
										ros::NodeHandle nh_private, 
										std::string nodeName) : 
		nh_(nh_camera),
		nh_private_(nh_private),
		it_(nh_camera),
		cim_rgb_(nh_camera),
		nodeName_(nodeName),
		address_("192.168.0.10")
	{
	
		if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, 
									ros::console::levels::Debug) ) {
   			ros::console::notifyLoggerLevelsChanged();
		}
		
	}
	
	Sensor2D::~Sensor2D() 
	{
		stop();
	}
	
	static void	cb_new_pad (GstElement *element,
							GstPad     *pad,
							gpointer    data)
	{
	  GstElement* elem = ( GstElement* ) data;
	  GstPad *sinkpad;
	   GstPadLinkReturn lres;

	  sinkpad = gst_element_get_static_pad ( elem , "sink" );
	  g_assert (sinkpad);
	  lres == gst_pad_link ( pad, sinkpad ); 
	  g_assert (GST_PAD_LINK_SUCCESSFUL(lres));
	  gst_object_unref( sinkpad );
	}

	void Sensor2D::init() {
		GstElement *souphttpsrc, 
			*sdpdemux, *videodepay, 
			*videodecoder, *videoconvert, 
			*filter;
		//GstElement *pipeline_;
	
		gboolean res;
		GstPadLinkReturn lres;
		GstPad *srcpad, *sinkpad;
	
		//GstPadTemplate *teePad;
	
	  if(!nh_private_.getParam(nodeName_+"/2dURL",address_))
			ROS_INFO_STREAM(
				"No ip for download sdp file given. Trying with default: " << address_);
	  	
		// always init first 
		gst_init (NULL,NULL);
	
		// the pipeline_ to hold everything 
		pipeline_ = gst_pipeline_new ("pipeline");
		g_assert (pipeline_);
	
		souphttpsrc = gst_element_factory_make ("souphttpsrc", "sdphttpsrc");
		g_assert (souphttpsrc);
		g_object_set (souphttpsrc, "location", address_.c_str(), NULL);
	
		sdpdemux = gst_element_factory_make ("sdpdemux", NULL);
		g_assert (sdpdemux);
		//g_object_set (sdpdemux, "debug", TRUE, NULL);
		//g_object_set (sdpdemux, "redirect", FALSE, NULL);
		//g_object_set (sdpdemux, "latency", 0, NULL);
	
		// the depayloading 
		videodepay = gst_element_factory_make ("rtph264depay", NULL);
		g_assert (videodepay);
	
		/* the decoding */
		videodecoder = gst_element_factory_make ("avdec_h264", NULL);
		g_assert (videodecoder);
		g_object_set (videodecoder, "skip-frame", 5, NULL);

		/*tee = gst_element_factory_make ("tee", NULL);
		g_assert (tee);*/
	
		// queue for local visualization
		/*queueLocal = gst_element_factory_make ("queue", NULL);
		g_assert (queueLocal);*/
	
		/*queueApp = gst_element_factory_make ("queue", NULL);
		g_assert (queueApp);*/
	
		/*videoflip = gst_element_factory_make ("videoflip", NULL);
		g_assert (videoflip);
		g_object_set (videoflip, "method", 1, NULL);
	
		videocrop = gst_element_factory_make ("videocrop", NULL);
		g_assert (videocrop);
		g_object_set (videocrop, "top", 0, NULL);
		g_object_set (videocrop, "left", 0, NULL);
		g_object_set (videocrop, "right", 0, NULL);
		g_object_set (videocrop, "bottom", 0, NULL);*/
	
		// video playback
		/*videosink = gst_element_factory_make ("autovideosink", NULL);
		g_assert (videosink);
		g_object_set (videosink, "sync", FALSE, NULL);*/
	
		videoconvert =gst_element_factory_make("videoconvert", NULL);
		g_assert (videoconvert);
	
		filter = gst_element_factory_make ("capsfilter", "filter");
			g_assert (filter);
		GstCaps *filtercaps;
		filtercaps = gst_caps_new_simple ("video/x-raw",
					 "format", G_TYPE_STRING, "BGR",
					 NULL);
			g_object_set (G_OBJECT (filter), "caps", filtercaps, NULL);
			gst_caps_unref (filtercaps);
	
		appsink = gst_element_factory_make("appsink", NULL);
		g_assert (appsink);
		//g_object_set (appsink, "emit-signals", TRUE, NULL);
		g_object_set (appsink, "drop", TRUE, NULL);
		g_object_set (appsink, "max-buffers", 1, NULL);
	
		// add depayloading and playback to the pipeline_ and link 
		gst_bin_add_many (GST_BIN (pipeline_),
					souphttpsrc,
					sdpdemux,
					videodepay, 
					videodecoder,
					videoconvert,
					filter, 
					appsink,
					NULL);
	
		res = gst_element_link (souphttpsrc, sdpdemux);
		g_assert (res == TRUE);
		//g_print ("link %d result \n", res);
	
		g_signal_connect (sdpdemux, "pad-added", G_CALLBACK (cb_new_pad), videodepay);
	
		res = gst_element_link_many (videodepay, videodecoder, videoconvert, filter, appsink, NULL);
		g_assert (res == TRUE);
	
	
		//g_signal_connect(appsink, "new-sample", G_CALLBACK(newFrame), NULL);
	
		//GstElement *session = sdpdemux.session;
		// the RTP pad that we have to connect to the depayloader will be created
		// dynamically so we connect to the pad-added signal, pass the depayloader as
		// user_data so that we can link to it.
		//printf("rtpbin: %p\n",rtpbin);
		//g_signal_connect (rtpbin, "pad-added", G_CALLBACK (pad_added_cb), videodepay);

		// give some stats when we receive RTCP 
		//g_signal_connect (sdpdemux, "on-ssrc-active", G_CALLBACK (on_ssrc_active_cb), videodepay);
	
		// set the pipeline_ to playing 
		ROS_INFO ("starting receiver pipeline");
		
		gst_element_set_state(pipeline_, GST_STATE_PAUSED);
		if (gst_element_get_state(pipeline_, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
			ROS_FATAL("Failed to PAUSE stream, check your configuration.");
			ros::shutdown();
			return;
		} else {
			ROS_DEBUG("Stream is PAUSED.");
		}	
		gst_element_set_state (pipeline_, GST_STATE_PLAYING);
		if (gst_element_get_state(pipeline_, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
			ROS_WARN("Failed to PLAY stream.");
		}
		//loop = g_main_loop_new (NULL, FALSE);
		//g_main_loop_run (loop);
	
	 {
		// Advertise all published topics
			cim_rgb_.setCameraName(nodeName_);
			if(cim_rgb_.validateURL(
					/*camera_info_url_*/"package://bta_ros/calib.yml")) {
				cim_rgb_.loadCameraInfo("package://bta_ros/calib.yml");
				ROS_INFO_STREAM("Loaded camera calibration from " << 
					"package://bta_ros/calib.yml"	);
			} else {
				ROS_WARN_STREAM("Camera info at: " <<
					"package://bta_ros/calib.yml" <<
					" not found. Using an uncalibrated config.");
			} 	
	
			pub_rgb_ = it_.advertiseCamera(nodeName_ + "/sensor2d/image_raw", 1);

			//sub_amp_ = nh_private_.subscribe("bta_node_amp", 1, &BtaRos::ampCb, this);
			//sub_dis_ = nh_private_.subscribe("bta_node_dis", 1, &BtaRos::disCb, this);
		}
		GstState state;
		while (nh_.ok() && !ros::isShuttingDown()) {
			gst_element_get_state(pipeline_, &state, NULL, -1);
			if (state == GST_STATE_PLAYING)
				getFrame();
			else {
				ROS_INFO("Stream STOPPED. Reconnecting");
				gst_element_set_state (pipeline_, GST_STATE_NULL);
				if (gst_element_set_state (pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
					ROS_ERROR("Failed to PLAY stream.");
				}
			}
			ros::spinOnce ();	
		}
		
		//stop();
	
		return;
	}

	void Sensor2D::stop() {
		g_print ("Returned, stopping playback\n");
	 	gst_element_set_state (pipeline_, GST_STATE_NULL);
	
		g_print ("Deleting pipeline_\n");
		gst_object_unref (GST_OBJECT (pipeline_));

		return;
	}

	void Sensor2D::getFrame() {
		GstSample *sample;
	
		sample = gst_app_sink_pull_sample((GstAppSink *)appsink);
	
		if(sample != NULL) {
			ROS_DEBUG("		frame 2D Arrived");
			sensor_msgs::ImagePtr rgb (new sensor_msgs::Image);
		  GstBuffer* sampleBuffer = NULL;
			GstStructure *s;
			gint width, height;
			GstMapInfo bufferInfo;
			gboolean res;
			GstCaps *caps;
		  
	  	caps = gst_sample_get_caps (sample);
	  	s = gst_caps_get_structure (caps, 0);
	  	res = gst_structure_get_int (s, "width", &width);
			res |= gst_structure_get_int (s, "height", &height);
			if (!res) {
				ROS_DEBUG ("could not get snapshot dimension\n");
				return;
			}
			
      sampleBuffer = gst_sample_get_buffer(sample);
      if(sampleBuffer != NULL)
      {
          gst_buffer_map(sampleBuffer, &bufferInfo, GST_MAP_READ);
					//dis->header.seq = frame->frameCounter;
        	//dis->header.stamp.sec = frame->timeStamp;
        	rgb->height = height;
        	rgb->width = width;
        	rgb->encoding = sensor_msgs::image_encodings::BGR8;
        	rgb->step = width*(sizeof(unsigned char)*3);
        	rgb->data.resize(bufferInfo.size);
        	memcpy ( &rgb->data[0], bufferInfo.data, bufferInfo.size );

         sensor_msgs::CameraInfoPtr ci_rgb(
         		new sensor_msgs::CameraInfo(cim_rgb_.getCameraInfo()));
					ci_rgb->header.frame_id = nodeName_+"/sensor2d";
					rgb->header.frame_id = nodeName_+"/sensor2d";

					pub_rgb_.publish(rgb,ci_rgb);
      }
      gst_buffer_unmap (sampleBuffer, &bufferInfo);
		  gst_sample_unref(sample);
		 }
		  return;
	}
	
}

