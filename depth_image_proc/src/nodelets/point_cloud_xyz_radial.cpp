/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 * 
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 * 
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <boost/thread.hpp>
#include <depth_image_proc/depth_traits.h>

#include <sensor_msgs/point_cloud2_iterator.h>

namespace depth_image_proc {

    namespace enc = sensor_msgs::image_encodings;

    class PointCloudXyzRadialNodelet : public nodelet::Nodelet
    {
	// Subscriptions
	boost::shared_ptr<image_transport::ImageTransport> it_;
	image_transport::CameraSubscriber sub_depth_;
	int queue_size_;

	// Publications
	boost::mutex connect_mutex_;
	typedef sensor_msgs::PointCloud2 PointCloud;
	ros::Publisher pub_point_cloud_;

	
	std::vector<double> D_;
	boost::array<double, 9> K_;
  
	int width_;
	int height_;

	cv::Mat binned;
  
	virtual void onInit();

	void connectCb();

	void depthCb(const sensor_msgs::ImageConstPtr& depth_msg,
		     const sensor_msgs::CameraInfoConstPtr& info_msg);

	// Handles float or uint16 depths
	template<typename T>
	void convert(const sensor_msgs::ImageConstPtr& depth_msg, PointCloud::Ptr& cloud_msg);
    };

    cv::Mat initMatrix(cv::Mat cameraMatrix, cv::Mat distCoeffs, int width, int height, bool radial)
    {
	int i,j;
	int totalsize = width*height;
	cv::Mat pixelVectors(1,totalsize,CV_32FC3);
	cv::Mat dst(1,totalsize,CV_32FC3);

	cv::Mat sensorPoints(cv::Size(height,width), CV_32FC2);
	cv::Mat undistortedSensorPoints(1,totalsize, CV_32FC2);

	std::vector<cv::Mat> ch;
	for(j = 0; j < height; j++)
	{
	    for(i = 0; i < width; i++)
	    {
		cv::Vec2f &p = sensorPoints.at<cv::Vec2f>(i,j);
		p[0] = i;
		p[1] = j;
	    }
	}

	sensorPoints = sensorPoints.reshape(2,1);

	cv::undistortPoints(sensorPoints, undistortedSensorPoints, cameraMatrix, distCoeffs);

	ch.push_back(undistortedSensorPoints);
	ch.push_back(cv::Mat::ones(1,totalsize,CV_32FC1));
	cv::merge(ch,pixelVectors);

	if(radial)
	{
	    for(i = 0; i < totalsize; i++)
	    {
		normalize(pixelVectors.at<cv::Vec3f>(i),
			  dst.at<cv::Vec3f>(i));
	    }
	    pixelVectors = dst;
	}
	return pixelVectors.reshape(3,width);
    }
  

    void PointCloudXyzRadialNodelet::onInit()
    {
	ros::NodeHandle& nh         = getNodeHandle();
	ros::NodeHandle& private_nh = getPrivateNodeHandle();
	it_.reset(new image_transport::ImageTransport(nh));

	// Read parameters
	private_nh.param("queue_size", queue_size_, 5);

	// Monitor whether anyone is subscribed to the output
	ros::SubscriberStatusCallback connect_cb = 
	    boost::bind(&PointCloudXyzRadialNodelet::connectCb, this);
	// Make sure we don't enter connectCb() between advertising and assigning to pub_point_cloud_
	boost::lock_guard<boost::mutex> lock(connect_mutex_);
	pub_point_cloud_ = nh.advertise<PointCloud>("points", 1, connect_cb, connect_cb);
    }

    // Handles (un)subscribing when clients (un)subscribe
    void PointCloudXyzRadialNodelet::connectCb()
    {
	boost::lock_guard<boost::mutex> lock(connect_mutex_);
	if (pub_point_cloud_.getNumSubscribers() == 0)
	{
	    sub_depth_.shutdown();
	}
	else if (!sub_depth_)
	{
	    image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
	    sub_depth_ = it_->subscribeCamera("image_raw",
					      queue_size_,
					      &PointCloudXyzRadialNodelet::depthCb,
					      this, hints);
	}
    }

    void PointCloudXyzRadialNodelet::depthCb(const sensor_msgs::ImageConstPtr& depth_msg,
					     const sensor_msgs::CameraInfoConstPtr& info_msg)
    {
	PointCloud::Ptr cloud_msg(new PointCloud);
	cloud_msg->header = depth_msg->header;
	cloud_msg->height = depth_msg->height;
	cloud_msg->width  = depth_msg->width;
	cloud_msg->is_dense = false;
	cloud_msg->is_bigendian = false;

	sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
	pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

	if(info_msg->D != D_ || info_msg->K != K_ || width_ != info_msg->width ||
	   height_ != info_msg->height)
	{
	    D_ = info_msg->D;
	    K_ = info_msg->K;
	    width_ = info_msg->width;
	    height_ = info_msg->height;
	    binned = initMatrix(cv::Mat_<double>(3, 3, &K_[0]),cv::Mat(D_),width_,height_,true);
	}

	if (depth_msg->encoding == enc::TYPE_16UC1)
	{
	    convert<uint16_t>(depth_msg, cloud_msg);
	}
	else if (depth_msg->encoding == enc::TYPE_32FC1)
	{
	    convert<float>(depth_msg, cloud_msg);
	}
	else
	{
	    NODELET_ERROR_THROTTLE(5, "Depth image has unsupported encoding [%s]", depth_msg->encoding.c_str());
	    return;
	}

	pub_point_cloud_.publish (cloud_msg);
    }

    template<typename T>
    void PointCloudXyzRadialNodelet::convert(const sensor_msgs::ImageConstPtr& depth_msg, PointCloud::Ptr& cloud_msg)
    {
	// Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
	double unit_scaling = DepthTraits<T>::toMeters( T(1) );
	float bad_point = std::numeric_limits<float>::quiet_NaN();

	sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
	sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
	sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
	const T* depth_row = reinterpret_cast<const T*>(&depth_msg->data[0]);
	int row_step = depth_msg->step / sizeof(T);
	for (int v = 0; v < (int)cloud_msg->height; ++v, depth_row += row_step)
	{
	    for (int u = 0; u < (int)cloud_msg->width; ++u, ++iter_x, ++iter_y, ++iter_z)
	    {
		T depth = depth_row[u];

		// Missing points denoted by NaNs
		if (!DepthTraits<T>::valid(depth))
		{
		    *iter_x = *iter_y = *iter_z = bad_point;
		    continue;
		}
		const cv::Vec3f &cvPoint = binned.at<cv::Vec3f>(u,v) * DepthTraits<T>::toMeters(depth);
		// Fill in XYZ
		*iter_x = cvPoint(0);
		*iter_y = cvPoint(1);
		*iter_z = cvPoint(2);
	    }
	}
    }

} // namespace depth_image_proc

// Register as nodelet
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(depth_image_proc::PointCloudXyzRadialNodelet,nodelet::Nodelet);
