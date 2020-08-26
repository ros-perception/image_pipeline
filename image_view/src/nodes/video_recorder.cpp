/****************************************************************************
* Software License Agreement (Apache License)
*
*     Copyright (C) 2012-2013 Open Source Robotics Foundation
*
*     Licensed under the Apache License, Version 2.0 (the "License");
*     you may not use this file except in compliance with the License.
*     You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
*     Unless required by applicable law or agreed to in writing, software
*     distributed under the License is distributed on an "AS IS" BASIS,
*     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*     See the License for the specific language governing permissions and
*     limitations under the License.
*
*****************************************************************************/

#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_calibration_parsers/parse.h>
#if CV_MAJOR_VERSION == 3
#include <opencv2/videoio.hpp>
#endif
#include <stdio.h>
#include "boost/date_time/posix_time/posix_time.hpp"
#include <queue>

cv::VideoWriter outputVideo;

int g_count = 0;
ros::Time g_last_wrote_time = ros::Time(0);
std::string encoding;
std::string codec;
int fps;
std::string base_filename, filename;
double min_depth_range;
double max_depth_range;
bool use_dynamic_range;
int colormap;
bool stamped_filename;
bool rolling_buffer;
ros::Duration rb_video_length;
int rb_n_videos;
ros::Time video_creation_stamp;
std::queue<std::string> filenames;
std::string video_topic;
bool use_posix;

void generateStampedFilename(std::string &filename) {
  std::size_t found = base_filename.find_last_of("/\\");
  std::string path = base_filename.substr(0, found + 1);
  std::string basename = base_filename.substr(found + 1);
  std::stringstream ss;
  if (use_posix) {
    boost::posix_time::ptime my_posix_time = ros::Time::now().toBoost();
    std::string iso_time_str = boost::posix_time::to_iso_extended_string(my_posix_time);
    ss << iso_time_str << "_" << basename;
  }
  else {
    ss << ros::Time::now().toNSec() << "_" << basename;
  }
  filename = path + ss.str();
  ROS_INFO("Video recording to %s", filename.c_str());
  // If rolling_buffer mode, store filenames in a vector
  if (rolling_buffer) {
    filenames.push(filename);
    // Delete oldest video
    if (filenames.size() > rb_n_videos) {
      remove(filenames.front().c_str());
      filenames.pop();
    }
  }
}

void callback(const sensor_msgs::ImageConstPtr& image_msg)
{
    if (!outputVideo.isOpened()) {

      // Generate stamped file name if required or in rolling_buffer mode
      if (stamped_filename or rolling_buffer) {
        generateStampedFilename(filename);
      }
      else {
        filename = base_filename;
      }
        cv::Size size(image_msg->width, image_msg->height);

        outputVideo.open(filename, 
#if CV_MAJOR_VERSION >= 3

                cv::VideoWriter::fourcc(codec.c_str()[0],
#else
                CV_FOURCC(codec.c_str()[0],
#endif
                          codec.c_str()[1],
                          codec.c_str()[2],
                          codec.c_str()[3]),
                fps,
                size,
                true);
        video_creation_stamp = ros::Time::now();

        if (!outputVideo.isOpened())
        {
            ROS_ERROR("Could not create the output video! Check filename and/or support for codec.");
            exit(-1);
        }

        ROS_INFO_STREAM("Starting to record " << codec << " video at " << size << "@" << fps << "fps. Press Ctrl+C to stop recording." );

    }

    if ((image_msg->header.stamp - g_last_wrote_time) < ros::Duration(1.0 / fps))
    {
      // Skip to get video with correct fps
      return;
    }

    // Check if video has reached maximum length (only if rolling_buffer mode enabled and video is opened)
    if (rolling_buffer && ros::Time::now() - video_creation_stamp >= rb_video_length) {
      outputVideo.release(); // Release video, so in next frame a new one will be created
      return;
    }

    try
    {
      cv_bridge::CvtColorForDisplayOptions options;
      options.do_dynamic_scaling = use_dynamic_range;
      options.min_image_value = min_depth_range;
      options.max_image_value = max_depth_range;
      options.colormap = colormap;
      const cv::Mat image = cv_bridge::cvtColorForDisplay(cv_bridge::toCvShare(image_msg), encoding, options)->image;
      if (!image.empty()) {
        outputVideo << image;
        ROS_INFO_STREAM("Recording frame " << g_count << "\x1b[1F");
        g_count++;
        g_last_wrote_time = image_msg->header.stamp;
      } else {
          ROS_WARN("Frame skipped, no data!");
      }
    } catch(cv_bridge::Exception)
    {
        ROS_ERROR("Unable to convert %s image to %s", image_msg->encoding.c_str(), encoding.c_str());
        return;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "video_recorder", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    ros::NodeHandle local_nh("~");
    local_nh.param("filename", base_filename, std::string("output.avi"));
    local_nh.param("stamped_filename", stamped_filename, false);
    local_nh.param("fps", fps, 15);
    local_nh.param("codec", codec, std::string("MJPG"));
    local_nh.param("encoding", encoding, std::string("bgr8"));
    // cv_bridge::CvtColorForDisplayOptions
    local_nh.param("min_depth_range", min_depth_range, 0.0);
    local_nh.param("max_depth_range", max_depth_range, 0.0);
    local_nh.param("use_dynamic_depth_range", use_dynamic_range, false);
    local_nh.param("colormap", colormap, -1);
    local_nh.param("rolling_buffer_mode", rolling_buffer, false); // rolling_buffer mode enabled/disabled
    double rb_v_duration;
    local_nh.param("rb_video_length", rb_v_duration, 5.0);
    rb_video_length = ros::Duration(rb_v_duration*60); // Video length in ros::Duration (seconds)
    local_nh.param("rb_n_videos", rb_n_videos, 10); // Number of videos to store in rolling_buffer mode
    local_nh.param("video_topic", video_topic, std::string("image")); // Input video topic
    local_nh.param("use_posix_timestamp", use_posix, true); // Use human readable posix format in stamped filenames

    // Generate stamped file name if required or in rolling_buffer mode
    if (stamped_filename or rolling_buffer) {
      generateStampedFilename(filename);
    }
    else {
      filename = base_filename;
    }

    if (codec.size() != 4) {
        ROS_ERROR("The video codec must be a FOURCC identifier (4 chars)");
        exit(-1);
    }

    image_transport::ImageTransport it(nh);
    std::string topic = nh.resolveName(video_topic);
    image_transport::Subscriber sub_image = it.subscribe(topic, 1, callback);

    ROS_INFO_STREAM("Waiting for topic " << topic << "...");
    ros::spin();
    std::cout << "\nVideo saved as " << filename << std::endl;
}
