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
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <rcutils/cmdline_parser.h>
// #include <camera_calibration_parsers/parse.h>
#if CV_MAJOR_VERSION == 3
#include <opencv2/videoio.hpp>
#endif

cv::VideoWriter outputVideo;

int g_count = 0;
rclcpp::Time g_last_wrote_time = rclcpp::Time(0);
std::string encoding;
std::string codec;
int fps;
std::string topic;
std::string filename;
double min_depth_range;
double max_depth_range;
bool use_dynamic_range;
int colormap;
rclcpp::Node::SharedPtr node;

void callback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg)
{
    if (!outputVideo.isOpened()) {

        cv::Size size(image_msg->width, image_msg->height);

        outputVideo.open(filename, 
#if CV_MAJOR_VERSION == 3
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

        if (!outputVideo.isOpened())
        {
            RCLCPP_ERROR(node->get_logger(), "Could not create the output video! Check filename and/or support for codec.");
            exit(-1);
        }
        std::stringstream ss;
        ss << "Starting to record " << codec << " video at " << size << "@" << fps << "fps. Press Ctrl+C to stop recording.";
        RCLCPP_INFO(node->get_logger(), ss.str().c_str());

    }

    if ((image_msg->header.stamp.sec - g_last_wrote_time.seconds()) < rclcpp::Duration(1 / fps).seconds())
    {
      // Skip to get video with correct fps
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
        RCLCPP_INFO(node->get_logger(), "Recording frame %d \x1b[1F", g_count);
        g_count++;
        g_last_wrote_time = image_msg->header.stamp;
      } else {
          RCLCPP_WARN(node->get_logger(), "Frame skipped, no data!");
      }
    } catch(cv_bridge::Exception)
    {
        RCLCPP_ERROR(node->get_logger(), "Unable to convert %s image to %s", image_msg->encoding.c_str(), encoding.c_str());
        return;
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    if (rcutils_cli_option_exist(argv, argv + argc, "--topic")){
      topic = std::string(rcutils_cli_get_option(argv, argv + argc, "--topic"));
    }
    node = rclcpp::Node::make_shared("video_recoder");
    node->get_parameter_or("filename", filename, std::string("output.avi"));
    bool stamped_filename;
    node->get_parameter_or("stamped_filename", stamped_filename, false);
    node->get_parameter_or("fps", fps, 15);
    node->get_parameter_or("codec", codec, std::string("MJPG"));
    node->get_parameter_or("encoding", encoding, std::string("bgr8"));
    // cv_bridge::CvtColorForDisplayOptions
    node->get_parameter_or("min_depth_range", min_depth_range, 0.0);
    node->get_parameter_or("max_depth_range", max_depth_range, 0.0);
    node->get_parameter_or("use_dynamic_depth_range", use_dynamic_range, false);
    node->get_parameter_or("colormap", colormap, -1);

    if (stamped_filename) {
      std::size_t found = filename.find_last_of("/\\");
      std::string path = filename.substr(0, found + 1);
      std::string basename = filename.substr(found + 1);
      std::stringstream ss;
      ss << rclcpp::Clock().now().seconds() << basename;
      filename = path + ss.str();
      RCLCPP_INFO(node->get_logger(), "Video recording to %s", filename.c_str());
    }

    if (codec.size() != 4) {
        RCLCPP_ERROR(node->get_logger(), "The video codec must be a FOURCC identifier (4 chars)");
        exit(-1);
    }

    std::shared_ptr<image_transport::ImageTransport> it;
    it.reset(new image_transport::ImageTransport(node));
    image_transport::Subscriber sub_image = it->subscribe(topic, 1, callback);

    RCLCPP_INFO(node->get_logger(), "Waiting for topic %s ...", topic.c_str());
    rclcpp::spin(node);
    std::cout << "\nVideo saved as " << filename << std::endl;
    rclcpp::shutdown();
}
