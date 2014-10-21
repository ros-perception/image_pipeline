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
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_calibration_parsers/parse.h>
#if OPENCV3
#include <opencv2/videoio.hpp>
#endif

cv::VideoWriter outputVideo;

int g_count = 0;
std::string encoding;
std::string codec;
int fps;
std::string filename;

void callback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info)
{
    if (!outputVideo.isOpened() && !info) return;
    else if (!outputVideo.isOpened() && info) {

        cv::Size size(info->width, info->height);

        outputVideo.open(filename, 
#if OPENCV3
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
            ROS_ERROR("Could not create the output video! Check filename and/or support for codec.");
            exit(-1);
        }

        ROS_INFO_STREAM("Starting to record " << codec << " video at " << size << "@" << fps << "fps. Press Ctrl+C to stop recording." );

    }
    else if (outputVideo.isOpened() && info) return;

    cv::Mat image;
    try
    {
        image = cv_bridge::toCvShare(image_msg, encoding)->image;
    } catch(cv_bridge::Exception)
    {
        ROS_ERROR("Unable to convert %s image to %s", image_msg->encoding.c_str(), encoding.c_str());
        return;
    }

    if (!image.empty()) {
        outputVideo << image;
        ROS_INFO_STREAM("Recording frame " << g_count << "\x1b[1F");
        g_count++;
    } else {
        ROS_WARN("Frame skipped, no data!");
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "video_recorder", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    std::string topic = nh.resolveName("image");
    image_transport::CameraSubscriber sub_camera = it.subscribeCamera(topic, 1, &callback);
    image_transport::Subscriber sub_image = it.subscribe(topic, 1,
            boost::bind(callback, _1, sensor_msgs::CameraInfoConstPtr()));

    ros::NodeHandle local_nh("~");
    local_nh.param("filename", filename, std::string("output.avi"));
    local_nh.param("fps", fps, 15);
    local_nh.param("codec", codec, std::string("MJPG"));
    local_nh.param("encoding", encoding, std::string("bgr8"));

    if (codec.size() != 4) {
        ROS_ERROR("The video codec must be a FOURCC identifier (4 chars)");
        exit(-1);
    }

    ROS_INFO_STREAM("Waiting for topic " << topic << "...");
    ros::spin();
    std::cout << "\nVideo saved as " << filename << std::endl;
}
