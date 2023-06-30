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

// Copyright 2019, Joshua Whitley
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cstdlib>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "image_view/image_view_node.hpp"
#include "utilities.hpp"

int main(int argc, char ** argv)
{
  using image_view::ImageViewNode;

  std::vector<std::string> args = rclcpp::init_and_remove_ros_arguments(argc, argv);

  // remove program name
  args.erase(args.begin());

  std::string image_transport{"raw"};
  std::string window_name{"window"};
  bool gui{true};
  bool autosize{false};
  int width{-1};
  int height{-1};
  std::string fileformat{"frame%04i.jpg"};
  int colormap{-1};
  int min_image_value{0};
  int max_image_value{0};

  if (image_view::has_option(args, "--image_transport")) {
    image_transport = image_view::get_option(args, "--image_transport");
  }
  if (image_view::has_option(args, "--window_name")) {
    window_name = image_view::get_option(args, "--window_name");
  }
  if (image_view::has_option(args, "--gui")) {
    std::string result = image_view::get_option(args, "--gui");
    if (result.size() == 1) {
      std::istringstream(result) >> gui;
    } else {
      std::istringstream(result) >> std::boolalpha >> gui;
    }
  }
  if (image_view::has_option(args, "--autosize")) {
    std::string result = image_view::get_option(args, "--autosize");
    if (result.size() == 1) {
      std::istringstream(result) >> autosize;
    } else {
      std::istringstream(result) >> std::boolalpha >> autosize;
    }
  }
  if (image_view::has_option(args, "--width")) {
    width = std::atoi(image_view::get_option(args, "--width").c_str());
  }
  if (image_view::has_option(args, "--height")) {
    height = std::atoi(image_view::get_option(args, "--height").c_str());
  }
  if (image_view::has_option(args, "--fileformat")) {
    fileformat = image_view::get_option(args, "--fileformat");
  }
  if (image_view::has_option(args, "--colormap")) {
    colormap = std::atoi(image_view::get_option(args, "--colormap").c_str());
  }
  if (image_view::has_option(args, "--min_image_value")) {
    min_image_value = std::atoi(image_view::get_option(args, "--min_image_value").c_str());
  }
  if (image_view::has_option(args, "--max_image_value")) {
    max_image_value = std::atoi(image_view::get_option(args, "--max_image_value").c_str());
  }

  rclcpp::NodeOptions options;
  // override default parameters with the desired transform
  options.parameter_overrides(
  {
    {"image_transport", image_transport},
    {"window_name", window_name},
    {"gui", gui},
    {"autosize", autosize},
    {"height", height},
    {"width", width},
    {"colormap", colormap},
    {"min_image_value", min_image_value},
    {"max_image_value", max_image_value},
    {"fileformat", fileformat},
  });

  auto iv_node = std::make_shared<ImageViewNode>(options);

  rclcpp::spin(iv_node);

  rclcpp::shutdown();

  return 0;
}
