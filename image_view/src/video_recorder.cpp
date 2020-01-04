// Copyright 2012, 2013, 2019 Open Source Robotics Foundation, Joshua Whitley
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

#include <rclcpp/rclcpp.hpp>

#include <memory>

#include "image_view/video_recorder_node.hpp"

int main(int argc, char ** argv)
{
  using image_view::VideoRecorderNode;

  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto vr_node = std::make_shared<VideoRecorderNode>(options);

  rclcpp::spin(vr_node);

  return 0;
}
