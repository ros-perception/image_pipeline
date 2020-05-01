// Copyright 2008, 2019 Willow Garage, Inc., Andreas Klintberg, Joshua Whitley
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of {copyright_holder} nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_resource.hpp>
#include <rcutils/cmdline_parser.h>

#include <cstring>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "image_proc/debayer.hpp"
#include "image_proc/rectify.hpp"

int main(int argc, char * argv[])
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;
  const rclcpp::NodeOptions options;

  // Debayer component, image_raw -> image_mono, image_color
  auto debayer_node = std::make_shared<image_proc::DebayerNode>(options);

  // Rectify component, image_mono -> image_rect
  auto rectify_mono_node = std::make_shared<image_proc::RectifyNode>(options);

  // Rectify component, image_color -> image_rect_color
  auto rectify_color_node = std::make_shared<image_proc::RectifyNode>(options);

  exec.add_node(debayer_node);
  exec.add_node(rectify_mono_node);
  exec.add_node(rectify_color_node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
