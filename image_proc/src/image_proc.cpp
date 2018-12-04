// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#ifdef __clang__
// TODO(dirk-thomas) custom implementation until we can use libc++ 3.9
#include <string>
namespace fs
{
class path
{
public:
  explicit path(const std::string & p)
  : path_(p)
  {}
  bool is_absolute()
  {
    return path_[0] == '/';
  }

private:
  std::string path_;
};
}  // namespace fs
#else
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#endif

#include <cstring>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <ament_index_cpp/get_resource.hpp>
#include "class_loader/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include <rcutils/cmdline_parser.h>
#include "../include/debayer.hpp"
#include "../include/rectify.hpp"
std::vector<std::string> split(
  const std::string & s, char delim, bool skip_empty = false)
{
  std::vector<std::string> result;
  std::stringstream ss;
  ss.str(s);
  std::string item;
  while (std::getline(ss, item, delim)) {
    if (skip_empty && item == "") {
      continue;
    }
    result.push_back(item);
  }
  return result;
}
void print_usage()
{
  printf("Usage for image_proc:\n");
  printf("options:\n");
  printf("--camera_namespace : Specifies the camera.\n");
}

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  // Force flush of the stdout buffer.
  rclcpp::init(argc, argv);
  std::string camera_namespace;
  if (rcutils_cli_option_exist(argv, argv + argc, "--camera_namespace")) {
    camera_namespace = std::string(rcutils_cli_get_option(argv, argv + argc, "--camera_namespace"));
  }
  else {
    print_usage();
    return 0;
  }

  rclcpp::executors::SingleThreadedExecutor exec;
  auto debayer_node = std::make_shared<image_proc::DebayerNode>();
  debayer_node->set_parameter_if_not_set("camera_namespace", camera_namespace);
  auto rectify_mono_node = std::make_shared<image_proc::RectifyNode>();
  rectify_mono_node->set_parameter_if_not_set("camera_namespace", camera_namespace);
  rectify_mono_node->set_parameter_if_not_set("image_mono", "/image_mono");
  auto rectify_color_node = std::make_shared<image_proc::RectifyNode>();
  rectify_color_node->set_parameter_if_not_set("camera_namespace", camera_namespace);
  rectify_color_node->set_parameter_if_not_set("image_color", "/image_color");
  exec.add_node(debayer_node);
  exec.add_node(rectify_mono_node);
  exec.add_node(rectify_color_node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
