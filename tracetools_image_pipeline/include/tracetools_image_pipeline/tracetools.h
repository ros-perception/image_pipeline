// Copyright 2021 Víctor Mayoral-Vilches
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


/** \mainpage tracetools_image_pipeline: tracing tools and instrumentation for
 *  for image_pipeline ROS 2 meta-package.
 *
 * `tracetools_image_pipeline` provides utilities to instrument ROS image_pipeline.
 * It provides two main headers:
 *
 * - tracetools/tracetools.h
 *   - instrumentation functions
 * - tracetools/utils.hpp
 *   - utility functions
 */

#ifndef TRACETOOLS_IMAGE_PIPELINE__TRACETOOLS_H_
#define TRACETOOLS_IMAGE_PIPELINE__TRACETOOLS_H_

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "tracetools_image_pipeline/config.h"
#include "tracetools_image_pipeline/visibility_control.hpp"

#ifndef TRACETOOLS_DISABLED
/// Call a tracepoint.
/**
 * This is the preferred method over calling the actual function directly.
 */
#  define TRACEPOINT(event_name, ...) \
  (ros_trace_ ## event_name)(__VA_ARGS__)
#  define DECLARE_TRACEPOINT(event_name, ...) \
  TRACETOOLS_PUBLIC void ros_trace_ ## event_name(__VA_ARGS__);
#else
#  define TRACEPOINT(event_name, ...) ((void) (0))
#  define DECLARE_TRACEPOINT(event_name, ...)
#endif

#ifdef __cplusplus
extern "C"
{
#endif

/// Get tracing compilation status.
/**
 * \return `true` if tracing is enabled, `false` otherwise
 */
TRACETOOLS_PUBLIC bool ros_trace_compile_status();

/// `image_proc_resize_init`
/**
 * Tracepoint while initiating the callback of image_proc::ResizeNode component
 *
 * Notes the `tracetools_image_pipeline` version automatically.
 *
 * \param[in] resize_node rclcpp::node::Node subject to the callback
 * \param[in] resize_image_msg image ROS message stored as sensor_msgs::msg::Image::ConstSharedPtr
 * \param[in] resize_info_msg info ROS message as sensor_msgs::msg::CameraInfo::ConstSharedPtr
 */
DECLARE_TRACEPOINT(
  image_proc_resize_init,
  const void * resize_node,
  const void * resize_image_msg,
  const void * resize_info_msg)

/// `image_proc_resize_fini`
/**
 * Tracepoint while finishing the callback of image_proc::ResizeNode component
 *
 * Notes the `tracetools_image_pipeline` version automatically.
 *
 * \param[in] resize_node rclcpp::node::Node subject to the callback
 * \param[in] resize_image_msg image ROS message stored as sensor_msgs::msg::Image::ConstSharedPtr
 * \param[in] resize_info_msg info ROS message as sensor_msgs::msg::CameraInfo::ConstSharedPtr
 */
DECLARE_TRACEPOINT(
  image_proc_resize_fini,
  const void * resize_node,
  const void * resize_image_msg,
  const void * resize_info_msg)

/// `image_proc_rectify_init`
/**
 * Tracepoint while initiating the callback of image_proc::ResizeNode component
 *
 * Notes the `tracetools_image_pipeline` version automatically.
 *
 * \param[in] rectify_node rclcpp::node::Node subject to the callback
 * \param[in] rectify_image_msg image ROS message stored as sensor_msgs::msg::Image::ConstSharedPtr
 * \param[in] rectify_info_msg info ROS message as sensor_msgs::msg::CameraInfo::ConstSharedPtr
 */
DECLARE_TRACEPOINT(
  image_proc_rectify_init,
  const void * rectify_node,
  const void * rectify_image_msg,
  const void * rectify_info_msg)

/// `image_proc_rectify_fini`
/**
 * Tracepoint while finishing the callback of image_proc::ResizeNode component
 *
 * Notes the `tracetools_image_pipeline` version automatically.
 *
 * \param[in] rectify_node rclcpp::node::Node subject to the callback
 * \param[in] rectify_image_msg image ROS message stored as sensor_msgs::msg::Image::ConstSharedPtr
 * \param[in] rectify_info_msg info ROS message as sensor_msgs::msg::CameraInfo::ConstSharedPtr
 */
DECLARE_TRACEPOINT(
  image_proc_rectify_fini,
  const void * rectify_node,
  const void * rectify_image_msg,
  const void * rectify_info_msg)


#ifdef __cplusplus
}
#endif

#endif  // TRACETOOLS_IMAGE_PIPELINE__TRACETOOLS_H_
