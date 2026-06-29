// Copyright 2024 Open Source Robotics Foundation, Inc.
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

#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <format>
#include <string>
#include <string_view>

namespace image_view
{
// Thin wrapper around std::vformat. The format string is a runtime ROS
// parameter (the "filename_format" parameter), so std::format -- which
// requires a compile-time-checked format string -- cannot be used directly;
// std::vformat is the runtime-format-string entry point.
//
// NOTE: format strings now use std::format's "{}" replacement-field syntax,
// not printf's "%" conversion specifiers. e.g. "left%04i.%s" -> "left{:04}.{}".
template<typename ... Args>
std::string string_format(std::string_view format, Args && ... args)
{
  return std::vformat(format, std::make_format_args(args ...));
}
}  // namespace image_view

#endif  // UTILS_HPP_
