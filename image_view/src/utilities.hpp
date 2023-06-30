// Copyright 2023 Open Source Robotics Foundation, Inc.
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

#ifndef UTILITIES_HPP_
#define UTILITIES_HPP_

#include <string>
#include <vector>

namespace image_view
{
/// Get the option from a list of arguments
/// param[in] args List of arguments
/// param[in] option name to extract
/// return option value
std::string get_option(const std::vector<std::string> & args, const std::string & option_name);

/// Is the option available in the list of arguments
/// param[in] args List of arguments
/// param[in] option name to extract
/// return true if the option exists or false otherwise
bool has_option(const std::vector<std::string> & args, const std::string & option_name);
}  // namespace image_view
#endif  // UTILITIES_HPP_
