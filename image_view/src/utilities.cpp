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

#include "utilities.hpp"

#include <string>
#include <vector>

namespace image_view
{
std::string get_option(const std::vector<std::string> & args, const std::string & option_name)
{
  for (auto it = args.begin(), end = args.end(); it != end; ++it) {
    if (*it == option_name) {
      if (it + 1 != end) {
        return *(it + 1);
      }
    }
  }

  return "";
}

bool has_option(const std::vector<std::string> & args, const std::string & option_name)
{
  for (auto it = args.begin(), end = args.end(); it != end; ++it) {
    if (*it == option_name) {
      return true;
    }
  }

  return false;
}
}  // namespace image_view
