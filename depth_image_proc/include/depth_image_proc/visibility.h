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

#ifndef DEPTH_IMAGE_PROC__VISIBILITY_H_
#define DEPTH_IMAGE_PROC__VISIBILITY_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__

  #ifdef __GNUC__
    #define DEPTH_IMAGE_PROC_EXPORT __attribute__ ((dllexport))
    #define DEPTH_IMAGE_PROC_IMPORT __attribute__ ((dllimport))
  #else
    #define DEPTH_IMAGE_PROC_EXPORT __declspec(dllexport)
    #define DEPTH_IMAGE_PROC_IMPORT __declspec(dllimport)
  #endif

  #ifdef DEPTH_IMAGE_PROC_DLL
    #define DEPTH_IMAGE_PROC_PUBLIC DEPTH_IMAGE_PROC_EXPORT
  #else
    #define DEPTH_IMAGE_PROC_PUBLIC DEPTH_IMAGE_PROC_IMPORT
  #endif

  #define DEPTH_IMAGE_PROC_PUBLIC_TYPE DEPTH_IMAGE_PROC_PUBLIC

  #define DEPTH_IMAGE_PROC_LOCAL

#else

  #define DEPTH_IMAGE_PROC_EXPORT __attribute__ ((visibility("default")))
  #define DEPTH_IMAGE_PROC_IMPORT

  #if __GNUC__ >= 4
    #define DEPTH_IMAGE_PROC_PUBLIC __attribute__ ((visibility("default")))
    #define DEPTH_IMAGE_PROC_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define DEPTH_IMAGE_PROC_PUBLIC
    #define DEPTH_IMAGE_PROC_LOCAL
  #endif

  #define DEPTH_IMAGE_PROC_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // DEPTH_IMAGE_PROC__VISIBILITY_H_
