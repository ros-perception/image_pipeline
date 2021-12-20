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

// Provide fake header guard for cpplint
#undef TRACETOOLS_IMAGE_PIPELINE__TP_CALL_H_
#ifndef TRACETOOLS_IMAGE_PIPELINE__TP_CALL_H_
#define TRACETOOLS_IMAGE_PIPELINE__TP_CALL_H_

#undef TRACEPOINT_PROVIDER
#define TRACEPOINT_PROVIDER ros2_image_pipeline

#undef TRACEPOINT_INCLUDE
#define TRACEPOINT_INCLUDE "tracetools_image_pipeline/tp_call.h"

#if !defined(_TRACETOOLS_IMAGE_PIPELINE__TP_CALL_H_) || defined(TRACEPOINT_HEADER_MULTI_READ)
#define _TRACETOOLS_IMAGE_PIPELINE__TP_CALL_H_

#include <lttng/tracepoint.h>

#include <stdint.h>
#include <stdbool.h>


// image_proc resize init callback
TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,       // tracepoint provider name
  image_proc_resize_init,    // tracepoint name
  TP_ARGS(
    // input arguments, see https://lttng.org/docs/v2.12/#doc-tpp-def-input-args
    const void *, resize_node_arg,
    const void *, resize_image_msg_arg,
    const void *, resize_info_msg_arg),
  TP_FIELDS(
    // output event fields, see https://lttng.org/man/3/lttng-ust/v2.12/#doc-ctf-macros
    ctf_integer_hex(const void *, resize_node, resize_node_arg)
    ctf_integer_hex(const void *, resize_image_msg, resize_image_msg_arg)
    ctf_integer_hex(const void *, resize_info_msg, resize_info_msg_arg)
    ctf_string(version, tracetools_image_pipeline_VERSION)
  )
)
// image_proc resize end of callback (after publication)
TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  image_proc_resize_fini,
  TP_ARGS(
    const void *, resize_node_arg,
    const void *, resize_image_msg_arg,
    const void *, resize_info_msg_arg),
  TP_FIELDS(
    ctf_integer_hex(const void *, resize_node, resize_node_arg)
    ctf_integer_hex(const void *, resize_image_msg, resize_image_msg_arg)
    ctf_integer_hex(const void *, resize_info_msg, resize_info_msg_arg)
    ctf_string(version, tracetools_image_pipeline_VERSION)
  )
)

// image_proc rectify init callback
TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,       // tracepoint provider name
  image_proc_rectify_init,    // tracepoint name
  TP_ARGS(
    // input arguments, see https://lttng.org/docs/v2.12/#doc-tpp-def-input-args
    const void *, rectify_node_arg,
    const void *, rectify_image_msg_arg,
    const void *, rectify_info_msg_arg),
  TP_FIELDS(
    // output event fields, see https://lttng.org/man/3/lttng-ust/v2.12/#doc-ctf-macros
    ctf_integer_hex(const void *, rectify_node, rectify_node_arg)
    ctf_integer_hex(const void *, rectify_image_msg, rectify_image_msg_arg)
    ctf_integer_hex(const void *, rectify_info_msg, rectify_info_msg_arg)
    ctf_string(version, tracetools_image_pipeline_VERSION)
  )
)
// image_proc rectify end of callback (after publication)
TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  image_proc_rectify_fini,
  TP_ARGS(
    const void *, rectify_node_arg,
    const void *, rectify_image_msg_arg,
    const void *, rectify_info_msg_arg),
  TP_FIELDS(
    ctf_integer_hex(const void *, rectify_node, rectify_node_arg)
    ctf_integer_hex(const void *, rectify_image_msg, rectify_image_msg_arg)
    ctf_integer_hex(const void *, rectify_info_msg, rectify_info_msg_arg)
    ctf_string(version, tracetools_image_pipeline_VERSION)
  )
)

#endif  // _TRACETOOLS_IMAGE_PIPELINE__TP_CALL_H_

#include <lttng/tracepoint-event.h>

#endif  // TRACETOOLS_IMAGE_PIPELINE__TP_CALL_H_
