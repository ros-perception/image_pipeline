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

#include "image_proc/edge_aware.hpp"

namespace image_proc
{

constexpr int avg(int a, int b)
{
  return (a + b) >> 1;
}

constexpr int avg3(int a, int b, int c)
{
  return (a + b + c) / 3;
}

constexpr int avg4(int a, int b, int c, int d)
{
  return (a + b + c + d) >> 2;
}

constexpr int wavg4(int a, int b, int c, int d, int x, int y)
{
  return ((a + b) * x + (c + d) * y) / (2 * (x + y));
}

void debayerEdgeAware(const cv::Mat & bayer, cv::Mat & color)
{
  unsigned width = bayer.cols;
  unsigned height = bayer.rows;
  unsigned rgb_line_step = color.step[0];
  unsigned rgb_line_skip = rgb_line_step - width * 3;
  int bayer_line_step = bayer.step[0];
  int bayer_line_step2 = bayer_line_step * 2;

  unsigned char * rgb_buffer = color.data;
  unsigned char * bayer_pixel = bayer.data;
  unsigned yIdx, xIdx;

  int dh, dv;

  // first two pixel values for first two lines
  // Bayer         0 1 2
  //         0     G r g
  // line_step     b g b
  // line_step2    g r g

  rgb_buffer[3] = rgb_buffer[0] = bayer_pixel[1];  // red pixel
  rgb_buffer[1] = bayer_pixel[0];  // green pixel
  rgb_buffer[rgb_line_step + 2] = rgb_buffer[2] = bayer_pixel[bayer_line_step];  // blue;

  // Bayer         0 1 2
  //         0     g R g
  // line_step     b g b
  // line_step2    g r g
  // rgb_pixel[3] = bayer_pixel[1];
  rgb_buffer[4] = avg3(bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1]);
  rgb_buffer[rgb_line_step + 5] = rgb_buffer[5] =
    avg(bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

  // BGBG line
  // Bayer         0 1 2
  //         0     g r g
  // line_step     B g b
  // line_step2    g r g
  rgb_buffer[rgb_line_step + 3] = rgb_buffer[rgb_line_step] =
    avg(bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
  rgb_buffer[rgb_line_step + 1] =
    avg3(bayer_pixel[0], bayer_pixel[bayer_line_step + 1], bayer_pixel[bayer_line_step2]);
  // rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

  // pixel (1, 1)  0 1 2
  //         0     g r g
  // line_step     b G b
  // line_step2    g r g
  // rgb_pixel[rgb_line_step + 3] = avg(bayer_pixel[1] , bayer_pixel[line_step2+1] );
  rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
  // rgb_pixel[rgb_line_step + 5] = avg(bayer_pixel[line_step] , bayer_pixel[line_step+2] );

  rgb_buffer += 6;
  bayer_pixel += 2;
  // rest of the first two lines
  for (xIdx = 2; xIdx < width - 2; xIdx += 2, rgb_buffer += 6, bayer_pixel += 2) {
    // GRGR line
    // Bayer        -1 0 1 2
    //           0   r G r g
    //   line_step   g b g b
    // line_step2    r g r g
    rgb_buffer[0] = avg(bayer_pixel[1], bayer_pixel[-1]);
    rgb_buffer[1] = bayer_pixel[0];
    rgb_buffer[2] = bayer_pixel[bayer_line_step + 1];

    // Bayer        -1 0 1 2
    //          0    r g R g
    //  line_step    g b g b
    // line_step2    r g r g
    rgb_buffer[3] = bayer_pixel[1];
    rgb_buffer[4] = avg3(bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1]);
    rgb_buffer[rgb_line_step + 5] = rgb_buffer[5] =
      avg(bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

    // BGBG line
    // Bayer         -1 0 1 2
    //         0      r g r g
    // line_step      g B g b
    // line_step2     r g r g
    rgb_buffer[rgb_line_step] = avg4(
      bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1],
      bayer_pixel[-1], bayer_pixel[bayer_line_step2 - 1]);
    rgb_buffer[rgb_line_step + 1] = avg4(
      bayer_pixel[0], bayer_pixel[bayer_line_step2],
      bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
    rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

    // Bayer         -1 0 1 2
    //         0      r g r g
    // line_step      g b G b
    // line_step2     r g r g
    rgb_buffer[rgb_line_step + 3] = avg(bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
    rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
    // rgb_pixel[rgb_line_step + 5] = avg(bayer_pixel[line_step] , bayer_pixel[line_step+2] );
  }

  // last two pixel values for first two lines
  // GRGR line
  // Bayer        -1 0 1
  //           0   r G r
  //   line_step   g b g
  // line_step2    r g r
  rgb_buffer[0] = avg(bayer_pixel[1], bayer_pixel[-1]);
  rgb_buffer[1] = bayer_pixel[0];
  rgb_buffer[rgb_line_step + 5] = rgb_buffer[rgb_line_step + 2] =
    rgb_buffer[5] = rgb_buffer[2] = bayer_pixel[bayer_line_step];

  // Bayer        -1 0 1
  //          0    r g R
  //  line_step    g b g
  // line_step2    r g r
  rgb_buffer[3] = bayer_pixel[1];
  rgb_buffer[4] = avg(bayer_pixel[0], bayer_pixel[bayer_line_step + 1]);
  // rgb_pixel[5] = bayer_pixel[line_step];

  // BGBG line
  // Bayer        -1 0 1
  //          0    r g r
  //  line_step    g B g
  // line_step2    r g r
  rgb_buffer[rgb_line_step] = avg4(
    bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1],
    bayer_pixel[-1], bayer_pixel[bayer_line_step2 - 1]);
  rgb_buffer[rgb_line_step + 1] = avg4(
    bayer_pixel[0], bayer_pixel[bayer_line_step2],
    bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
  // rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

  // Bayer         -1 0 1
  //         0      r g r
  // line_step      g b G
  // line_step2     r g r
  rgb_buffer[rgb_line_step + 3] = avg(bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
  rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
  // rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];

  bayer_pixel += bayer_line_step + 2;
  rgb_buffer += rgb_line_step + 6 + rgb_line_skip;

  // main processing
  for (yIdx = 2; yIdx < height - 2; yIdx += 2) {
    // first two pixel values
    // Bayer         0 1 2
    //        -1     b g b
    //         0     G r g
    // line_step     b g b
    // line_step2    g r g

    rgb_buffer[3] = rgb_buffer[0] = bayer_pixel[1];  // red pixel
    rgb_buffer[1] = bayer_pixel[0];  // green pixel
    rgb_buffer[2] = avg(bayer_pixel[bayer_line_step], bayer_pixel[-bayer_line_step]);  // blue;

    // Bayer         0 1 2
    //        -1     b g b
    //         0     g R g
    // line_step     b g b
    // line_step2    g r g
    // rgb_pixel[3] = bayer_pixel[1];
    rgb_buffer[4] = avg4(
      bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1],
      bayer_pixel[1 - bayer_line_step]);
    rgb_buffer[5] = avg4(
      bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2],
      bayer_pixel[-bayer_line_step], bayer_pixel[2 - bayer_line_step]);

    // BGBG line
    // Bayer         0 1 2
    //         0     g r g
    // line_step     B g b
    // line_step2    g r g
    rgb_buffer[rgb_line_step + 3] = rgb_buffer[rgb_line_step] =
      avg(bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
    rgb_buffer[rgb_line_step + 1] = avg3(
      bayer_pixel[0], bayer_pixel[bayer_line_step + 1], bayer_pixel[bayer_line_step2]);
    rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

    // pixel (1, 1)  0 1 2
    //         0     g r g
    // line_step     b G b
    // line_step2    g r g
    // rgb_pixel[rgb_line_step + 3] = avg(bayer_pixel[1] , bayer_pixel[line_step2+1] );
    rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
    rgb_buffer[rgb_line_step + 5] =
      avg(bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

    rgb_buffer += 6;
    bayer_pixel += 2;

    // continue with rest of the line
    for (xIdx = 2; xIdx < width - 2; xIdx += 2, rgb_buffer += 6, bayer_pixel += 2) {
      // GRGR line
      // Bayer        -1 0 1 2
      //          -1   g b g b
      //           0   r G r g
      //   line_step   g b g b
      // line_step2    r g r g
      rgb_buffer[0] = avg(bayer_pixel[1], bayer_pixel[-1]);
      rgb_buffer[1] = bayer_pixel[0];
      rgb_buffer[2] = avg(bayer_pixel[bayer_line_step], bayer_pixel[-bayer_line_step]);

      // Bayer        -1 0 1 2
      //          -1   g b g b
      //          0    r g R g
      //  line_step    g b g b
      // line_step2    r g r g

      dh = std::abs(bayer_pixel[0] - bayer_pixel[2]);
      dv = std::abs(bayer_pixel[-bayer_line_step + 1] - bayer_pixel[bayer_line_step + 1]);

      if (dh > dv) {
        rgb_buffer[4] = avg(bayer_pixel[-bayer_line_step + 1], bayer_pixel[bayer_line_step + 1]);
      } else if (dv > dh) {
        rgb_buffer[4] = avg(bayer_pixel[0], bayer_pixel[2]);
      } else {
        rgb_buffer[4] = avg4(
          bayer_pixel[-bayer_line_step + 1], bayer_pixel[bayer_line_step + 1],
          bayer_pixel[0], bayer_pixel[2]);
      }

      rgb_buffer[3] = bayer_pixel[1];
      rgb_buffer[5] = avg4(
        bayer_pixel[-bayer_line_step], bayer_pixel[2 - bayer_line_step],
        bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

      // BGBG line
      // Bayer         -1 0 1 2
      //         -1     g b g b
      //          0     r g r g
      // line_step      g B g b
      // line_step2     r g r g
      rgb_buffer[rgb_line_step] = avg4(
        bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1],
        bayer_pixel[-1], bayer_pixel[bayer_line_step2 - 1]);
      rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

      dv = std::abs(bayer_pixel[0] - bayer_pixel[bayer_line_step2]);
      dh = std::abs(bayer_pixel[bayer_line_step - 1] - bayer_pixel[bayer_line_step + 1]);

      if (dv > dh) {
        rgb_buffer[rgb_line_step + 1] =
          avg(bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
      } else if (dh > dv) {
        rgb_buffer[rgb_line_step + 1] = avg(bayer_pixel[0], bayer_pixel[bayer_line_step2]);
      } else {
        rgb_buffer[rgb_line_step + 1] = avg4(
          bayer_pixel[0], bayer_pixel[bayer_line_step2], bayer_pixel[bayer_line_step - 1],
          bayer_pixel[bayer_line_step + 1]);
      }

      // Bayer         -1 0 1 2
      //         -1     g b g b
      //          0     r g r g
      // line_step      g b G b
      // line_step2     r g r g
      rgb_buffer[rgb_line_step + 3] = avg(bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
      rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
      rgb_buffer[rgb_line_step + 5] =
        avg(bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);
    }

    // last two pixels of the line
    // last two pixel values for first two lines
    // GRGR line
    // Bayer        -1 0 1
    //           0   r G r
    //   line_step   g b g
    // line_step2    r g r
    rgb_buffer[0] = avg(bayer_pixel[1], bayer_pixel[-1]);
    rgb_buffer[1] = bayer_pixel[0];
    rgb_buffer[rgb_line_step + 5] = rgb_buffer[rgb_line_step + 2] =
      rgb_buffer[5] = rgb_buffer[2] = bayer_pixel[bayer_line_step];

    // Bayer        -1 0 1
    //          0    r g R
    //  line_step    g b g
    // line_step2    r g r
    rgb_buffer[3] = bayer_pixel[1];
    rgb_buffer[4] = avg(bayer_pixel[0], bayer_pixel[bayer_line_step + 1]);
    // rgb_pixel[5] = bayer_pixel[line_step];

    // BGBG line
    // Bayer        -1 0 1
    //          0    r g r
    //  line_step    g B g
    // line_step2    r g r
    rgb_buffer[rgb_line_step] = avg4(
      bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1],
      bayer_pixel[-1], bayer_pixel[bayer_line_step2 - 1]);
    rgb_buffer[rgb_line_step + 1] = avg4(
      bayer_pixel[0], bayer_pixel[bayer_line_step2],
      bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
    // rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

    // Bayer         -1 0 1
    //         0      r g r
    // line_step      g b G
    // line_step2     r g r
    rgb_buffer[rgb_line_step + 3] = avg(bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
    rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
    // rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];

    bayer_pixel += bayer_line_step + 2;
    rgb_buffer += rgb_line_step + 6 + rgb_line_skip;
  }

  // last two lines
  // Bayer         0 1 2
  //        -1     b g b
  //         0     G r g
  // line_step     b g b

  rgb_buffer[rgb_line_step + 3] = rgb_buffer[rgb_line_step] =
    rgb_buffer[3] = rgb_buffer[0] = bayer_pixel[1];  // red pixel
  rgb_buffer[1] = bayer_pixel[0];  // green pixel
  rgb_buffer[rgb_line_step + 2] = rgb_buffer[2] = bayer_pixel[bayer_line_step];  // blue;

  // Bayer         0 1 2
  //        -1     b g b
  //         0     g R g
  // line_step     b g b
  // rgb_pixel[3] = bayer_pixel[1];
  rgb_buffer[4] = avg4(
    bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1],
    bayer_pixel[1 - bayer_line_step]);
  rgb_buffer[5] = avg4(
    bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2],
    bayer_pixel[-bayer_line_step], bayer_pixel[2 - bayer_line_step]);

  // BGBG line
  // Bayer         0 1 2
  //        -1     b g b
  //         0     g r g
  // line_step     B g b
  // rgb_pixel[rgb_line_step] = bayer_pixel[1];
  rgb_buffer[rgb_line_step + 1] = avg(bayer_pixel[0], bayer_pixel[bayer_line_step + 1]);
  rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

  // Bayer         0 1 2
  //        -1     b g b
  //         0     g r g
  // line_step     b G b
  // rgb_pixel[rgb_line_step + 3] = avg(bayer_pixel[1] , bayer_pixel[line_step2+1] );
  rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
  rgb_buffer[rgb_line_step + 5] =
    avg(bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

  rgb_buffer += 6;
  bayer_pixel += 2;

  // rest of the last two lines
  for (xIdx = 2; xIdx < width - 2; xIdx += 2, rgb_buffer += 6, bayer_pixel += 2) {
    // GRGR line
    // Bayer       -1 0 1 2
    //        -1    g b g b
    //         0    r G r g
    // line_step    g b g b
    rgb_buffer[0] = avg(bayer_pixel[1], bayer_pixel[-1]);
    rgb_buffer[1] = bayer_pixel[0];
    rgb_buffer[2] = avg(bayer_pixel[bayer_line_step], bayer_pixel[-bayer_line_step]);

    // Bayer       -1 0 1 2
    //        -1    g b g b
    //         0    r g R g
    // line_step    g b g b
    rgb_buffer[rgb_line_step + 3] = rgb_buffer[3] = bayer_pixel[1];
    rgb_buffer[4] = avg4(
      bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1],
      bayer_pixel[1 - bayer_line_step]);
    rgb_buffer[5] = avg4(
      bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2],
      bayer_pixel[-bayer_line_step], bayer_pixel[-bayer_line_step + 2]);

    // BGBG line
    // Bayer       -1 0 1 2
    //        -1    g b g b
    //         0    r g r g
    // line_step    g B g b
    rgb_buffer[rgb_line_step] = avg(bayer_pixel[-1], bayer_pixel[1]);
    rgb_buffer[rgb_line_step + 1] = avg3(
      bayer_pixel[0], bayer_pixel[bayer_line_step - 1],
      bayer_pixel[bayer_line_step + 1]);
    rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

    // Bayer       -1 0 1 2
    //        -1    g b g b
    //         0    r g r g
    // line_step    g b G b
    // rgb_pixel[rgb_line_step + 3] = bayer_pixel[1];
    rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
    rgb_buffer[rgb_line_step + 5] =
      avg(bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);
  }

  // last two pixel values for first two lines
  // GRGR line
  // Bayer       -1 0 1
  //        -1    g b g
  //         0    r G r
  // line_step    g b g
  rgb_buffer[rgb_line_step] = rgb_buffer[0] = avg(bayer_pixel[1], bayer_pixel[-1]);
  rgb_buffer[1] = bayer_pixel[0];
  rgb_buffer[5] = rgb_buffer[2] = avg(bayer_pixel[bayer_line_step], bayer_pixel[-bayer_line_step]);

  // Bayer       -1 0 1
  //        -1    g b g
  //         0    r g R
  // line_step    g b g
  rgb_buffer[rgb_line_step + 3] = rgb_buffer[3] = bayer_pixel[1];
  rgb_buffer[4] = avg3(
    bayer_pixel[0], bayer_pixel[bayer_line_step + 1],
    bayer_pixel[-bayer_line_step + 1]);
  // rgb_pixel[5] = avg(bayer_pixel[line_step], bayer_pixel[-line_step] );

  // BGBG line
  // Bayer       -1 0 1
  //        -1    g b g
  //         0    r g r
  // line_step    g B g
  // rgb_pixel[rgb_line_step] = avg2(bayer_pixel[-1], bayer_pixel[1] );
  rgb_buffer[rgb_line_step + 1] = avg3(
    bayer_pixel[0], bayer_pixel[bayer_line_step - 1],
    bayer_pixel[bayer_line_step + 1]);
  rgb_buffer[rgb_line_step + 5] = rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

  // Bayer       -1 0 1
  //        -1    g b g
  //         0    r g r
  // line_step    g b G
  // rgb_pixel[rgb_line_step + 3] = bayer_pixel[1];
  rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
  // rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];
}

void debayerEdgeAwareWeighted(const cv::Mat & bayer, cv::Mat & color)
{
  unsigned width = bayer.cols;
  unsigned height = bayer.rows;
  unsigned rgb_line_step = color.step[0];
  unsigned rgb_line_skip = rgb_line_step - width * 3;
  int bayer_line_step = bayer.step[0];
  int bayer_line_step2 = bayer_line_step * 2;

  unsigned char * rgb_buffer = color.data;
  unsigned char * bayer_pixel = bayer.data;
  unsigned yIdx, xIdx;

  int dh, dv;

  // first two pixel values for first two lines
  // Bayer         0 1 2
  //         0     G r g
  // line_step     b g b
  // line_step2    g r g

  rgb_buffer[3] = rgb_buffer[0] = bayer_pixel[1];  // red pixel
  rgb_buffer[1] = bayer_pixel[0];  // green pixel
  rgb_buffer[rgb_line_step + 2] = rgb_buffer[2] = bayer_pixel[bayer_line_step];  // blue;

  // Bayer         0 1 2
  //         0     g R g
  // line_step     b g b
  // line_step2    g r g
  // rgb_pixel[3] = bayer_pixel[1];
  rgb_buffer[4] = avg3(bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1]);
  rgb_buffer[rgb_line_step + 5] = rgb_buffer[5] =
    avg(bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

  // BGBG line
  // Bayer         0 1 2
  //         0     g r g
  // line_step     B g b
  // line_step2    g r g
  rgb_buffer[rgb_line_step + 3] = rgb_buffer[rgb_line_step] =
    avg(bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
  rgb_buffer[rgb_line_step + 1] = avg3(
    bayer_pixel[0], bayer_pixel[bayer_line_step + 1],
    bayer_pixel[bayer_line_step2]);
  // rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

  // pixel (1, 1)  0 1 2
  //         0     g r g
  // line_step     b G b
  // line_step2    g r g
  // rgb_pixel[rgb_line_step + 3] = avg(bayer_pixel[1] , bayer_pixel[line_step2+1] );
  rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
  // rgb_pixel[rgb_line_step + 5] = avg(bayer_pixel[line_step] , bayer_pixel[line_step+2]);

  rgb_buffer += 6;
  bayer_pixel += 2;

  // rest of the first two lines
  for (xIdx = 2; xIdx < width - 2; xIdx += 2, rgb_buffer += 6, bayer_pixel += 2) {
    // GRGR line
    // Bayer        -1 0 1 2
    //           0   r G r g
    //   line_step   g b g b
    // line_step2    r g r g
    rgb_buffer[0] = avg(bayer_pixel[1], bayer_pixel[-1]);
    rgb_buffer[1] = bayer_pixel[0];
    rgb_buffer[2] = bayer_pixel[bayer_line_step + 1];

    // Bayer        -1 0 1 2
    //          0    r g R g
    //  line_step    g b g b
    // line_step2    r g r g
    rgb_buffer[3] = bayer_pixel[1];
    rgb_buffer[4] = avg3(bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1]);
    rgb_buffer[rgb_line_step + 5] = rgb_buffer[5] =
      avg(bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

    // BGBG line
    // Bayer         -1 0 1 2
    //         0      r g r g
    // line_step      g B g b
    // line_step2     r g r g
    rgb_buffer[rgb_line_step] = avg4(
      bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1],
      bayer_pixel[-1], bayer_pixel[bayer_line_step2 - 1]);
    rgb_buffer[rgb_line_step + 1] = avg4(
      bayer_pixel[0], bayer_pixel[bayer_line_step2],
      bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
    rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

    // Bayer         -1 0 1 2
    //         0      r g r g
    // line_step      g b G b
    // line_step2     r g r g
    rgb_buffer[rgb_line_step + 3] = avg(bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
    rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
    // rgb_pixel[rgb_line_step + 5] = avg(bayer_pixel[line_step] , bayer_pixel[line_step+2] );
  }

  // last two pixel values for first two lines
  // GRGR line
  // Bayer        -1 0 1
  //           0   r G r
  //   line_step   g b g
  // line_step2    r g r
  rgb_buffer[0] = avg(bayer_pixel[1], bayer_pixel[-1]);
  rgb_buffer[1] = bayer_pixel[0];
  rgb_buffer[rgb_line_step + 5] = rgb_buffer[rgb_line_step + 2] =
    rgb_buffer[5] = rgb_buffer[2] = bayer_pixel[bayer_line_step];

  // Bayer        -1 0 1
  //          0    r g R
  //  line_step    g b g
  // line_step2    r g r
  rgb_buffer[3] = bayer_pixel[1];
  rgb_buffer[4] = avg(bayer_pixel[0], bayer_pixel[bayer_line_step + 1]);
  // rgb_pixel[5] = bayer_pixel[line_step];

  // BGBG line
  // Bayer        -1 0 1
  //          0    r g r
  //  line_step    g B g
  // line_step2    r g r
  rgb_buffer[rgb_line_step] = avg4(
    bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1],
    bayer_pixel[-1], bayer_pixel[bayer_line_step2 - 1]);
  rgb_buffer[rgb_line_step + 1] = avg4(
    bayer_pixel[0], bayer_pixel[bayer_line_step2],
    bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
  // rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

  // Bayer         -1 0 1
  //         0      r g r
  // line_step      g b G
  // line_step2     r g r
  rgb_buffer[rgb_line_step + 3] = avg(bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
  rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
  // rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];

  bayer_pixel += bayer_line_step + 2;
  rgb_buffer += rgb_line_step + 6 + rgb_line_skip;

  // main processing
  for (yIdx = 2; yIdx < height - 2; yIdx += 2) {
    // first two pixel values
    // Bayer         0 1 2
    //        -1     b g b
    //         0     G r g
    // line_step     b g b
    // line_step2    g r g

    rgb_buffer[3] = rgb_buffer[0] = bayer_pixel[1];  // red pixel
    rgb_buffer[1] = bayer_pixel[0];  // green pixel
    rgb_buffer[2] = avg(bayer_pixel[bayer_line_step], bayer_pixel[-bayer_line_step]);  // blue;

    // Bayer         0 1 2
    //        -1     b g b
    //         0     g R g
    // line_step     b g b
    // line_step2    g r g
    // rgb_pixel[3] = bayer_pixel[1];
    rgb_buffer[4] = avg4(
      bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1],
      bayer_pixel[1 - bayer_line_step]);
    rgb_buffer[5] = avg4(
      bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2],
      bayer_pixel[-bayer_line_step], bayer_pixel[2 - bayer_line_step]);

    // BGBG line
    // Bayer         0 1 2
    //         0     g r g
    // line_step     B g b
    // line_step2    g r g
    rgb_buffer[rgb_line_step + 3] = rgb_buffer[rgb_line_step] =
      avg(bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
    rgb_buffer[rgb_line_step + 1] = avg3(
      bayer_pixel[0], bayer_pixel[bayer_line_step + 1],
      bayer_pixel[bayer_line_step2]);
    rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

    // pixel (1, 1)  0 1 2
    //         0     g r g
    // line_step     b G b
    // line_step2    g r g
    // rgb_pixel[rgb_line_step + 3] = avg(bayer_pixel[1] , bayer_pixel[line_step2+1] );
    rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
    rgb_buffer[rgb_line_step + 5] =
      avg(bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

    rgb_buffer += 6;
    bayer_pixel += 2;

    // continue with rest of the line
    for (xIdx = 2; xIdx < width - 2; xIdx += 2, rgb_buffer += 6, bayer_pixel += 2) {
      // GRGR line
      // Bayer        -1 0 1 2
      //          -1   g b g b
      //           0   r G r g
      //   line_step   g b g b
      // line_step2    r g r g
      rgb_buffer[0] = avg(bayer_pixel[1], bayer_pixel[-1]);
      rgb_buffer[1] = bayer_pixel[0];
      rgb_buffer[2] = avg(bayer_pixel[bayer_line_step], bayer_pixel[-bayer_line_step]);

      // Bayer        -1 0 1 2
      //          -1   g b g b
      //          0    r g R g
      //  line_step    g b g b
      // line_step2    r g r g

      dh = std::abs(bayer_pixel[0] - bayer_pixel[2]);
      dv = std::abs(bayer_pixel[-bayer_line_step + 1] - bayer_pixel[bayer_line_step + 1]);

      if (dv == 0 && dh == 0) {
        rgb_buffer[4] = avg4(
          bayer_pixel[1 - bayer_line_step], bayer_pixel[1 + bayer_line_step],
          bayer_pixel[0], bayer_pixel[2]);
      } else {
        rgb_buffer[4] = wavg4(
          bayer_pixel[1 - bayer_line_step], bayer_pixel[1 + bayer_line_step],
          bayer_pixel[0], bayer_pixel[2], dh, dv);
      }

      rgb_buffer[3] = bayer_pixel[1];
      rgb_buffer[5] = avg4(
        bayer_pixel[-bayer_line_step], bayer_pixel[2 - bayer_line_step],
        bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

      // BGBG line
      // Bayer         -1 0 1 2
      //         -1     g b g b
      //          0     r g r g
      // line_step      g B g b
      // line_step2     r g r g
      rgb_buffer[rgb_line_step] = avg4(
        bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1],
        bayer_pixel[-1], bayer_pixel[bayer_line_step2 - 1]);
      rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

      dv = std::abs(bayer_pixel[0] - bayer_pixel[bayer_line_step2]);
      dh = std::abs(bayer_pixel[bayer_line_step - 1] - bayer_pixel[bayer_line_step + 1]);

      if (dv == 0 && dh == 0) {
        rgb_buffer[rgb_line_step + 1] = avg4(
          bayer_pixel[0], bayer_pixel[bayer_line_step2],
          bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
      } else {
        rgb_buffer[rgb_line_step + 1] = wavg4(
          bayer_pixel[0], bayer_pixel[bayer_line_step2], bayer_pixel[bayer_line_step - 1],
          bayer_pixel[bayer_line_step + 1], dh, dv);
      }

      // Bayer         -1 0 1 2
      //         -1     g b g b
      //          0     r g r g
      // line_step      g b G b
      // line_step2     r g r g
      rgb_buffer[rgb_line_step + 3] =
        avg(bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
      rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
      rgb_buffer[rgb_line_step + 5] =
        avg(bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);
    }

    // last two pixels of the line
    // last two pixel values for first two lines
    // GRGR line
    // Bayer        -1 0 1
    //           0   r G r
    //   line_step   g b g
    // line_step2    r g r
    rgb_buffer[0] = avg(bayer_pixel[1], bayer_pixel[-1]);
    rgb_buffer[1] = bayer_pixel[0];
    rgb_buffer[rgb_line_step + 5] = rgb_buffer[rgb_line_step + 2] =
      rgb_buffer[5] = rgb_buffer[2] = bayer_pixel[bayer_line_step];

    // Bayer        -1 0 1
    //          0    r g R
    //  line_step    g b g
    // line_step2    r g r
    rgb_buffer[3] = bayer_pixel[1];
    rgb_buffer[4] = avg(bayer_pixel[0], bayer_pixel[bayer_line_step + 1]);
    // rgb_pixel[5] = bayer_pixel[line_step];

    // BGBG line
    // Bayer        -1 0 1
    //          0    r g r
    //  line_step    g B g
    // line_step2    r g r
    rgb_buffer[rgb_line_step] = avg4(
      bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1],
      bayer_pixel[-1], bayer_pixel[bayer_line_step2 - 1]);
    rgb_buffer[rgb_line_step + 1] = avg4(
      bayer_pixel[0], bayer_pixel[bayer_line_step2],
      bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
    // rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

    // Bayer         -1 0 1
    //         0      r g r
    // line_step      g b G
    // line_step2     r g r
    rgb_buffer[rgb_line_step + 3] = avg(bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
    rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
    // rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];

    bayer_pixel += bayer_line_step + 2;
    rgb_buffer += rgb_line_step + 6 + rgb_line_skip;
  }

  // last two lines
  // Bayer         0 1 2
  //        -1     b g b
  //         0     G r g
  // line_step     b g b

  rgb_buffer[rgb_line_step + 3] = rgb_buffer[rgb_line_step] =
    rgb_buffer[3] = rgb_buffer[0] = bayer_pixel[1];  // red pixel
  rgb_buffer[1] = bayer_pixel[0];  // green pixel
  rgb_buffer[rgb_line_step + 2] = rgb_buffer[2] = bayer_pixel[bayer_line_step];  // blue;

  // Bayer         0 1 2
  //        -1     b g b
  //         0     g R g
  // line_step     b g b
  // rgb_pixel[3] = bayer_pixel[1];
  rgb_buffer[4] = avg4(
    bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1],
    bayer_pixel[1 - bayer_line_step]);
  rgb_buffer[5] = avg4(
    bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2],
    bayer_pixel[-bayer_line_step], bayer_pixel[2 - bayer_line_step]);

  // BGBG line
  // Bayer         0 1 2
  //        -1     b g b
  //         0     g r g
  // line_step     B g b
  // rgb_pixel[rgb_line_step] = bayer_pixel[1];
  rgb_buffer[rgb_line_step + 1] = avg(bayer_pixel[0], bayer_pixel[bayer_line_step + 1]);
  rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

  // Bayer         0 1 2
  //        -1     b g b
  //         0     g r g
  // line_step     b G b
  // rgb_pixel[rgb_line_step + 3] = avg(bayer_pixel[1] , bayer_pixel[line_step2+1] );
  rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
  rgb_buffer[rgb_line_step + 5] =
    avg(bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

  rgb_buffer += 6;
  bayer_pixel += 2;

  // rest of the last two lines
  for (xIdx = 2; xIdx < width - 2; xIdx += 2, rgb_buffer += 6, bayer_pixel += 2) {
    // GRGR line
    // Bayer       -1 0 1 2
    //        -1    g b g b
    //         0    r G r g
    // line_step    g b g b
    rgb_buffer[0] = avg(bayer_pixel[1], bayer_pixel[-1]);
    rgb_buffer[1] = bayer_pixel[0];
    rgb_buffer[2] = avg(bayer_pixel[bayer_line_step], bayer_pixel[-bayer_line_step]);

    // Bayer       -1 0 1 2
    //        -1    g b g b
    //         0    r g R g
    // line_step    g b g b
    rgb_buffer[rgb_line_step + 3] = rgb_buffer[3] = bayer_pixel[1];
    rgb_buffer[4] = avg4(
      bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1],
      bayer_pixel[1 - bayer_line_step]);
    rgb_buffer[5] = avg4(
      bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2],
      bayer_pixel[-bayer_line_step], bayer_pixel[-bayer_line_step + 2]);

    // BGBG line
    // Bayer       -1 0 1 2
    //        -1    g b g b
    //         0    r g r g
    // line_step    g B g b
    rgb_buffer[rgb_line_step] = avg(bayer_pixel[-1], bayer_pixel[1]);
    rgb_buffer[rgb_line_step + 1] =
      avg3(bayer_pixel[0], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
    rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

    // Bayer       -1 0 1 2
    //        -1    g b g b
    //         0    r g r g
    // line_step    g b G b
    // rgb_pixel[rgb_line_step + 3] = bayer_pixel[1];
    rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
    rgb_buffer[rgb_line_step + 5] =
      avg(bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);
  }

  // last two pixel values for first two lines
  // GRGR line
  // Bayer       -1 0 1
  //        -1    g b g
  //         0    r G r
  // line_step    g b g
  rgb_buffer[rgb_line_step] = rgb_buffer[0] = avg(bayer_pixel[1], bayer_pixel[-1]);
  rgb_buffer[1] = bayer_pixel[0];
  rgb_buffer[5] = rgb_buffer[2] = avg(bayer_pixel[bayer_line_step], bayer_pixel[-bayer_line_step]);

  // Bayer       -1 0 1
  //        -1    g b g
  //         0    r g R
  // line_step    g b g
  rgb_buffer[rgb_line_step + 3] = rgb_buffer[3] = bayer_pixel[1];
  rgb_buffer[4] = avg3(
    bayer_pixel[0], bayer_pixel[bayer_line_step + 1],
    bayer_pixel[-bayer_line_step + 1]);
  // rgb_pixel[5] = avg(bayer_pixel[line_step], bayer_pixel[-line_step] );

  // BGBG line
  // Bayer       -1 0 1
  //        -1    g b g
  //         0    r g r
  // line_step    g B g
  // rgb_pixel[rgb_line_step] = avg2(bayer_pixel[-1], bayer_pixel[1] );
  rgb_buffer[rgb_line_step + 1] = avg3(
    bayer_pixel[0], bayer_pixel[bayer_line_step - 1],
    bayer_pixel[bayer_line_step + 1]);
  rgb_buffer[rgb_line_step + 5] = rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

  // Bayer       -1 0 1
  //        -1    g b g
  //         0    r g r
  // line_step    g b G
  // rgb_pixel[rgb_line_step + 3] = bayer_pixel[1];
  rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
  // rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];
}

}  // namespace image_proc
