/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#include "yuv422.h"

#define CLIP_CHAR(c) ((c)>255?255:(c)<0?0:(c))

namespace image_proc {

void yuv422ToGray(const cv::Mat& yuv, cv::Mat& gray)
{
  unsigned width = gray.cols;
  unsigned height = gray.rows;
  unsigned gray_skip = gray.step[0] - width;
  unsigned yuv_skip = yuv.step[0] - width*2;
  unsigned char* gray_buffer = gray.datastart;
  const unsigned char* yuv_buffer = yuv.datastart;

  // u y1 v y2
  for( unsigned yIdx = 0; yIdx < height;
       ++yIdx, gray_buffer += gray_skip, yuv_buffer += yuv_skip )
  {
    for( unsigned xIdx = 0; xIdx < width;
         ++xIdx, ++gray_buffer, yuv_buffer += 2 )
    {
      *gray_buffer = yuv_buffer[1];
    }
  }
}

void yuv422ToColor(const cv::Mat& yuv, cv::Mat& color)
{
  unsigned width = color.cols;
  unsigned height = color.rows;
  unsigned bgr_skip = color.step[0] - width*3;
  unsigned yuv_skip = yuv.step[0] - width*2;
  unsigned char* bgr_buffer = color.datastart;
  const unsigned char* yuv_buffer = yuv.datastart;

  // 0  1   2  3
  // u  y1  v  y2
  for( unsigned yIdx = 0; yIdx < height;
       ++yIdx, bgr_buffer += bgr_skip, yuv_buffer += yuv_skip )
  {
    for( unsigned xIdx = 0; xIdx < width;
         xIdx += 2, bgr_buffer += 6, yuv_buffer += 4 )
    {
      int v = yuv_buffer[2] - 128;
      int u = yuv_buffer[0] - 128;

      bgr_buffer[0] =  CLIP_CHAR (yuv_buffer[1] + ((u * 33292 + 8192 ) >> 14));
      bgr_buffer[1] =  CLIP_CHAR (yuv_buffer[1] + ((v * -9519 - u * 6472 + 8192 ) >> 14));
      bgr_buffer[2] =  CLIP_CHAR (yuv_buffer[1] + ((v * 18678 + 8192 ) >> 14));

      bgr_buffer[3] =  CLIP_CHAR (yuv_buffer[3] + ((u * 33292 + 8192 ) >> 14));
      bgr_buffer[4] =  CLIP_CHAR (yuv_buffer[3] + ((v * -9519 - u * 6472 + 8192 ) >> 14));
      bgr_buffer[5] =  CLIP_CHAR (yuv_buffer[3] + ((v * 18678 + 8192 ) >> 14));
    }
  }
}

} // namespace image_proc
