/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2018, The MITRE Corporation.
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
*   * Neither the name of The MITRE Corporation nor the names of its
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

#include "visionworks_image_proc/visionworks_cv_helpers.h"
#include <OVX/UtilityOVX.hpp>

int stereo_visionworks::CopyCvMatToVxImage(vx_image _vxImage,
                                           vx_context _context,
                                           const cv::Mat& _cvImage)
{
  vx_uint32 width = _cvImage.cols;
  vx_uint32 height = _cvImage.rows;
  vx_df_image color;
  switch( _cvImage.depth() )
  {
  case CV_8U:
    color = VX_DF_IMAGE_U8;
    break;
  case CV_16U:
    color = VX_DF_IMAGE_U16;
    break;
  case CV_16S:
    color = VX_DF_IMAGE_S16;
    break;
  case CV_32S:
    color = VX_DF_IMAGE_S32;
    break;
  default:
    return -1;
    break;
  } //end: switch( _cvImage.depth() )

  
  vx_imagepatch_addressing_t addr;
  addr.dim_x = width;
  addr.dim_y = height;
  addr.stride_x = static_cast<vx_uint32>(_cvImage.elemSize());
  addr.stride_y = static_cast<vx_uint32>(_cvImage.step.p[0]);

  vx_uint8* vxImageData = _cvImage.data;
  _vxImage = vxCreateImage(_context, width, height, color);

  vx_rectangle_t rect;
  vxGetValidRegionImage(_vxImage, &rect);
  vxCopyImagePatch(_vxImage, &rect, 0, &addr, vxImageData, VX_WRITE_ONLY, VX_MEMORY_TYPE_HOST);

  return 0;
} //end: CopyCvMatToVxImage()

void stereo_visionworks::CreateVxMaskImage(vx_image _imageMask,
                                           vx_context _context,
                                           const unsigned int _imageWidth,
                                           const unsigned int _imageHeight,
                                           const unsigned int _maskStartX,
                                           const unsigned int _maskStartY,
                                           const unsigned int _maskWidth,
                                           const unsigned int _maskHeight,
                                           const unsigned char _maskValue)
{
  int maskWidth = std::min<unsigned int>(static_cast<const unsigned int>(_maskWidth), static_cast<const unsigned int>(_imageWidth));
  int maskHeight = std::min<unsigned int>(static_cast<const unsigned int>(_maskHeight), static_cast<const unsigned int>(_imageHeight));

  cv::Mat imageMaskCv(_imageHeight, _imageWidth,  CV_8U, cv::Scalar(_maskValue));
  imageMaskCv(cv::Rect(_maskStartX, _maskStartY, _maskWidth, _maskHeight)) = 0;

  CopyCvMatToVxImage(_imageMask, _context, imageMaskCv);
  NVXIO_CHECK_REFERENCE(_imageMask);
} //end: CreateVXMaskImage()
