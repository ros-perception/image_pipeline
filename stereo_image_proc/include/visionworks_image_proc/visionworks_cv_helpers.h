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

#pragma once
/**
 * @file visionworks_cv_helpers.h
 * @brief OpenCV to OpenVX helper functions.
 * @author Zach LaCelle (zlacelle@mitre.org)
 */

#include <VX/vx.h>
#include <opencv2/opencv.hpp>

namespace stereo_visionworks
{
  /**
   * @brief Copy an OpenCV matrix to an OpenVX Image
   * @param [out] _vxImage The OpenVX Image to fill
   * @param [in] _context The OpenVX context for this action
   * @param [in] _cvImage The OpenCV matrix to copy into the OpenVX structure.
   * @return 0 of success, < 0 if error, > 0 if warning
   */
  int CopyCvMatToVxImage(vx_image _vxImage, vx_context _context, const cv::Mat& _cvImage);
  /**
   * @brief Create an OpenVX "masking" image, with most of the image 0 and a rectangular
   * section set to a custom mask value.
   * @param [out] _imageMask The completed masking image
   * @param [in] _context The OpenVX context for this action
   * @param [in] _imageWidth The width of the image to generate
   * @param [in] _imageHeight The height of the image to generate
   * @param [in] _maskStartX The starting point in the X direction of the mask, in pixels
   * @param [in] _maskStartY The starting point in the Y direction of the mask, in pixels
   * @param [in] _maskWidth The width of the mask, in pixels
   * @param [in] _maskHeight The height of the mask, in pixels
   * @param [in] _maskValue The custom pixel value to apply to pixels located with the masking rectangle
   */
  void CreateVxMaskImage(vx_image _imageMask,
                         vx_context _context,
                         const unsigned int _imageWidth,
                         const unsigned int _imageHeight,
                         const unsigned int _maskStartX,
                         const unsigned int _maskStartY,
                         const unsigned int _maskWidth,
                         const unsigned int _maskHeight,
                         const unsigned char _maskValue);
} //end: namespace stereo_visionworks
