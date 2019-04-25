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

#include "visionworks_image_proc/visionworks_interface.h"
#include "visionworks_image_proc/visionworks_matching.h"

#include <NVX/nvx_opencv_interop.hpp>
//#include <NVX/nvx.h>

stereo_visionworks::StereoParams::StereoParams() :
  minDisparity_(0),
  maxDisparity_(64),
  p1_(8),
  p2_(109),
  sadWindowSize_(5),
  btClipValue_(31),
  maxDiff_(32000),
  uniquenessRatio_(0),
  scanlinesMask_(85),
  ctWinSize_(0),
  hcWinSize_(1),
  speckleSize_(400),
  speckleRange_(4)
{}
stereo_visionworks::StereoParams::StereoParams(uint32_t _minDisparity,
                                               uint32_t _maxDisparity,
                                               uint32_t _p1,
                                               uint32_t _p2,
                                               uint32_t _sadWindowSize,
                                               uint32_t _ctWinSize,
                                               uint32_t _hcWinSize,
                                               uint32_t _btClipValue,
                                               uint32_t _maxDiff,
                                               uint32_t _uniquenessRatio,
                                               uint32_t _speckleSize,
                                               uint32_t _speckleRange) :
  minDisparity_(_minDisparity),
  maxDisparity_(_maxDisparity),
  p1_(_p1),
  p2_(_p2),
  sadWindowSize_(_sadWindowSize),
  ctWinSize_(_ctWinSize),
  hcWinSize_(_hcWinSize),
  btClipValue_(_btClipValue),
  maxDiff_(_maxDiff),
  uniquenessRatio_(_uniquenessRatio),
  speckleSize_(_speckleSize),
  speckleRange_(_speckleRange)
{}

void stereo_visionworks::StereoParams::Reset()
{
  minDisparity_ = 0;
  maxDisparity_ = 64;
  p1_ = 8;
  p2_ = 109;
  sadWindowSize_ = 5;
  btClipValue_ = 31;
  maxDiff_ = 32000;
  uniquenessRatio_ = 0;
  scanlinesMask_ = 85;
  ctWinSize_ = 0;
  hcWinSize_ = 1;
  speckleSize_ = 400;
  speckleRange_ = 4;
} //end: Reset()

stereo_visionworks::VisionWorksInterface::VisionWorksInterface()
{
  params_.Reset();
} //end: VisionWorksInterface()

stereo_visionworks::VisionWorksInterface::VisionWorksInterface(const StereoParams& _params)
{
  params_ = _params;
} //end: VisionWorksInterface()

void stereo_visionworks::VisionWorksInterface::SetParams(const StereoParams& _params)
{
  params_ = _params;
} //end: SetParams()

void stereo_visionworks::VisionWorksInterface::RunStereo(cv::Mat _disparity,
                                                         const StereoType& _stereoType,
                                                         const cv::Mat& _leftRect,
                                                         const cv::Mat& _rightRect,
                                                         const bool& _printPerf)
{
  ovxio::ContextGuard context;
  // To enable performance counters for this context, set:
  //  * VX_DIRECTIVE_ENABLE_PERFORMANCE
  // To disable performance counters for this context, set:
  //  * VX_DIRECTIVE_DISABLE_PERFORMANCE
  vxDirective(context, VX_DIRECTIVE_ENABLE_PERFORMANCE);
  // To enable logging for graph debugging this context, set:
  //  * VX_DIRECTIVE_ENABLE_LOGGING
  // To disable logging for graph debugging for this context, set:
  //  * VX_DIRECTIVE_DISABLE_LOGGING
  vxDirective(context, VX_DIRECTIVE_ENABLE_LOGGING);

  // Messages generated by the OpenVX Framework will be processed by
  // ovxio::stdoutLogCallback
  vxRegisterLogCallback(context, &ovxio::stdoutLogCallback, vx_false_e);

  // Copy left/right images into vx_image structures.
  // Note that the vx_image structures share the same memory as the cv::Mat
  // when using createVXImageFromCVMat().
  vx_image left_rect_vx = nvx_cv::createVXImageFromCVMat(context, _leftRect);
  NVXIO_CHECK_REFERENCE(left_rect_vx);
  vx_image right_rect_vx = nvx_cv::createVXImageFromCVMat(context, _rightRect);
  NVXIO_CHECK_REFERENCE(right_rect_vx);

  // Initialize our scratch buffers
  // Note that disparity16_ is used by the OpenCV functions
  // as well as our function here, and is the "output" of this
  // section of code (used later by a common "fill" function
  // for ROS)
  vx_image disparity_vx = nvx_cv::createVXImageFromCVMat(context, _disparity);
  NVXIO_CHECK_REFERENCE(disparity_vx);
    
  //Perform stereo matching calculation here
  stereo_visionworks::BaseStereo* stereo;
  if( _stereoType == stereo_visionworks::STEREO_TYPE_BM )
  {
    stereo = new stereo_visionworks::BM(disparity_vx, context, params_, left_rect_vx, right_rect_vx);
  } //end: if( _stereoType == stereo_visionworks::STEREO_TYPE_BM )
  else
  {
    stereo = new stereo_visionworks::SGBM(disparity_vx, context, params_, left_rect_vx, right_rect_vx);
  } //end: else

  // Run stereo pipeline
  // After this call, disparity16_ will have the results of the disparity
  // calculations.
  stereo->RunPipeline();  
  
  //Print performance metrics
  if( _printPerf )
  {
    stereo->PrintPerfs();
  } //end: if( _printPerf )
  
  //Cleanup
  vxReleaseImage(&left_rect_vx);
  vxReleaseImage(&right_rect_vx);
  vxReleaseImage(&disparity_vx);
  delete stereo;
  stereo = NULL;
} //end: RunStereo()

