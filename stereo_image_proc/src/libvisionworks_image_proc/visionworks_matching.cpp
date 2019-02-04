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

#include "visionworks_image_proc/visionworks_matching.h"
#include "visionworks_image_proc/masking_node.h"
#include "visionworks_image_proc/speckle_filter_node.h"

#include <opencv2/opencv.hpp>
#include <NVX/nvx_opencv_interop.hpp>

stereo_visionworks::SGBM::SGBM(vx_image _disparity,
                               vx_context _context,
                               const stereo_visionworks::StereoParams& _params,
                               vx_image _left,
                               vx_image _right)
{
  //Timing for the full loop
  timespec startTime;
  clock_gettime(CLOCK_MONOTONIC, &startTime);
  double startTime_s = (startTime.tv_sec)+(startTime.tv_nsec / 1e9);
    
  vx_df_image leftFormat = VX_DF_IMAGE_VIRT;
  vx_uint32 leftWidth = 0;
  vx_uint32 leftHeight = 0;
  greyColorConvertLastCycle_ = false;
  depthConvertLastCycle_ = false;
  imageMaskedLastCycle_ = false;
  
  //Get width/height from left image
  NVXIO_SAFE_CALL( vxQueryImage(_left, VX_IMAGE_ATTRIBUTE_FORMAT, &leftFormat, sizeof(leftFormat)) );
  NVXIO_SAFE_CALL( vxQueryImage(_left, VX_IMAGE_ATTRIBUTE_WIDTH, &leftWidth, sizeof(leftWidth)) );
  NVXIO_SAFE_CALL( vxQueryImage(_left, VX_IMAGE_ATTRIBUTE_HEIGHT, &leftHeight, sizeof(leftHeight)) );

  //Register custom VisionWorks kernels
  RegisterImageMaskingKernel(_context);
  
  //Create the graph
  mainGraph_ = vxCreateGraph(_context);
  NVXIO_CHECK_REFERENCE(mainGraph_);

  //Convert images to grayscale
  vx_image left_gray, right_gray;
  bool mustReleaseTempGrays = false;
  if( leftFormat != VX_DF_IMAGE_U8 )
  {
    left_gray = vxCreateVirtualImage(mainGraph_, leftWidth, leftHeight, VX_DF_IMAGE_U8);
    NVXIO_CHECK_REFERENCE(left_gray);
    right_gray = vxCreateVirtualImage(mainGraph_, leftWidth, leftHeight, VX_DF_IMAGE_U8);
    NVXIO_CHECK_REFERENCE(right_gray);
    
    left_cvt_color_node_ = vxColorConvertNode(mainGraph_, _left, left_gray);
    NVXIO_CHECK_REFERENCE(left_cvt_color_node_);
    right_cvt_color_node_ = vxColorConvertNode(mainGraph_, _right, right_gray);
    NVXIO_CHECK_REFERENCE(right_cvt_color_node_);

    greyColorConvertLastCycle_ = true;
  } //end: if( leftFormat != VX_DF_IMAGE_U8 )
  else
  {
    left_gray = _left;
    right_gray = _right;
  } //end: else

  // The SGM algorithm is now added as a node to the graph via the
  // nvxSemiGlobalMatchingNode().The input to the SGM node is previously
  // constructed left_gray and right_gray vx_images and the configuration
  // parameters. The output of the SGM node is the disparity image
  // that holds S16 fixed-point disparity values. The fixed-point values
  // have Q11.4 format (one sign bit, eleven integer bits and four
  // fractional bits).
  
  //Options for flags:
  // NVX_SGM_PYRAMIDAL_STEREO:
  //  Use pyramidal scheme: lower resolution imagery for nearby objects and the full
  //  resolution for far-away objects
  // NVX_SGM_FILTER_TOP_AREA:
  //  Filter cost at top image area with low gradients.
  vx_enum sgbmFlags = NVX_SGM_PYRAMIDAL_STEREO;
  semiGlobalBlockMatchingNode_ = nvxSemiGlobalMatchingNode(
    mainGraph_,
    left_gray,
    right_gray,
    _disparity,
    _params.minDisparity_,
    _params.maxDisparity_,
    _params.p1_,
    _params.p2_,
    _params.sadWindowSize_,
    _params.ctWinSize_,
    _params.hcWinSize_,
    _params.btClipValue_,
    _params.maxDiff_,
    _params.uniquenessRatio_,
    _params.scanlinesMask_,
    sgbmFlags);
  NVXIO_CHECK_REFERENCE(semiGlobalBlockMatchingNode_);

  // Mask off the left side of the image
  vx_uint8 maskValue = 0u;
  maskNode_ = ImageMaskingNode(mainGraph_,
                               _disparity,
                               0u,
                               0u,
                               _params.maxDisparity_,
                               leftHeight,
                               maskValue);
  imageMaskedLastCycle_ = true;
  NVXIO_CHECK_REFERENCE(maskNode_);
  
  // verify the graph
  NVXIO_SAFE_CALL( vxVerifyGraph(mainGraph_) );

  if( mustReleaseTempGrays )
  {
    vxReleaseImage(&left_gray);
    vxReleaseImage(&right_gray);
  }

  timespec endTime;
  clock_gettime(CLOCK_MONOTONIC, &endTime);
  double endTime_s = (endTime.tv_sec)+(endTime.tv_nsec / 1e9);
  lastSetupLoopTime_s_ = endTime_s - startTime_s;
} //end: SGBM()

void stereo_visionworks::SGBM::PrintPerfs() const
{
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  double monotime = ts.tv_sec + (ts.tv_nsec / 1e9);
  clock_gettime(CLOCK_REALTIME, &ts);
  double realtime = ts.tv_sec + (ts.tv_nsec / 1e9);
  printf("RT: %f, MT: %f\n", realtime, monotime);
  double loopSetupTime_ms = lastSetupLoopTime_s_ * 1000;
  printf("Loop Setup Time : %.3f ms\n", loopSetupTime_ms);
  ovxio::printPerf(mainGraph_, "Stereo");
  if( greyColorConvertLastCycle_ )
  {
    ovxio::printPerf(left_cvt_color_node_, "Left Color Convert");
    ovxio::printPerf(right_cvt_color_node_, "Right Color Convert");
  }
  ovxio::printPerf(semiGlobalBlockMatchingNode_, "SGBM");
  if( imageMaskedLastCycle_ )
  {
    ovxio::printPerf(maskNode_, "Image Masking Node");
  }
  if( depthConvertLastCycle_ )
  {
    ovxio::printPerf(convert_depth_node_, "Convert Depth");
  }
} //end: PrintPerfs()

stereo_visionworks::SGBM::~SGBM()
{
  vxReleaseGraph(&mainGraph_);
} //end: ~SGBM()

stereo_visionworks::BM::BM(vx_image _disparity,
                           vx_context _context,
                           const stereo_visionworks::StereoParams& _params,
                           vx_image _left,
                           vx_image _right)
{
  //Timing for the full loop
  timespec startTime;
  clock_gettime(CLOCK_MONOTONIC, &startTime);
  double startTime_s = (startTime.tv_sec)+(startTime.tv_nsec / 1e9);

  vx_df_image leftFormat = VX_DF_IMAGE_VIRT;
  vx_uint32 leftWidth = 0;
  vx_uint32 leftHeight = 0;
  greyColorConvertLastCycle_ = false;
  depthConvertLastCycle_ = false;
  speckleFilterLastCycle_ = false;
  
  //Get width/height from left image
  NVXIO_SAFE_CALL( vxQueryImage(_left, VX_IMAGE_ATTRIBUTE_FORMAT, &leftFormat, sizeof(leftFormat)) );
  NVXIO_SAFE_CALL( vxQueryImage(_left, VX_IMAGE_ATTRIBUTE_WIDTH, &leftWidth, sizeof(leftWidth)) );
  NVXIO_SAFE_CALL( vxQueryImage(_left, VX_IMAGE_ATTRIBUTE_HEIGHT, &leftHeight, sizeof(leftHeight)) );

  //Register custom kernels
  RegisterSpeckleFilterKernel(_context);
  
  //Create the graph
  mainGraph_ = vxCreateGraph(_context);
  NVXIO_CHECK_REFERENCE(mainGraph_);

  //Create our VX_DF_IMAGE_U8 scratch image, which will need to be converted to
  //VX_DF_IMAGE_S16 for output as _disparity
  vx_image disparity_u8 = vxCreateVirtualImage(mainGraph_, leftWidth, leftHeight, VX_DF_IMAGE_U8);
  NVXIO_CHECK_REFERENCE(disparity_u8);
  
  //Convert images to grayscale
  vx_image left_gray, right_gray;
  bool mustReleaseTempGrays = false;
  if( leftFormat != VX_DF_IMAGE_U8 )
  {
    left_gray = vxCreateVirtualImage(mainGraph_, leftWidth, leftHeight, VX_DF_IMAGE_U8);
    NVXIO_CHECK_REFERENCE(left_gray);
    right_gray = vxCreateVirtualImage(mainGraph_, leftWidth, leftHeight, VX_DF_IMAGE_U8);
    NVXIO_CHECK_REFERENCE(right_gray);
    
    left_cvt_color_node_ = vxColorConvertNode(mainGraph_, _left, left_gray);
    NVXIO_CHECK_REFERENCE(left_cvt_color_node_);
    right_cvt_color_node_ = vxColorConvertNode(mainGraph_, _right, right_gray);
    NVXIO_CHECK_REFERENCE(right_cvt_color_node_);

    greyColorConvertLastCycle_ = true;
  } //end: if( leftFormat != VX_DF_IMAGE_U8 )
  else
  {
    left_gray = _left;
    right_gray = _right;
  } //end: else

  // The BM algorithm is now added as a node to the graph via the
  // nvxSemiGlobalMatchingNode().The input to the BM node is previously
  // constructed left_gray and right_gray vx_images and the configuration
  // parameters. The output of the BM node is the disparity image
  // in VX_DF_IMAGE_U8 format.
  blockMatchingNode_ = nvxStereoBlockMatchingNode(
    mainGraph_,
    left_gray,
    right_gray,
    disparity_u8,
    _params.sadWindowSize_,
    _params.maxDisparity_);
  NVXIO_CHECK_REFERENCE(blockMatchingNode_);

  // Perform speckle filtering
  // This line is from OpenCV's filterSpeckles() function
  vx_uint32 filteredValue = (_params.minDisparity_ - 1) << 4;
  speckleFilterNode_ = SpeckleFilterNode(mainGraph_,
                                         disparity_u8,
                                         filteredValue,
                                         _params.speckleSize_,
                                         _params.speckleRange_);
  NVXIO_CHECK_REFERENCE(speckleFilterNode_);
  speckleFilterLastCycle_ = true;
  
  // Up-convert u8 to s16
  // Note: need to get it into Q11.4 (1 sign bit, 11 data bits, 4 fractional bits)
  vx_int32 shift = 4;
  vx_scalar upConvertShift = vxCreateScalar(_context, VX_TYPE_INT32, &shift);
  NVXIO_CHECK_REFERENCE(upConvertShift);
  // This is ignored for upConvert, but needs to be passed in anyways
  vx_convert_policy_e convertPolicy = VX_CONVERT_POLICY_SATURATE;
  convert_depth_node_ = vxConvertDepthNode(mainGraph_, disparity_u8, _disparity, VX_CONVERT_POLICY_SATURATE, upConvertShift);
  vxReleaseScalar(&upConvertShift);
  NVXIO_CHECK_REFERENCE(convert_depth_node_);
  depthConvertLastCycle_ = true;
  
  // verify the graph
  NVXIO_SAFE_CALL( vxVerifyGraph(mainGraph_) );

  // clean up
  vxReleaseImage(&disparity_u8);
  if( mustReleaseTempGrays )
  {
    vxReleaseImage(&left_gray);
    vxReleaseImage(&right_gray);
  }

  timespec endTime;
  clock_gettime(CLOCK_MONOTONIC, &endTime);
  double endTime_s = (endTime.tv_sec)+(endTime.tv_nsec / 1e9);
  lastSetupLoopTime_s_ = endTime_s - startTime_s;
} //end: BM()

void stereo_visionworks::BM::PrintPerfs() const
{
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  double monotime = ts.tv_sec + (ts.tv_nsec / 1e9);
  clock_gettime(CLOCK_REALTIME, &ts);
  double realtime = ts.tv_sec + (ts.tv_nsec / 1e9);
  printf("RT: %f, MT: %f\n", realtime, monotime);
  double loopSetupTime_ms = lastSetupLoopTime_s_ * 1000;
  printf("Loop Setup Time : %.3f ms\n", loopSetupTime_ms);
  ovxio::printPerf(mainGraph_, "Stereo");
  if( greyColorConvertLastCycle_ )
  {
    ovxio::printPerf(left_cvt_color_node_, "Left Color Convert");
    ovxio::printPerf(right_cvt_color_node_, "Right Color Convert");
  }
  ovxio::printPerf(blockMatchingNode_, "BM");
  if( speckleFilterLastCycle_ )
  {
    ovxio::printPerf(speckleFilterNode_, "Speckle Filter");
  }
  if( depthConvertLastCycle_ )
  {
    ovxio::printPerf(convert_depth_node_, "Convert Depth");
  }
} //end: PrintPerfs()

stereo_visionworks::BM::~BM()
{
  vxReleaseGraph(&mainGraph_);
} //end: ~BM()
