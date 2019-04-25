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
 * @file visionworks_interface.h
 * @brief Interface from ROS into VisionWorks code
 * @author Zach LaCelle (zlacelle@mitre.org)
 */

#include <cstdint>
#include <opencv2/core/mat.hpp>

namespace stereo_visionworks
{
  /**
   * @enum StereoType
   * @brief Types of algorithms we support through this interface
   */
  enum StereoType
  {
    STEREO_TYPE_BM    = 0, ///< Stereo Block Matching
    STEREO_TYPE_SGBM  = 1  ///< Semi-Global Block Matching
  };

  /**
   * @class StereoParams
   * @ingroup VisionWorks
   * @brief Contains parameters used by VisionWorks code for stereo processing.
   *
   * This matches very closely with parameters already in use by ROS in the
   * stereo_image_proc functions. 
   *
   * Not all of these are used for all algorithms. For instance, SGBM uses
   * P1 and P2, but BM does not.
   */ 
  class StereoParams
  {
  public:
    ///////////////////
    // disparity values
    uint32_t minDisparity_;    ///< Minimum disparity from [0-256]
    uint32_t maxDisparity_;    ///< Maximum disparity from [0-256]

    //////////////////////////
    // discontinuity penalties
    uint32_t p1_;              ///< [0,256]
    uint32_t p2_;              ///< [0,256]

    //////////////////
    // SAD window size
    uint32_t sadWindowSize_;   ///< [0,31]

    ///////////////////////////////
    // Census Transform window size
    uint32_t ctWinSize_;       ///< [0,5]

    ///////////////////////////
    // Hamming Cost window size
    uint32_t hcWinSize_;       ///< [0,5]

    /////////////////////
    // BT-cost clip value
    uint32_t btClipValue_;     ///< [15,95]

    ///////////////////////
    // Validation threshold
    uint32_t maxDiff_;         ///< [0,sizeof(uint32)]
    uint32_t uniquenessRatio_; ///< [0,100]
    uint32_t  scanlinesMask_;  ///< [0,256]

    ////////////////////////////////
    // Postprocess Speckle Filtering
    /**
     * @brief The number of pixels below which a disparity "blob"
     * is considered a speckle.
     *
     * Valid range is [0,1000]
     */
    uint32_t speckleSize_;
    /**
     * @brief How close in disparity two pixels need to be to be
     * part of a blob.
     *
     * Valid range is [0,31]
     */
    uint32_t speckleRange_;

    /**
     * @brief Default constructor for StereoParams, based on the VisionWorks sample code.
     */
    StereoParams();
    /**
     * @brief Constructor for StereoParams
     */
    StereoParams(uint32_t _minDisparity,
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
                 uint32_t _speckleRange);
    /**
     * @brief Resets stereo parameters to "sane" values
     * @return none
     */
    void Reset();
  }; //end: class StereoParams

  /**
   * @class VisionWorksInterface
   * @ingroup VisionWorks
   * @brief Private Implementation (pImpl) style interface to VisionWorks functionality
   *
   * This class provides an opaque interface to VisionWorks functions, via a set of parameters
   * stored within the class and a RunStereo() function. The goal of this type of interface
   * is to simplify and clearly delineate between traditional OpenCV calls and the NVIDIA
   * VisionWorks implementation.
   */
  class VisionWorksInterface
  {
  public:
    VisionWorksInterface();
    VisionWorksInterface(const StereoParams& _params);

    /**
     * @brief Set stereo parameters
     * @param [in] _params The input parameters
     * @return none
     */
    void SetParams(const StereoParams& _params);
    
    /**
     * @brief Main interface function to run stereo algorithms, selected
     * using the StereoType value.
     * @param [out] _disparity The already-initialized disparity output image, in S16 format (Q11.4).
     * @param [in] _stereoType The type of stereo algorithm to run
     * @param [in] _leftRect The rectified left image
     * @param [in] _rightRect The rectified right image
     * @param [in] _printPerf Whether we should print pipeline performance metrics (true: print metrics, false: do not print metrics)
     *
     * This function creates an object with parent type BaseStereo (currently, BM or SGBM) and runs the stereo pipeline associated
     * with that object. The OpenCV matrix _disparity is then filled.
     */
    void RunStereo(cv::Mat _disparity,
                   const StereoType& _stereoType,
                   const cv::Mat& _leftRect,
                   const cv::Mat& _rightRect,
                   const bool& _printPerf = false);
  private:
    StereoParams params_; ///< Current stereo parameter settings for VisionWorks function calls
  }; //end: class VisionWorksInterface
} //end: namespace stereo_visionworks
