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
 * @file visionworks_matching.h
 * @brief Contains the VisionWorks wrapper code for execution of
 * stereo matching algorithms.
 * @author Zach LaCelle (zlacelle@mitre.org)
 */

//For parameters class
#include "visionworks_image_proc/visionworks_interface.h"

#include <VX/vx.h>
#include <OVX/UtilityOVX.hpp>

namespace stereo_visionworks
{
  /**
   * @class BaseStereo
   * @ingroup VisionWorks
   * @brief Virtual Base Class for Stereo Algorithm Implementation
   *
   * Defines a set of generic functions and members for re-implementation by
   * VisionWorks-based, graph-style stereo applications.
   */
  class BaseStereo
  {
  public:
    /**
     * @brief Default constructor for virtual base stereo class. Sets the graph pointer
     * to NULL awaiting initialization.
     */
    BaseStereo() : mainGraph_(nullptr) {}
    /**
     * @brief Pure virtual destructor for the base stereo class.
     */
    virtual ~BaseStereo() = 0;
    /**
     * @brief Run the current stereo pipeline.
     * 
     * Each stereo implementation is responsible for filling the execution graph, mainGraph_.
     * The execution beyond that point is identical: simply call vxProcessGraph() using the
     * NVXIO_SAFE_CALL wrapper.
     */
    void RunPipeline() { NVXIO_SAFE_CALL( vxProcessGraph(mainGraph_) ); }
    /**
     * @brief Print performance characteristics.
     *
     * Each implementation is responsible for printing out the performance of different graph
     * nodes when asked.
     */
    virtual void PrintPerfs() const = 0;
    
  protected:
    vx_graph mainGraph_;                ///< The graph which will be executed once populated
  };
  inline BaseStereo::~BaseStereo() {}

  /**
   * @class SGBM
   * @ingroup VisionWorks
   * @brief Performs the Semi-Global Block Matching algorithm on two calibrated images, 
   * from author K. Konolige.
   */
  class SGBM : public BaseStereo
  {
  public:
    /**
     * @brief Constructor for SGBM pipeline object
     * @param [out] _disparity The disparity image produced
     * @param [in] _context The ovxio Context object
     * @param [in] _params The SGBM parameters to use
     * @param [in] _left The left image, grayscale and rectified
     * @param [in] _right The right image, grayscale and rectified
     *
     * Sets up the pipeline for execution of SGBM on two rectified, grayscale images.
     *
     * These input images MUST be in VX_DF_IMAGE_U8 format (raw8 for OpenCV)
     * and MUST have width divisible by 4!
     *
     * The output disparity image MUST be a VX_DF_IMAGE_S16, and MUST have the
     * same size as the left input image!
     *
     * The format of the output disparity image is a Q11.4 fixed point format, with
     * values from [min_disparity, max_disparity).
     */
    SGBM(vx_image _disparity, vx_context _context, const stereo_visionworks::StereoParams& _params,
         vx_image _left, vx_image _right);
    /**
     * Destructor for SGBM
     */
    ~SGBM();

    /**
     * @brief Print performance characteristics of the SGBM class execution.
     */
    void PrintPerfs() const;
  private:    
    vx_node left_cvt_color_node_;         ///< OpenVX Graph node for converting color of the left image to grayscale, if required
    vx_node right_cvt_color_node_;        ///< OpenVX Graph node for converting color of the right image to grayscale, if required
    vx_node semiGlobalBlockMatchingNode_; ///< OpenVX Graph node for running the SGBM algorithm
    vx_node maskNode_;                    ///< OpenVX Graph node for masking SGBM output image
    vx_node convert_depth_node_;          ///< OpenVX Graph node for converting the resulting depth output, if required 

    bool greyColorConvertLastCycle_;      ///< Stores if we converted our images to grayscale last cycle, for debugging purposes
    bool depthConvertLastCycle_;          ///< Stores if we converted our depth last cycle, for debugging purposes
    bool imageMaskedLastCycle_;           ///< Stores if we masked the output image last cycle

    ///////////////////
    //Extra timing info
    double lastSetupLoopTime_s_;          ///< Time spent in the constructor of the stereo object, setting up
  };

  /**
   * @class BM
   * @ingroup visionworks
   * @brief Run the Stereo Block Matching algorithm.
   */
  class BM : public BaseStereo
  {
  public:
    /**
     * @brief Constructor for BM pipeline object
     * @param [out] _disparity The disparity image produced
     * @param [in] _context The ovxio Context object
     * @param [in] _params The SGBM parameters to use
     * @param [in] _left The left image, grayscale and rectified
     * @param [in] _right The right image, grayscale and rectified
     *
     * Sets up the pipeline for execution of BM on two rectified, grayscale images.
     *
     * These input images MUST be in VX_DF_IMAGE_U8 format (raw8 for OpenCV)
     * and MUST have width divisible by 4!
     *
     * The output disparity image MUST be a VX_DF_IMAGE_S16, and MUST have the
     * same size as the left input image!
     *
     * The format of the output disparity image is a Q11.4 fixed point format, with
     * values from [min_disparity, max_disparity).
     */
    BM(vx_image _disparity, vx_context _context, const stereo_visionworks::StereoParams& _params,
       vx_image _left, vx_image _right);
    ~BM();

    /**
     * @brief Print performance characteristics of the BM class execution.
     */
    void PrintPerfs() const;
  private:    
    vx_node left_cvt_color_node_;    ///< OpenVX Graph node for converting color of the left image to grayscale, if required
    vx_node right_cvt_color_node_;   ///< OpenVX Graph node for converting color of the right image to grayscale, if required
    vx_node blockMatchingNode_;      ///< OpenVX Graph node for running the BM algorithm
    vx_node speckleFilterNode_;      ///< OpenVX Graph node for speckle filtering
    vx_node convert_depth_node_;     ///< OpenVX Graph node for converting the resulting depth output, if required

    bool greyColorConvertLastCycle_; ///< Stores if we converted our images to grayscale last cycle, for debugging purposes
    bool depthConvertLastCycle_;     ///< Stores if we converted our depth last cycle, for debugging purposes
    bool speckleFilterLastCycle_;    ///< Stores if we ran the speckle filter node last cycle, for debugging purposes
    
    ///////////////////
    //Extra timing info
    double lastSetupLoopTime_s_;     ///< Time spent in the constructor of the stereo object, setting up
  };
} //end: namespace stereo_visionworks
