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
 * @file visionworks_custom_kernels.h
 * @brief Values common across OpenVX custom kernels in this project.
 * @author Zach LaCelle (zlacelle@mitre.org)
 */

/**
 * @enum VisionWorksKernelEnum Enumerated values for OpenVX Custom Kernels
 * @brief Specifies values for OpenVX custom kernels
 *
 * OpenVX uniquely enumerates kernels based on a vendor ID and library ID. In the 
 * case where a vendor does not yet have an ID, VX_ID_DEFAULT is provided.
 * 
 * The function VX_KERNEL_BASE generates the ID based on the vendor and library.
 */
enum VisionWorksKernelEnum
{
    MASKING_LIB                  = 0x1,
    SPECKLE_FILTER_LIB           = 0x2,
    SOBEL_FILTER_LIB             = 0x3,
    KERNEL_IMAGE_MASKING         = VX_KERNEL_BASE( VX_ID_DEFAULT, MASKING_LIB ),
    KERNEL_IMAGE_SPECKLE_FILTER  = VX_KERNEL_BASE( VX_ID_DEFAULT, SPECKLE_FILTER_LIB ),
    KERNEL_IMAGE_SOBEL_FILTER    = VX_KERNEL_BASE( VX_ID_DEFAULT, SOBEL_FILTER_LIB )
};
