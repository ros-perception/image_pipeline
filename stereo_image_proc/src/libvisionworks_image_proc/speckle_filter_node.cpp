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

#include <NVX/nvx_opencv_interop.hpp>
#include <VX/vx.h>
#include <OVX/UtilityOVX.hpp>
#include "visionworks_image_proc/speckle_filter_node.h"
#include "visionworks_image_proc/visionworks_custom_kernels.h"

#include <opencv2/calib3d.hpp>
//#include <opencv2/gpu/gpu.hpp>

vx_node SpeckleFilterNode( vx_graph _graph,
                           vx_image _input,
                           vx_uint32 _filteredValue,
                           vx_uint32 _speckleSize,
                           vx_uint32 _speckleRange)
{
  vx_node node = NULL;
  vx_context context = vxGetContext( ( vx_reference ) _graph );
  vx_kernel kernel = vxGetKernelByEnum( context, KERNEL_IMAGE_SPECKLE_FILTER );
  vx_status kernelStatus = vxGetStatus((vx_reference)kernel);
  if( kernelStatus == VX_SUCCESS )
  {
    node = vxCreateGenericNode( _graph, kernel );
    vxReleaseKernel(&kernel);

    // Scalar values all have to be of type vx_scalar.
    // Perform conversions here
    vx_status status;
    vx_scalar s_filteredValue  = vxCreateScalar(context, VX_TYPE_UINT32, &_filteredValue );
    status = vxGetStatus((vx_reference)(s_filteredValue));
    if( status != VX_SUCCESS )
    {
      vxAddLogEntry((vx_reference)node, status, "ERROR: Failed to convert filteredValue to vx_scalar.");
      return node;
    }
    vx_scalar s_speckleSize  = vxCreateScalar(context, VX_TYPE_UINT32, &_speckleSize );
    status = vxGetStatus((vx_reference)(s_speckleSize));
    if( status != VX_SUCCESS )
    {
      vxAddLogEntry((vx_reference)node, status, "ERROR: Failed to convert speckleSize to vx_scalar.");
      return node;
    }
    vx_scalar s_speckleRange  = vxCreateScalar(context, VX_TYPE_UINT32, &_speckleRange );
    status = vxGetStatus((vx_reference)(s_speckleRange));
    if( status != VX_SUCCESS )
    {
      vxAddLogEntry((vx_reference)node, status, "ERROR: Failed to convert speckleRange to vx_scalar.");
      return node;
    }
    
    vxSetParameterByIndex(node, 0, (vx_reference)_input );
    vxSetParameterByIndex(node, 1, (vx_reference)s_filteredValue );
    vxSetParameterByIndex(node, 2, (vx_reference)s_speckleSize );
    vxSetParameterByIndex(node, 3, (vx_reference)s_speckleRange );
  } //end: if( vxGetStatus((vx_reference)kernel) == VX_SUCCESS )
  else
  {
    vxAddLogEntry((vx_reference)kernel, kernelStatus, "Could not get kernel by enum in SpeckleFilterNode.");
  } //end: else
  return node;
}

vx_status VX_CALLBACK SpeckleFilterValidator( vx_node _node,
                                              const vx_reference _parameters[], vx_uint32 _numArgs,
                                              vx_meta_format _metas[] )
{
  if( _numArgs != 4 )
  {
    vxAddLogEntry((vx_reference)_node, VX_ERROR_INVALID_PARAMETERS, "Invalid number of parameters to Speckle Filtering node (should be 4)");
    return VX_FAILURE;
  }
  
  // parameter #0 -- input image of format VX_DF_IMAGE_U8
  vx_df_image format = VX_DF_IMAGE_VIRT;
  vxQueryImage( ( vx_image )_parameters[0], VX_IMAGE_FORMAT, &format, sizeof( format ) );
  if( format != VX_DF_IMAGE_U8 )
  {
    vxAddLogEntry((vx_reference)_node, VX_ERROR_INVALID_FORMAT, "SpeckleFilterValidator: Invalid format on input image (must be VX_DF_IMAGE_U8.");
    return VX_ERROR_INVALID_FORMAT;
  }

  // Parameter #1 - #5 -- filteredValue, speckleSize, speckleRange
  vx_enum type = VX_TYPE_INVALID;
  vxQueryScalar((vx_scalar)_parameters[1], VX_SCALAR_TYPE, &type, sizeof(type));
  if( type != VX_TYPE_UINT32 )
  {
    vxAddLogEntry((vx_reference)_node, VX_ERROR_INVALID_TYPE, "SpeckleFilterValidator: Invalid type on input filteredValue.");
    return VX_ERROR_INVALID_TYPE;
  }
  vxQueryScalar((vx_scalar)_parameters[2], VX_SCALAR_TYPE, &type, sizeof(type));
  if( type != VX_TYPE_UINT32 )
  {
    vxAddLogEntry((vx_reference)_node, VX_ERROR_INVALID_TYPE, "SpeckleFilterValidator: Invalid type on input speckleSize.");
    return VX_ERROR_INVALID_TYPE;
  }
  vxQueryScalar((vx_scalar)_parameters[3], VX_SCALAR_TYPE, &type, sizeof(type));
  if( type != VX_TYPE_UINT32 )
  {
    vxAddLogEntry((vx_reference)_node, VX_ERROR_INVALID_TYPE, "SpeckleFilterValidator: Invalid type on input speckleRange.");
    return VX_ERROR_INVALID_TYPE;
  }

  return VX_SUCCESS;
}

vx_status VX_CALLBACK SpeckleFilterHostSideFunction( vx_node _node, const vx_reference * _refs, vx_uint32 _numArgs )
{
  if( _numArgs != 4 )
  {
    vxAddLogEntry((vx_reference)_node, VX_ERROR_INVALID_PARAMETERS, "Process function has invalid number of parameters (should be 4).");
    return VX_FAILURE;
  }
  
  vx_image image = (vx_image)(_refs[0]);

  vx_uint32 filteredValue, speckleSize, speckleRange;
  vx_scalar tempScalar;
  tempScalar = (vx_scalar)_refs[1];
  vxCopyScalar(tempScalar, &(filteredValue), VX_READ_ONLY, VX_MEMORY_TYPE_HOST);
  tempScalar = (vx_scalar)_refs[2];
  vxCopyScalar(tempScalar, &(speckleSize), VX_READ_ONLY, VX_MEMORY_TYPE_HOST);
  tempScalar = (vx_scalar)_refs[3];
  vxCopyScalar(tempScalar, &(speckleRange), VX_READ_ONLY, VX_MEMORY_TYPE_HOST);
  vx_uint32 imageWidth;
  vx_uint32 imageHeight;
  vxQueryImage(image, VX_IMAGE_ATTRIBUTE_WIDTH, &(imageWidth), sizeof(imageWidth));
  vxQueryImage(image, VX_IMAGE_ATTRIBUTE_HEIGHT, &(imageHeight), sizeof(imageHeight));
  
  // Map VX Image to OpenCV Mat, no memory copy

  vx_uint32 plane_index = 0;
  vx_rectangle_t rect;
  rect.start_x = 0;
  rect.start_y = 0;
  rect.end_x = imageWidth;
  rect.end_y = imageHeight;

  //GPU
  // Note: This is not used in favor of the cv::Mat (CPU Mat), because as of writing
  // this code NVIDIA JetPack 3.2 for the TX2 didn't have OpenCV GPU support.
  //nvx_cv::VXImageToCVMatMapper imageMapper(image, plane_index, &rect, VX_READ_AND_WRITE, NVX_MEMORY_TYPE_CUDA);
  //cv::gpu::GpuMat imageMat = imageMapper.getGpuMat();
  //cv::gpu::GpuMat scratchBuf;
  
  //CPU
  nvx_cv::VXImageToCVMatMapper imageMapper(image, plane_index, &rect, VX_READ_AND_WRITE, VX_MEMORY_TYPE_HOST);
  cv::Mat imageMat = imageMapper.getMat();
  cv::Mat scratchBuf;

  cv::filterSpeckles(imageMat, filteredValue, speckleSize, speckleRange, scratchBuf);
  
  return VX_SUCCESS;
}

vx_status RegisterSpeckleFilterKernel( vx_context _context )
{
  vx_status status = VX_SUCCESS;
  vx_kernel kernel = vxAddUserKernel( _context,
                                      "app.userkernels.specklefilter",
                                      KERNEL_IMAGE_SPECKLE_FILTER,
                                      SpeckleFilterHostSideFunction,
                                      4,   // numParams
                                      SpeckleFilterValidator,
                                      NULL,
                                      NULL );
  status = vxGetStatus((vx_reference)kernel);
  if( status != VX_SUCCESS )
  {
    vxAddLogEntry((vx_reference)_context, status, "Failed to create Speckle Filter Kernel");
    return status;
  }
  
  status |= vxAddParameterToKernel( kernel, 0, VX_BIDIRECTIONAL,  VX_TYPE_IMAGE,  VX_PARAMETER_STATE_REQUIRED );
  if( status != VX_SUCCESS )
  {
    vxAddLogEntry((vx_reference)_context, status, "Failed to initialize Speckle Filter parameters");
    return VX_FAILURE;
  }
  status |= vxAddParameterToKernel( kernel, 1, VX_INPUT,  VX_TYPE_SCALAR,  VX_PARAMETER_STATE_REQUIRED );
  if( status != VX_SUCCESS )
  {
    vxAddLogEntry((vx_reference)_context, status, "Failed to initialize Speckle Filter parameters");
    return VX_FAILURE;
  }
  status |= vxAddParameterToKernel( kernel, 2, VX_INPUT,  VX_TYPE_SCALAR,  VX_PARAMETER_STATE_REQUIRED );
  if( status != VX_SUCCESS )
  {
    vxAddLogEntry((vx_reference)_context, status, "Failed to initialize Speckle Filter parameters");
    return VX_FAILURE;
  }
  status |= vxAddParameterToKernel( kernel, 3, VX_INPUT,  VX_TYPE_SCALAR,  VX_PARAMETER_STATE_REQUIRED );
  if( status != VX_SUCCESS )
  {
    vxAddLogEntry((vx_reference)_context, status, "Failed to initialize Speckle Filter parameters");
    return VX_FAILURE;
  }
  
  status = vxFinalizeKernel( kernel );
  if( status != VX_SUCCESS )
  {
    vxReleaseKernel(&kernel);
    vxAddLogEntry((vx_reference)_context, status, "Failed to finalize Speckle Filter Kernel");
    return VX_FAILURE;
  }
    
  return status;
}
