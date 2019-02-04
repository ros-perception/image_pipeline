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
#include "visionworks_image_proc/masking_node.h"
#include "visionworks_image_proc/visionworks_custom_kernels.h"

vx_node ImageMaskingNode( vx_graph _graph,
                          vx_image _input,
                          vx_uint32 _startX,
                          vx_uint32 _startY,
                          vx_uint32 _width,
                          vx_uint32 _height,
                          vx_uint8 _maskingValue )
{
  vx_node node = NULL;
  vx_context context = vxGetContext( ( vx_reference ) _graph );
  vx_kernel kernel = vxGetKernelByEnum( context, KERNEL_IMAGE_MASKING );
  if( vxGetStatus((vx_reference)kernel) == VX_SUCCESS )
  {
    node = vxCreateGenericNode( _graph, kernel );
    vxReleaseKernel(&kernel);

    // Scalar values all have to be of type vx_scalar.
    // Perform conversions here
    vx_status status;
    vx_scalar s_startX  = vxCreateScalar(context, VX_TYPE_UINT32, &_startX );
    status = vxGetStatus((vx_reference)(s_startX));
    if( status != VX_SUCCESS )
    {
      vxAddLogEntry((vx_reference)node, status, "ERROR: Failed to convert startX to vx_scalar.");
      return node;
    }
    vx_scalar s_startY  = vxCreateScalar(context, VX_TYPE_UINT32, &_startY );
    status = vxGetStatus((vx_reference)(s_startY));
    if( status != VX_SUCCESS )
    {
      vxAddLogEntry((vx_reference)node, status, "ERROR: Failed to convert startY to vx_scalar.");
      return node;
    }
    vx_scalar s_width  = vxCreateScalar(context, VX_TYPE_UINT32, &_width );
    status = vxGetStatus((vx_reference)(s_width));
    if( status != VX_SUCCESS )
    {
      vxAddLogEntry((vx_reference)node, status, "ERROR: Failed to convert width to vx_scalar.");
      return node;
    }
    vx_scalar s_height  = vxCreateScalar(context, VX_TYPE_UINT32, &_height );
    status = vxGetStatus((vx_reference)(s_height));
    if( status != VX_SUCCESS )
    {
      vxAddLogEntry((vx_reference)node, status, "ERROR: Failed to convert height to vx_scalar.");
      return node;
    }
    vx_scalar s_maskingValue  = vxCreateScalar(context, VX_TYPE_UINT8, &_maskingValue );
    status = vxGetStatus((vx_reference)(s_maskingValue));
    if( status != VX_SUCCESS )
    {
      vxAddLogEntry((vx_reference)node, status, "ERROR: Failed to convert maskingValue to vx_scalar.");
      return node;
    }
    
    vxSetParameterByIndex(node, 0, (vx_reference)_input );
    vxSetParameterByIndex(node, 1, (vx_reference)s_startX );
    vxSetParameterByIndex(node, 2, (vx_reference)s_startY );
    vxSetParameterByIndex(node, 3, (vx_reference)s_width );
    vxSetParameterByIndex(node, 4, (vx_reference)s_height );
    vxSetParameterByIndex(node, 5, (vx_reference)s_maskingValue );
  }
  return node;
}

vx_status VX_CALLBACK ImageMaskingValidator( vx_node _node,
                                             const vx_reference _parameters[], vx_uint32 _numArgs,
                                             vx_meta_format _metas[] )
{
  if( _numArgs != 6 )
  {
    vxAddLogEntry((vx_reference)_node, VX_ERROR_INVALID_PARAMETERS, "Invalid number of parameters to Image Masking node (should be 6)");
    return VX_FAILURE;
  }
  
  // parameter #0 -- input image of format VX_DF_IMAGE_U8 or VX_DF_IMAGE_S16
  vx_df_image format = VX_DF_IMAGE_VIRT;
  vxQueryImage( ( vx_image )_parameters[0], VX_IMAGE_FORMAT, &format, sizeof( format ) );
  if( format != VX_DF_IMAGE_U8 && format != VX_DF_IMAGE_S16 )
  {
    vxAddLogEntry((vx_reference)_node, VX_ERROR_INVALID_FORMAT, "ImageMaskingValidator: Invalid format on input image.");
    return VX_ERROR_INVALID_FORMAT;
  }

  // Parameter #1 - #5 -- startX, startY, width, height, maskingValue
  vx_enum type = VX_TYPE_INVALID;
  vxQueryScalar((vx_scalar)_parameters[1], VX_SCALAR_TYPE, &type, sizeof(type));
  if( type != VX_TYPE_UINT32 )
  {
    vxAddLogEntry((vx_reference)_node, VX_ERROR_INVALID_TYPE, "ImageMaskingValidator: Invalid type on input startX.");
    return VX_ERROR_INVALID_TYPE;
  }
  vxQueryScalar((vx_scalar)_parameters[2], VX_SCALAR_TYPE, &type, sizeof(type));
  if( type != VX_TYPE_UINT32 )
  {
    vxAddLogEntry((vx_reference)_node, VX_ERROR_INVALID_TYPE, "ImageMaskingValidator: Invalid type on input startY.");
    return VX_ERROR_INVALID_TYPE;
  }
  vxQueryScalar((vx_scalar)_parameters[3], VX_SCALAR_TYPE, &type, sizeof(type));
  if( type != VX_TYPE_UINT32 )
  {
    vxAddLogEntry((vx_reference)_node, VX_ERROR_INVALID_TYPE, "ImageMaskingValidator: Invalid type on input width.");
    return VX_ERROR_INVALID_TYPE;
  }
  vxQueryScalar((vx_scalar)_parameters[4], VX_SCALAR_TYPE, &type, sizeof(type));
  if( type != VX_TYPE_UINT32 )
  {
    vxAddLogEntry((vx_reference)_node, VX_ERROR_INVALID_TYPE, "ImageMaskingValidator: Invalid type on input height.");
    return VX_ERROR_INVALID_TYPE;
  }
  vxQueryScalar((vx_scalar)_parameters[5], VX_SCALAR_TYPE, &type, sizeof(type));
  if( type != VX_TYPE_UINT8 )
  {
    vxAddLogEntry((vx_reference)_node, VX_ERROR_INVALID_TYPE, "ImageMaskingValidator: Invalid type on input maskingValue.");
    return VX_ERROR_INVALID_TYPE;
  }
  
  return VX_SUCCESS;
}

vx_status VX_CALLBACK ImageMaskingHostSideFunction( vx_node _node, const vx_reference * _refs, vx_uint32 _numArgs )
{
  if( _numArgs != 6 )
  {
    vxAddLogEntry((vx_reference)_node, VX_ERROR_INVALID_PARAMETERS, "Process function has invalid number of parameters (should be 6).");
    return VX_FAILURE;
  }
  
  vx_image image = (vx_image)(_refs[0]);

  vx_scalar tempScalar = (vx_scalar)_refs[5];
  vx_uint8 maskedOffPixelValue;
  vxCopyScalar(tempScalar, &maskedOffPixelValue, VX_READ_ONLY, VX_MEMORY_TYPE_HOST);
  void* basePtr = NULL;
  vx_imagepatch_addressing_t addr;
  vx_map_id mapID;
  vx_uint32 plane = 0;
  vx_rectangle_t maskRect;
  tempScalar = (vx_scalar)_refs[1];
  vxCopyScalar(tempScalar, &(maskRect.start_x), VX_READ_ONLY, VX_MEMORY_TYPE_HOST);
  tempScalar = (vx_scalar)_refs[2];
  vxCopyScalar(tempScalar, &(maskRect.start_y), VX_READ_ONLY, VX_MEMORY_TYPE_HOST);
  tempScalar = (vx_scalar)_refs[3];
  vx_uint32 tempInt;
  vxCopyScalar(tempScalar, &(tempInt), VX_READ_ONLY, VX_MEMORY_TYPE_HOST);
  maskRect.end_x = maskRect.start_x + tempInt;
  tempScalar = (vx_scalar)_refs[4];
  vxCopyScalar(tempScalar, &(tempInt), VX_READ_ONLY, VX_MEMORY_TYPE_HOST);
  maskRect.end_y = maskRect.start_y + tempInt;
  vxMapImagePatch(image, &maskRect, plane,
                  &mapID, &addr, &basePtr,
                  VX_WRITE_ONLY, VX_MEMORY_TYPE_HOST, 0);
  for( vx_uint32 y = 0; y < addr.dim_y; y += addr.step_y )
  {
    for( vx_uint32 x = 0; x < addr.dim_x; x += addr.step_x )
    {
      vx_int16* ptr2 = static_cast<vx_int16*>(vxFormatImagePatchAddress2d(basePtr, x, y, &addr));
      *ptr2 = static_cast<vx_int16>(maskedOffPixelValue);
    }
  } //end: for( ... )
  vxUnmapImagePatch(image, mapID);

  return VX_SUCCESS;
}

vx_status RegisterImageMaskingKernel( vx_context _context )
{
  vx_status status = VX_SUCCESS;
  vx_kernel kernel = vxAddUserKernel( _context,
                                      "app.userkernels.imagemasking",
                                      KERNEL_IMAGE_MASKING,
                                      ImageMaskingHostSideFunction,
                                      6,   // numParams
                                      ImageMaskingValidator,
                                      NULL,
                                      NULL );
  status = vxGetStatus((vx_reference)kernel);
  if( status != VX_SUCCESS )
  {
    vxAddLogEntry((vx_reference)_context, status, "Failed to create Image Masking Kernel");
    return status;
  }
  
  status |= vxAddParameterToKernel( kernel, 0, VX_BIDIRECTIONAL,  VX_TYPE_IMAGE,  VX_PARAMETER_STATE_REQUIRED );
  if( status != VX_SUCCESS )
  {
    vxAddLogEntry((vx_reference)_context, status, "Failed to initialize Masking parameters");
    return VX_FAILURE;
  }
  status |= vxAddParameterToKernel( kernel, 1, VX_INPUT,  VX_TYPE_SCALAR,  VX_PARAMETER_STATE_REQUIRED );
  if( status != VX_SUCCESS )
  {
    vxAddLogEntry((vx_reference)_context, status, "Failed to initialize Masking parameters");
    return VX_FAILURE;
  }
  status |= vxAddParameterToKernel( kernel, 2, VX_INPUT,  VX_TYPE_SCALAR,  VX_PARAMETER_STATE_REQUIRED );
  if( status != VX_SUCCESS )
  {
    vxAddLogEntry((vx_reference)_context, status, "Failed to initialize Masking parameters");
    return VX_FAILURE;
  }
  status |= vxAddParameterToKernel( kernel, 3, VX_INPUT,  VX_TYPE_SCALAR,  VX_PARAMETER_STATE_REQUIRED );
  if( status != VX_SUCCESS )
  {
    vxAddLogEntry((vx_reference)_context, status, "Failed to initialize Masking parameters");
    return VX_FAILURE;
  }
  status |= vxAddParameterToKernel( kernel, 4, VX_INPUT,  VX_TYPE_SCALAR,  VX_PARAMETER_STATE_REQUIRED );
  if( status != VX_SUCCESS )
  {
    vxAddLogEntry((vx_reference)_context, status, "Failed to initialize Masking parameters");
    return VX_FAILURE;
  }
  status |= vxAddParameterToKernel( kernel, 5, VX_INPUT,  VX_TYPE_SCALAR,  VX_PARAMETER_STATE_REQUIRED );
  if( status != VX_SUCCESS )
  {
    vxAddLogEntry((vx_reference)_context, status, "Failed to initialize Masking parameters");
    return VX_FAILURE;
  }
  
  status = vxFinalizeKernel( kernel );
  if( status != VX_SUCCESS )
  {
    vxReleaseKernel(&kernel);
    vxAddLogEntry((vx_reference)_context, status, "Failed to finalize Masking Kernel");
    return VX_FAILURE;
  }
    
  return status;
}
