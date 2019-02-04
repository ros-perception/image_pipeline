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
#include "visionworks_image_proc/sobel_filter_node.h"
#include "visionworks_image_proc/visionworks_custom_kernels.h"

#include <opencv2/calib3d.hpp>
//copied from opencv2/core/internal.hpp for CV_CAST_8U
#define  CV_CAST_8U(t)  (uchar)(!((t) & ~255) ? (t) : (t) > 0 ? 255 : 0)

vx_node SobelFilterNode( vx_graph _graph,
                         vx_image _leftIn,
                         vx_image _rightIn,
                         vx_image _leftOut,
                         vx_image _rightOut,
                         vx_uint32 _prefilterCap )
{
  vx_node node = NULL;
  vx_context context = vxGetContext( ( vx_reference ) _graph );
  vx_kernel kernel = vxGetKernelByEnum( context, KERNEL_IMAGE_SOBEL_FILTER );
  vx_status kernelStatus = vxGetStatus((vx_reference)kernel);
  if( kernelStatus == VX_SUCCESS )
  {
    node = vxCreateGenericNode( _graph, kernel );
    vxReleaseKernel(&kernel);

    // Scalar values all have to be of type vx_scalar.
    // Perform conversions here
    vx_status status;
    vx_scalar s_prefilterCap  = vxCreateScalar(context, VX_TYPE_UINT32, &_prefilterCap );
    status = vxGetStatus((vx_reference)(s_prefilterCap));
    if( status != VX_SUCCESS )
    {
      vxAddLogEntry((vx_reference)node, status, "ERROR: Failed to convert prefilterCap to vx_scalar.");
      return node;
    }
    
    vxSetParameterByIndex(node, 0, (vx_reference)_leftIn );
    vxSetParameterByIndex(node, 1, (vx_reference)_rightIn );
    vxSetParameterByIndex(node, 2, (vx_reference)_leftOut );
    vxSetParameterByIndex(node, 3, (vx_reference)_rightOut );
    vxSetParameterByIndex(node, 4, (vx_reference)s_prefilterCap );
  } //end: if( vxGetStatus((vx_reference)kernel) == VX_SUCCESS )
  else
  {
    vxAddLogEntry((vx_reference)kernel, kernelStatus, "Could not get kernel by enum in SobelFilterNode.");
  } //end: else
  return node;
}

vx_status VX_CALLBACK SobelFilterValidator( vx_node _node,
                                            const vx_reference _parameters[], vx_uint32 _numArgs,
                                            vx_meta_format _metas[] )
{
  if( _numArgs != 5 )
  {
    vxAddLogEntry((vx_reference)_node, VX_ERROR_INVALID_PARAMETERS, "Invalid number of parameters to Sobel Filtering node (should be 5)");
    return VX_FAILURE;
  }
  
  // parameter #0 - #3 -- left input image of format VX_DF_IMAGE_U8
  vx_df_image format = VX_DF_IMAGE_VIRT;
  vxQueryImage( ( vx_image )_parameters[0], VX_IMAGE_FORMAT, &format, sizeof( format ) );
  if( format != VX_DF_IMAGE_U8 )
  {
    vxAddLogEntry((vx_reference)_node, VX_ERROR_INVALID_FORMAT, "SobelFilterValidator: Invalid format on left input image (must be VX_DF_IMAGE_U8.");
    return VX_ERROR_INVALID_FORMAT;
  }
  format = VX_DF_IMAGE_VIRT;
  vxQueryImage( ( vx_image )_parameters[1], VX_IMAGE_FORMAT, &format, sizeof( format ) );
  if( format != VX_DF_IMAGE_U8 )
  {
    vxAddLogEntry((vx_reference)_node, VX_ERROR_INVALID_FORMAT, "SobelFilterValidator: Invalid format on right input image (must be VX_DF_IMAGE_U8.");
    return VX_ERROR_INVALID_FORMAT;
  }
  format = VX_DF_IMAGE_VIRT;
  vxQueryImage( ( vx_image )_parameters[2], VX_IMAGE_FORMAT, &format, sizeof( format ) );
  if( format != VX_DF_IMAGE_U8 )
  {
    vxAddLogEntry((vx_reference)_node, VX_ERROR_INVALID_FORMAT, "SobelFilterValidator: Invalid format on left output image (must be VX_DF_IMAGE_U8.");
    return VX_ERROR_INVALID_FORMAT;
  }
  format = VX_DF_IMAGE_VIRT;
  vxQueryImage( ( vx_image )_parameters[3], VX_IMAGE_FORMAT, &format, sizeof( format ) );
  if( format != VX_DF_IMAGE_U8 )
  {
    vxAddLogEntry((vx_reference)_node, VX_ERROR_INVALID_FORMAT, "SobelFilterValidator: Invalid format on right output image (must be VX_DF_IMAGE_U8.");
    return VX_ERROR_INVALID_FORMAT;
  }
  
  // Parameter #4 -- prefilterCap
  vx_enum type = VX_TYPE_INVALID;
  vxQueryScalar((vx_scalar)_parameters[4], VX_SCALAR_TYPE, &type, sizeof(type));
  if( type != VX_TYPE_UINT32 )
  {
    vxAddLogEntry((vx_reference)_node, VX_ERROR_INVALID_TYPE, "SobelFilterValidator: Invalid type on input prefilterCap.");
    return VX_ERROR_INVALID_TYPE;
  }

  return VX_SUCCESS;
}

vx_status VX_CALLBACK SobelFilterHostSideFunction( vx_node _node, const vx_reference * _refs, vx_uint32 _numArgs )
{
  if( _numArgs != 5 )
  {
    vxAddLogEntry((vx_reference)_node, VX_ERROR_INVALID_PARAMETERS, "Process function has invalid number of parameters (should be 5).");
    return VX_FAILURE;
  }
  
  vx_image leftIn = (vx_image)(_refs[0]);
  vx_image rightIn = (vx_image)(_refs[1]);
  vx_image leftOut = (vx_image)(_refs[2]);
  vx_image rightOut = (vx_image)(_refs[3]);

  vx_uint32 prefilterCap;
  vx_scalar tempScalar;
  tempScalar = (vx_scalar)_refs[4];
  vxCopyScalar(tempScalar, &(prefilterCap), VX_READ_ONLY, VX_MEMORY_TYPE_HOST);

  // Map VX Image to OpenCV Mat, no memory copy
  nvx_cv::VXImageToCVMatMapper leftImageInMapper(leftIn);
  nvx_cv::VXImageToCVMatMapper rightImageInMapper(rightIn);
  nvx_cv::VXImageToCVMatMapper leftImageOutMapper(leftOut);
  nvx_cv::VXImageToCVMatMapper rightImageOutMapper(rightOut);
  cv::Mat cvLeftIn = leftImageInMapper.getMat();
  cv::Mat cvRightIn = rightImageInMapper.getMat();
  cv::Mat cvLeftOut = leftImageOutMapper.getMat();
  cv::Mat cvRightOut = rightImageOutMapper.getMat();

  //This is a copy of the code from OpenCV sterebm.cpp.
  //Unfortunately this isn't broken out into a callable function.
  OpenCVPrefilterXSobel(cvLeftIn, cvLeftOut, prefilterCap);
  OpenCVPrefilterXSobel(cvRightIn, cvRightOut, prefilterCap);
  
  return VX_SUCCESS;
}

vx_status RegisterSobelFilterKernel( vx_context _context )
{
  vx_status status = VX_SUCCESS;
  vx_kernel kernel = vxAddUserKernel( _context,
                                      "app.userkernels.sobelfilter",
                                      KERNEL_IMAGE_SOBEL_FILTER,
                                      SobelFilterHostSideFunction,
                                      5,   // numParams
                                      SobelFilterValidator,
                                      NULL,
                                      NULL );
  status = vxGetStatus((vx_reference)kernel);
  if( status != VX_SUCCESS )
  {
    vxAddLogEntry((vx_reference)_context, status, "Failed to create Sobel Filter Kernel");
    return status;
  }
  
  status |= vxAddParameterToKernel( kernel, 0, VX_INPUT,  VX_TYPE_IMAGE,  VX_PARAMETER_STATE_REQUIRED );
  if( status != VX_SUCCESS )
  {
    vxAddLogEntry((vx_reference)_context, status, "Failed to initialize Sobel Filter parameters");
    return VX_FAILURE;
  }
  status |= vxAddParameterToKernel( kernel, 1, VX_INPUT,  VX_TYPE_IMAGE,  VX_PARAMETER_STATE_REQUIRED );
  if( status != VX_SUCCESS )
  {
    vxAddLogEntry((vx_reference)_context, status, "Failed to initialize Sobel Filter parameters");
    return VX_FAILURE;
  }
    status |= vxAddParameterToKernel( kernel, 2, VX_OUTPUT,  VX_TYPE_IMAGE,  VX_PARAMETER_STATE_REQUIRED );
  if( status != VX_SUCCESS )
  {
    vxAddLogEntry((vx_reference)_context, status, "Failed to initialize Sobel Filter parameters");
    return VX_FAILURE;
  }
    status |= vxAddParameterToKernel( kernel, 3, VX_OUTPUT,  VX_TYPE_IMAGE,  VX_PARAMETER_STATE_REQUIRED );
  if( status != VX_SUCCESS )
  {
    vxAddLogEntry((vx_reference)_context, status, "Failed to initialize Sobel Filter parameters");
    return VX_FAILURE;
  }
  status |= vxAddParameterToKernel( kernel, 4, VX_INPUT,  VX_TYPE_SCALAR,  VX_PARAMETER_STATE_REQUIRED );
  if( status != VX_SUCCESS )
  {
    vxAddLogEntry((vx_reference)_context, status, "Failed to initialize Sobel Filter parameters");
    return VX_FAILURE;
  }
  
  status = vxFinalizeKernel( kernel );
  if( status != VX_SUCCESS )
  {
    vxReleaseKernel(&kernel);
    vxAddLogEntry((vx_reference)_context, status, "Failed to finalize Sobel Filter Kernel");
    return VX_FAILURE;
  }
    
  return status;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// IMPORTANT: License below applies to PrefilterXSobel function, copied from
// OpenCV's stereobm.cpp source code.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                        Intel License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000, Intel Corporation, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of Intel Corporation may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
// Contributed to OpenCV by Kurt Konolige
void OpenCVPrefilterXSobel(cv::Mat _src, cv::Mat _dst, int _ftzero)
{
  int x = 0;
  int y = 0;
  const int OFS = 256*4;
  const int TABSZ = OFS*2 + 256;
  uchar tab[TABSZ];
  cv::Size size = _src.size();

  for( x = 0; x < TABSZ; ++x )
  {
    if( x - OFS < -_ftzero )
    {
      tab[x] = (uchar)(0);
    }
    else if( x - OFS > _ftzero )
    {
      tab[x] = (uchar)(_ftzero * 2);
    }
    else
    {
      tab[x] = (uchar)(x - OFS + _ftzero);
    }
  } //end: for( x = 0; x < TABSZ; ++x )
  uchar val0 = tab[0 + OFS];

#if CV_SSE2
  volatile bool useSIMD = cv::checkHardwareSupport(CV_CPU_SSE2);
#endif

  for( y = 0; y < size.height-1; y += 2 )
  {
    //The ternary operators let us assign to const ptr. Comments explaining each.
    const uchar* srow1 = _src.ptr<uchar>(y);
    //if y > 0 then srow0 = srow1 - _src.step, else if size.height > 1 then srow0 = srow1 + _src.step, else srow0 = srow1
    const uchar* srow0 = y > 0 ? srow1 - _src.step : size.height > 1 ? srow1 + _src.step : srow1;
    //if y < (size.height - 1) then srow2 = srow1 + _src.step, else if size.height > 1 then srow2 = srow1 - _src.step, else srow2 = srow1
    const uchar* srow2 = y < size.height-1 ? srow1 + _src.step : size.height > 1 ? srow1 - _src.step : srow1;
    //if y < (size.height - 2) then srow3 = srow1 + (_src.step * 2), else srow3 = srow1
    const uchar* srow3 = y < size.height-2 ? srow1 + _src.step*2 : srow1;
    uchar* dptr0 = _dst.ptr<uchar>(y);
    uchar* dptr1 = dptr0 + _dst.step;

    dptr0[0] = dptr0[size.width-1] = dptr1[0] = dptr1[size.width-1] = val0;
    x = 1;

#if CV_SSE2
    if( useSIMD )
    {
      __m128i z = _mm_setzero_si128(), ftz = _mm_set1_epi16((short)_ftzero),
        ftz2 = _mm_set1_epi8(CV_CAST_8U(_ftzero*2));
      for( ; x <= size.width-9; x += 8 )
      {
        __m128i c0 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i*)(srow0 + x - 1)), z);
        __m128i c1 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i*)(srow1 + x - 1)), z);
        __m128i d0 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i*)(srow0 + x + 1)), z);
        __m128i d1 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i*)(srow1 + x + 1)), z);

        d0 = _mm_sub_epi16(d0, c0);
        d1 = _mm_sub_epi16(d1, c1);

        __m128i c2 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i*)(srow2 + x - 1)), z);
        __m128i c3 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i*)(srow3 + x - 1)), z);
        __m128i d2 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i*)(srow2 + x + 1)), z);
        __m128i d3 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i*)(srow3 + x + 1)), z);

        d2 = _mm_sub_epi16(d2, c2);
        d3 = _mm_sub_epi16(d3, c3);

        __m128i v0 = _mm_add_epi16(d0, _mm_add_epi16(d2, _mm_add_epi16(d1, d1)));
        __m128i v1 = _mm_add_epi16(d1, _mm_add_epi16(d3, _mm_add_epi16(d2, d2)));
        v0 = _mm_packus_epi16(_mm_add_epi16(v0, ftz), _mm_add_epi16(v1, ftz));
        v0 = _mm_min_epu8(v0, ftz2);

        _mm_storel_epi64((__m128i*)(dptr0 + x), v0);
        _mm_storel_epi64((__m128i*)(dptr1 + x), _mm_unpackhi_epi64(v0, v0));
      }
    }
#endif
#if CV_NEON
    int16x8_t ftz = vdupq_n_s16 ((short) _ftzero);
    uint8x8_t ftz2 = vdup_n_u8 (cv::saturate_cast<uchar>(_ftzero*2));

    for(; x <=size.width-9; x += 8 )
    {
      uint8x8_t c0 = vld1_u8 (srow0 + x - 1);
      uint8x8_t c1 = vld1_u8 (srow1 + x - 1);
      uint8x8_t d0 = vld1_u8 (srow0 + x + 1);
      uint8x8_t d1 = vld1_u8 (srow1 + x + 1);

      int16x8_t t0 = vreinterpretq_s16_u16 (vsubl_u8 (d0, c0));
      int16x8_t t1 = vreinterpretq_s16_u16 (vsubl_u8 (d1, c1));

      uint8x8_t c2 = vld1_u8 (srow2 + x - 1);
      uint8x8_t c3 = vld1_u8 (srow3 + x - 1);
      uint8x8_t d2 = vld1_u8 (srow2 + x + 1);
      uint8x8_t d3 = vld1_u8 (srow3 + x + 1);

      int16x8_t t2 = vreinterpretq_s16_u16 (vsubl_u8 (d2, c2));
      int16x8_t t3 = vreinterpretq_s16_u16 (vsubl_u8 (d3, c3));

      int16x8_t v0 = vaddq_s16 (vaddq_s16 (t2, t0), vaddq_s16 (t1, t1));
      int16x8_t v1 = vaddq_s16 (vaddq_s16 (t3, t1), vaddq_s16 (t2, t2));


      uint8x8_t v0_u8 = vqmovun_s16 (vaddq_s16 (v0, ftz));
      uint8x8_t v1_u8 = vqmovun_s16 (vaddq_s16 (v1, ftz));
      v0_u8 =  vmin_u8 (v0_u8, ftz2);
      v1_u8 =  vmin_u8 (v1_u8, ftz2);
      vqmovun_s16 (vaddq_s16 (v1, ftz));

      vst1_u8 (dptr0 + x, v0_u8);
      vst1_u8 (dptr1 + x, v1_u8);
    }
#endif

    for( ; x < size.width-1; x++ )
    {
      int d0 = srow0[x+1] - srow0[x-1], d1 = srow1[x+1] - srow1[x-1],
        d2 = srow2[x+1] - srow2[x-1], d3 = srow3[x+1] - srow3[x-1];
      int v0 = tab[d0 + d1*2 + d2 + OFS];
      int v1 = tab[d1 + d2*2 + d3 + OFS];
      dptr0[x] = (uchar)v0;
      dptr1[x] = (uchar)v1;
    }
  }

#if CV_NEON
  uint8x16_t val0_16 = vdupq_n_u8 (val0);
#endif

  for( ; y < size.height; y++ )
  {
    uchar* dptr = _dst.ptr<uchar>(y);
    x = 0;
#if CV_NEON
    for(; x <= size.width-16; x+=16 )
      vst1q_u8 (dptr + x, val0_16);
#endif
    for(; x < size.width; x++ )
      dptr[x] = val0;
  }
} //end: OpenCVPrefilterXSobel()
///////////////////////////////////////////////////////////////////////////////////////////////////
