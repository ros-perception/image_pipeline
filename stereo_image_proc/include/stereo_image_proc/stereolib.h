/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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

//
// C version of stereo algorithm
// Basic column-oriented library
//

//
// input buffer sizes:
// do_prefilter_norm - (xim+64)*4
// do_stereo_y       - (yim + yim + yim*dlen)*2 + yim*dlen*xwin + 6*64
//                     ~ yim*dlen*(xwin+4)
// do_stereo_d       - (dlen*(yim-YKERN-ywin) + yim + yim*dlen)*2 + yim*dlen*xwin + 6*64
//                     ~ yim*dlen*(xwin+5)

#ifndef STEREOLIBH
#define STEREOLIBH

#ifdef __cplusplus
extern "C" {
#endif

#ifdef WIN32
  //#ifndef GCC
#include "pstdint.h"		// MSVC++ doesn't have stdint.h
#else
#include <stdint.h>
#endif
#include <math.h>
#include <string.h>
#include <stdlib.h>

// kernel size is fixed
#define KSIZE 7
#define XKERN KSIZE
#define YKERN KSIZE

// filtered disparity value
// NOTE: lowest valid disparity is 1
#define FILTERED 0

// prefilter

//#define do_prefilter do_prefilter_norm
//#define do_prefilter do_prefilter_fast
#define do_prefilter do_prefilter_xsobel

void
do_prefilter_norm(uint8_t *im,	// input image
	  uint8_t *ftim,	// feature image output
	  int xim, int yim,	// size of image
	  uint8_t ftzero,	// feature offset from zero
	  uint8_t *buf		// buffer storage
	  );

void
do_prefilter_fast(uint8_t *im,	// input image
	  uint8_t *ftim,	// feature image output
	  int xim, int yim,	// size of image
	  uint8_t ftzero,	// feature offset from zero
	  uint8_t *buf		// buffer storage
	  )
#ifdef GCC
//  __attribute__ ((force_align_arg_pointer)) // align to 16 bytes
#endif
;

void
do_prefilter_xsobel(uint8_t *im, // input image
	  uint8_t *ftim,	// feature image output
	  int xim, int yim,	// size of image
	  uint8_t ftzero,	// feature offset from zero
	  uint8_t *buf		// buffer storage
	  );




// algorithm requires buffers to be passed in

//#define do_stereo do_stereo_y
//#define do_stereo do_stereo_d
#define do_stereo do_stereo_d_fast
//#define do_stereo do_stereo_so
//#define do_stereo do_stereo_mw
//#define do_stereo do_stereo_dp

// inner loop over disparities
void
do_stereo_d(uint8_t *lim, uint8_t *rim, // input feature images
	  int16_t *disp,	// disparity output
	  int16_t *text,	// texture output
	  int xim, int yim,	// size of images
	  uint8_t ftzero,	// feature offset from zero
	  int xwin, int ywin,	// size of corr window, usually square
	  int dlen,		// size of disparity search, multiple of 8
	  int tfilter_thresh,	// texture filter threshold
	  int ufilter_thresh,	// uniqueness filter threshold, percent
	  uint8_t *buf		// buffer storage
	  );

// inner loop over columns
void
do_stereo_y(uint8_t *lim, uint8_t *rim, // input feature images
	  int16_t *disp,	// disparity output
	  int16_t *text,	// texture output
	  int xim, int yim,	// size of images
	  uint8_t ftzero,	// feature offset from zero
	  int xwin, int ywin,	// size of corr window, usually square
	  int dlen,		// size of disparity search, multiple of 8
	  int tfilter_thresh,	// texture filter threshold
	  int ufilter_thresh,	// uniqueness filter threshold, percent
	  uint8_t *buf		// buffer storage
	  );

void
do_stereo_d_fast(uint8_t *lim, uint8_t *rim, // input feature images
	  int16_t *disp,	// disparity output
	  int16_t *text,	// texture output
	  int xim, int yim,	// size of images
	  uint8_t ftzero,	// feature offset from zero
	  int xwin, int ywin,	// size of corr window, usually square
	  int dlen,		// size of disparity search, multiple of 8
	  int tfilter_thresh,	// texture filter threshold
	  int ufilter_thresh,	// uniqueness filter threshold, percent
	  uint8_t *buf		// buffer storage
	  );


//December 2008 
//Additions by Federico Tombari
//Stereo matching with regularization (Scanline Optimization)
void do_stereo_so(uint8_t *lim, uint8_t *rim, // input feature images
	  int16_t *disp,	// disparity output
	  int xim, int yim,	// size of images
	  uint8_t ftzero,	// feature offset from zero
	  int xwin, int ywin,	// size of corr window, usually square
	  int dlen,		// size of disparity search, multiple of 8
	  int pfilter_thresh,	// texture filter threshold
	  int ufilter_thresh,	// uniqueness filter threshold, percent
	  int smooth_thresh,    // smoothness threshold
	  int unique_c		//uniqueness check
	);

//Stereo matching using multiple windows
void do_stereo_mw(uint8_t *lim, uint8_t *rim, // input feature images
	  int16_t *disp,	// disparity output
	  int xim, int yim,	// size of images
	  uint8_t ftzero,	// feature offset from zero
	  int xwin, int ywin,	// size of corr window, usually square
	  int dlen,		// size of disparity search, multiple of 8
	  int pfilter_thresh,	// texture filter threshold
	  int ufilter_thresh,	// uniqueness filter threshold, percent
	  int unique_c		//uniqueness check
	  );

//Stereo matching using Dynamic Programming
void do_stereo_dp(uint8_t *lim, uint8_t *rim, // input feature images
	  int16_t *disp,	// disparity output
	  int xim, int yim,	// size of images
	  uint8_t ftzero,	// feature offset from zero
	  int xwin, int ywin,	// size of corr window, usually square
	  int dlen,		// size of disparity search, multiple of 8
	  int pfilter_thresh,	// texture filter threshold
	  int ufilter_thresh,	// uniqueness filter threshold, percent
	  int smooth_thresh,    // smoothness threshold
	  int unique_c		//uniqueness check
	);

//Stereo matching using Local Smoothness
void do_stereo_ls(uint8_t *lim, uint8_t *rim, // input feature images
	  int16_t *disp,	// disparity output
	  int xim, int yim,	// size of images
	  uint8_t ftzero,	// feature offset from zero
	  int xwin, int ywin,	// size of corr window, usually square
	  int dlen,		// size of disparity search, multiple of 8
	  int pfilter_thresh,	// texture filter threshold
	  int ufilter_thresh,	// uniqueness filter threshold, percent
	  int smooth_thresh,    // smoothness threshold
	  int unique_c		//uniqueness check
	);

//Stereo matching using NCC and block-matching approach
void do_stereo_ncc(uint8_t *lim, uint8_t *rim, // input feature images
	  int16_t *disp,	// disparity output
	  int xim, int yim,	// size of images
	  uint8_t ftzero,	// feature offset from zero
	  int xwin, int ywin,	// size of corr window, usually square
	  int dlen,		// size of disparity search, multiple of 8
	  int pfilter_thresh,	// texture filter threshold
	  int ufilter_thresh,	// uniqueness filter threshold, percent
	  int unique_c		//uniqueness check
	);
//
// sparse stereo
// corr window fixed at 15x15
// returns disparity value in 1/16 pixel, -1 on failure
// refpat is a 16x16 patch around the feature pixel
//   feature pixel is at coordinate 7,7 within the patch
// rim is the right gradient image
// x,y is the feature pixel coordinate
// other parameters as in do_stereo
//

int
do_stereo_sparse(uint8_t *refpat, uint8_t *rim, // input feature images
  	  int x, int y,	        // position of feature pixel
	  int xim, int yim,	// size of images
	  uint8_t ftzero,	// feature offset from zero
	  int dlen,		// size of disparity search, multiple of 8
	  int tfilter_thresh,	// texture filter threshold
	  int ufilter_thresh	// uniqueness filter threshold, percent
	  );


// SSE2 version
int
do_stereo_sparse_fast(uint8_t *refpat, uint8_t *rim, // input feature images
  	  int x, int y,	        // position of feature pixel
	  int xim, int yim,	// size of images
	  uint8_t ftzero,	// feature offset from zero
	  int dlen,		// size of disparity search, multiple of 8
	  int tfilter_thresh,	// texture filter threshold
	  int ufilter_thresh	// uniqueness filter threshold, percent
	  );


// speckle filter
// needs buffers: labels[image size], wbuf[image size], rtype[image size]
void do_speckle(int16_t *disp, int16_t badval, int width, int height, 
		int rdiff, int rcount, 
		uint32_t *labels, uint32_t *wbuf, uint8_t *rtype);


//
// bilinear interpolation structure
// embeds fixed-point interpolation coefficients
// should be 8 bytes in size

#define INTOFFSET 8		// 8-bit fractional offset of interpolation fixed-point

typedef struct 
{
  uint32_t addr;		// pixel addr, upper left
  uint8_t a1,a2,b1,b2;		// coefficients
} inttab_t;

void 
do_rectify_mono(uint8_t *dest,	// destination image
		uint8_t *src,	// source image
		int w,		// image width
		int h,		// image height
		inttab_t *rtab	// interpolation table
		);


void 
do_rectify_mono_fast(uint8_t *dest, // destination image
		uint8_t *src,	// source image
		int w,		// image width
		int h,		// image height
		inttab_t *rtab	// interpolation table
		);


#ifdef __cplusplus
}
#endif


#endif

	  
