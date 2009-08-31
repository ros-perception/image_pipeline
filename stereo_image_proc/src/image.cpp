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
// image.cpp
// classes for monocular and stereo image
//

#include "image.h"


#include <sstream>
#include <iostream>

#define PRINTF(a...) printf(a)

using namespace cam;

// image class fns

ImageData::ImageData()
{
  imWidth = imHeight = 0;

  // buffers
  im = NULL;
  imColor = NULL;
  imRect = NULL;
  imRectColor = NULL;
  imRaw = NULL;
  imRawType = COLOR_CODING_NONE;
  imRawSize = 0;
  imType = COLOR_CODING_NONE;
  imSize = 0;
  imColorType = COLOR_CODING_NONE;
  imColorSize = 0;
  imRectType = COLOR_CODING_NONE;
  imRectSize = 0;
  imRectColorType = COLOR_CODING_NONE;
  imRectColorSize = 0;
  params = NULL;

  // color conversion
  colorConvertType = COLOR_CONVERSION_BILINEAR;
  //  colorConvertType = COLOR_CONVERSION_EDGE;

  // rectification mapping
  hasRectification = false;
  initRect = false;
  rMapxy = NULL;
  rMapa = NULL;

  // calibration matrices
  rD = cvCreateMat(5,1,CV_64F);
  rK = cvCreateMat(3,3,CV_64F);
  rR = cvCreateMat(3,3,CV_64F);
  rKp = cvCreateMat(3,3,CV_64F);

  // temp images, these will be changed when used
  srcIm = cvCreateImageHeader(cvSize(640,480), IPL_DEPTH_8U, 1);
  dstIm = cvCreateImageHeader(cvSize(640,480), IPL_DEPTH_8U, 1);
}

ImageData::~ImageData()
{
  releaseBuffers();
  cvReleaseImageHeader(&srcIm);
  cvReleaseImageHeader(&dstIm);
}

// storage

void
ImageData::releaseBuffers()
{
  // should we release im_raw???
  MEMFREE(im);
  MEMFREE(imColor);
  MEMFREE(imRect);
  MEMFREE(imRectColor);
  im = NULL;
  imColor = NULL;
  imRect = NULL;
  imRectColor = NULL;
  imRaw = NULL;

  imRawType = COLOR_CODING_NONE;
  imRawSize = 0;
  imType = COLOR_CODING_NONE;
  imSize = 0;
  imColorType = COLOR_CODING_NONE;
  imColorSize = 0;
  imRectType = COLOR_CODING_NONE;
  imRectSize = 0;
  imRectColorType = COLOR_CODING_NONE;
  imRectColorSize = 0;

  initRect = false;
  if (rMapxy)
    cvReleaseMat(&rMapxy);
  if (rMapa)
    cvReleaseMat(&rMapa);
  rMapxy = NULL;
  rMapa = NULL;
}



// rectification

bool
ImageData::initRectify(bool force)
{
  if (force)
    hasRectification = true;

  if (!hasRectification || imWidth == 0 || imHeight == 0)
    return false;

  if (initRect && !force && rMapxy != NULL && rMapa != NULL)
    return true;		// already done

  // set values of cal matrices
  for (int i=0; i<3; i++)
    for (int j=0; j<3; j++)
      CV_MAT_ELEM(*rK, double, i, j) = K[i*3+j];

  // rectified K matrix, from projection matrix
  for (int i=0; i<3; i++)
    for (int j=0; j<3; j++)
      CV_MAT_ELEM(*rKp, double, i, j) = P[i*4+j];

  for (int i=0; i<3; i++)
    for (int j=0; j<3; j++)
      CV_MAT_ELEM(*rR, double, i, j) = R[i*3+j];

  for (int i=0; i<5; i++)
    CV_MAT_ELEM(*rD, double, i, 0) = D[i];

  // Set up rectification mapping
  rMapxy = cvCreateMat(imHeight, imWidth, CV_16SC2);
  rMapa  = cvCreateMat(imHeight, imWidth, CV_16UC1);//CV_16SC1);
  mx = cvCreateMat(imHeight, imWidth, CV_32FC1);
  my = cvCreateMat(imHeight, imWidth, CV_32FC1);
  cvInitUndistortRectifyMap(rK,rD,rR,rKp,mx,my);
  cvConvertMaps(mx,my,rMapxy,rMapa);

  initRect = true;
  return true;
}


bool
ImageData::doRectify()
{
  if (!hasRectification)
  {
    return false;		// has no rectification
  }

  if (imWidth == 0 || imHeight == 0)
  {
    return false;
  }

  doBayerMono();

  if (imType == COLOR_CODING_NONE && imColorType == COLOR_CODING_NONE)
  {
    return false;		// nothing to rectify
  }

  if (!((imType != COLOR_CODING_NONE && imRectType == COLOR_CODING_NONE) ||
	(imColorType != COLOR_CODING_NONE && imRectColorType == COLOR_CODING_NONE)))
  {
    return true;		// already done
  }

  initRectify();		// ok to call multiple times

  CvSize size = cvSize(imWidth,imHeight);

  // rectify grayscale image
  if (imType != COLOR_CODING_NONE)
    {
      // set up rectified data buffer
      if (imRectSize < imSize)
	{
	  MEMFREE(imRect);
	  imRectSize = imWidth*imHeight;
	  imRect = (uint8_t *)MEMALIGN(imSize);
	}

      // set up images
      imRectType = imType;
      cvInitImageHeader(srcIm, size, IPL_DEPTH_8U, 1);
      cvInitImageHeader(dstIm, size, IPL_DEPTH_8U, 1);
      cvSetData(srcIm, im, imWidth);
      cvSetData(dstIm, imRect, imWidth);

      cvRemap(srcIm,dstIm,rMapxy,rMapa);
      //cvRemap(srcIm,dstIm,mx,my);
    }

  // rectify color image
  // assumes RGB
  if (imColorType != COLOR_CODING_NONE)
    {
      // set up rectified data buffer
      if (imRectColorSize < imColorSize)
	{
	  MEMFREE(imRectColor);
	  imRectColorSize = imWidth*imHeight*3;
	  imRectColor = (uint8_t *)MEMALIGN(imRectColorSize);
	}

      // set up images
      imRectColorType = imColorType;
      cvInitImageHeader(srcIm, size, IPL_DEPTH_8U, 3);
      cvInitImageHeader(dstIm, size, IPL_DEPTH_8U, 3);
      cvSetData(srcIm, imColor, imWidth*3);
      cvSetData(dstIm, imRectColor, imWidth*3);

      cvRemap(srcIm,dstIm,rMapxy,rMapa);
      //cvRemap(srcIm,dstIm,mx,my);
    }
  return true;
}



// stereo class fns

StereoData::StereoData()
{
  imLeft = new ImageData();
  imRight = new ImageData();

  // disparity buffer
  imDisp = NULL;
  imDispSize = 0;
  buf = NULL;
  flim = NULL;
  frim = NULL;
  wbuf = NULL;
  rbuf = NULL;
  lbuf = NULL;
  maxyim = maxxim = maxdlen = maxcorr = 0;

  // nominal values
  imWidth = 640;
  imHeight = 480;
  corrSize = 15;
  filterSize = 11;
  horOffset = 0;
  setDispOffsets();
  dpp = 16;
  numDisp = 64;
  offx = 0;

  textureThresh = 10;
  uniqueThresh = 12;
  smoothThresh = 30;
  speckleDiff = 8;
  speckleRegionSize = 100;
  rangeMax = 0.0;
  rangeMin = 0.0;
  unique_check = 0;

  hasRectification = false;

  // point array/vector
  numPts = 0;
  imPts = NULL;
  imCoords = NULL;
  imPtsColor = NULL;
  isPtArray = false;
  imPtsSize = 0;
}


StereoData::~StereoData()
{
  releaseBuffers();
  free(imLeft);
  free(imRight);
  // should free all buffers
  MEMFREE(buf);
  MEMFREE(flim);
  MEMFREE(frim);
}

bool
StereoData::setHoropter(int val)
{
  if (val < 0) val = 0;
  if (val > 128) val = 128;
  offx = val;
  return true;
}

bool
StereoData::setTextureThresh(int val)
{
  if (val < 0) val = 0;
  if (val > 100) val = 10;
  textureThresh = val;
  return true;
}

bool
StereoData::setUniqueThresh(int val)
{
  if (val < 0) val = 0;
  if (val > 100) val = 10;
  uniqueThresh = val;
  return true;
}

bool
StereoData::setSmoothnessThresh(int val)
{
  if (val < 0) val = 0;
  if (val > 100) val = 10;
  smoothThresh = val;
  return true;
}


bool
StereoData::setNumDisp(int val)
{
  val = (val/16)*16;		// set to multiple of 16
  if (val < 0) val = 0;
  if (val > 256) val = 256;
  numDisp = val;
  PRINTF("[StereoData] Num disp set to %d\n", val);
  return true;
}


bool
StereoData::setCorrSize(int val)
{
  val = val | 0x1;		// must be odd
  if (val < 5) val = 5;
  if (val > 23) val = 23;
  corrSize = val;
  //  PRINTF("[StereoData] Stereo correlation window size set to %d\n", val);
  setDispOffsets();
  return true;
}


bool
StereoData::setRangeMax(double val)
{
  if (val < 0.0) val = 0.0;
  rangeMax = val;
  return true;
}


bool
StereoData::setRangeMin(double val)
{
  if (val < 0.0) val = 0.0;
  rangeMin = val;
  return true;
}

bool
StereoData::setUniqueCheck(bool val)
{
  unique_check = !unique_check;
  return true;
}

void
StereoData::setDispOffsets()
{
  /*
   * disparity image size
   * ====================
   * dleft  : (logs + corrs - 2)/2 - 1 + offx
   * dwidth : w - (logs + corrs + offx - 2)
   * dtop   : (logs + corrs - 2)/2
   * dheight: h - (logs + corrs)
   *
   */

  imDtop = (filterSize + corrSize - 2)/2;
  imDleft = (filterSize + corrSize - 2)/2 - 1 + horOffset;
  imDwidth = imWidth - (filterSize + corrSize + horOffset - 2);
  imDheight = imHeight - (filterSize + corrSize);
}

void
StereoData::releaseBuffers()
{
  MEMFREE(imDisp);
  imDisp = NULL;
  imDispSize = 0;
  hasDisparity = false;
  MEMFREE(imPts);
  MEMFREE(imCoords);
  imPtsSize = 0;
  numPts = 0;
  imLeft->releaseBuffers();
  imRight->releaseBuffers();
}



// image size
// needs to deal with buffers
void
StereoData::setSize(int width, int height)
{
  imWidth = width;
  imHeight = height;
  imLeft->imWidth = width;
  imLeft->imHeight = height;
  //  imLeft->imSize = width*height;
  imRight->imWidth = width;
  imRight->imHeight = height;
  //  imRight->imSize = width*height;
}


//
// rectification and  stereo processing
//

bool
StereoData::doRectify()
{
  bool res = imLeft->doRectify();
  res = imRight->doRectify() && res;
  return res;
}

//
// Color processing
//
void
StereoData::doBayerColorRGB()
{
  imLeft->doBayerColorRGB();
  imRight->doBayerColorRGB();
}

//
// Mono processing
//
void
StereoData::doBayerMono()
{
  imLeft->doBayerMono();
  imRight->doBayerMono();
}


//
// includes post-filtering
//

bool
StereoData::doDisparity(stereo_algorithm_t alg)
{
  uint8_t *lim, *rim;

  // first do any rectification necessary
  doRectify();

  // check if disparity is already present
  if (hasDisparity)
    return true;

  // check if the rectified images are present
  if (imLeft->imRectType == COLOR_CODING_NONE ||
      imRight->imRectType == COLOR_CODING_NONE)
    return false;

  // variables
  lim = (uint8_t *)imLeft->imRect;
  rim = (uint8_t *)imRight->imRect;
  int xim = imWidth;
  int yim = imHeight;

  // some parameters
  int ftzero = 31;		// max 31 cutoff for prefilter value (31 default)
  int dlen   = numDisp;		// number of disparities
  int corr   = corrSize;	// correlation window size
  int tthresh = textureThresh;	// texture threshold
  int uthresh = uniqueThresh;	// uniqueness threshold, percent
  int sthresh = smoothThresh;
  bool unique_c = unique_check;

  // printf("Unique check: %d\r", unique_check);

  // allocate buffers
  // TODO: make these consistent with current values
  if (!imDisp)
    imDisp = (int16_t *)MEMALIGN(xim*yim*2);

  if (!buf || yim*dlen*(corr+5) > maxyim*maxdlen*(maxcorr+5))
    buf  = (uint8_t *)MEMALIGN(yim*2*dlen*(corr+5)); // local storage for the algorithm
  if (!flim || xim*yim > maxxim*maxyim)
    flim = (uint8_t *)MEMALIGN(xim*yim); // feature image
  if (!frim || xim*yim > maxxim*maxyim)
    frim = (uint8_t *)MEMALIGN(xim*yim); // feature image
  if (xim > maxxim) maxxim = xim;
  if (yim > maxyim) maxyim = yim;
  if (dlen > maxdlen) maxdlen = dlen;
  if (corr > maxcorr) maxcorr = corr;

  // prefilter
  do_prefilter(lim, flim, xim, yim, ftzero, buf);
  do_prefilter(rim, frim, xim, yim, ftzero, buf);

  // clear disparity buffer - do we need to do this???
  memset(imDisp, 0, xim*yim*sizeof(int16_t));


  // use appropriate algorithm
  switch(alg){
	case NORMAL_ALGORITHM:
  		do_stereo(flim, frim, imDisp, NULL, xim, yim,
	    	ftzero, corr, corr, dlen, tthresh, uthresh, buf);
	break;
	case SCANLINE_ALGORITHM:
		do_stereo_so(flim, frim, imDisp, xim, yim,
	    	ftzero, corr, corr, dlen, tthresh, uthresh, sthresh, unique_c);
	break;
	case DP_ALGORITHM:
		do_stereo_dp(flim, frim, imDisp, xim, yim,
	    	ftzero, corr, corr, dlen, tthresh, uthresh, sthresh, unique_c);
	break;
	case MW_ALGORITHM:
		do_stereo_mw(flim, frim, imDisp, xim, yim,
		   ftzero, corr, corr, dlen, tthresh, uthresh, unique_c);
	break;
	case LS_ALGORITHM:
		do_stereo_ls(flim, frim, imDisp, xim, yim,
		    ftzero, corr, corr, dlen, tthresh, uthresh, sthresh, unique_c);
	break;
	case NCC_ALGORITHM:
		do_stereo_ncc(flim, frim, imDisp, xim, yim,
	    	ftzero, corr, corr, dlen, tthresh, uthresh, unique_c);
	break;

	default:
	PRINTF("No algorithm has been selected..sorry!\n");
  }



  hasDisparity = true;

  doSpeckle();
  return true;
}



//
// apply speckle filter
// useful for STOC processing, where it's not done on-camera
//

bool
StereoData::doSpeckle()
{
  if (!hasDisparity) return false;

  // speckle filter
  if (speckleRegionSize > 0)
    {
      int xim = imWidth;
      int yim = imHeight;
      if (!rbuf)
	rbuf  = (uint8_t *)malloc(xim*yim); // local storage for the algorithm
      if (!lbuf)
	lbuf = (uint32_t *)malloc(xim*yim*sizeof(uint32_t)); // local storage for the algorithm
      if (!wbuf)
	wbuf = (uint32_t *)malloc(xim*yim*sizeof(uint32_t)); // local storage for the algorithm
      do_speckle(imDisp, 0, xim, yim, speckleDiff, speckleRegionSize,
		 lbuf, wbuf, rbuf);
    }
  return true;
}


bool
StereoData::setSpeckleRegionSize(int val)
{
  speckleRegionSize = val;
  return true;
}

bool
StereoData::setSpeckleDiff(int val)
{
  speckleDiff = val;
  return true;
}



//
// param sting parsing routines
//

#include <iostream>
using namespace std;

template <class T>
void extract(std::string& data, std::string section, std::string param, T& t)
{
  size_t found = data.find(section);
  if (found != string::npos)
    {
      found = data.find(param,found);
      if (found != string::npos)
	{
	  std::istringstream iss(data.substr(found+param.length()));
	  iss >> t;
	}
    }
}

void extract(std::string& data, std::string section,
		  std::string param, double *m, int n)
{
  size_t found = data.find(section);
  if (found != string::npos)
    {
      found = data.find(param,found);
      if (found != string::npos)
	{
	  std::istringstream iss(data.substr(found+param.length()));
	  double v;
	  for (int i=0; i<n; i++)
	    {
	      iss >> v;
	      m[i] = v;
	    }
	}
    }
}


//
// Conversion to 3D points
// Convert to vector or image array of pts, depending on isArray arg
// Should we do disparity automatically here?
//


bool
StereoData::doCalcPts(bool isArray)
{
  numPts = 0;
  doDisparity();
  if (!hasDisparity)
    return false;

  int ix = imDleft;
  int iy = imDtop;
  int ih = imDheight;
  int iw = imDwidth;
  int w = imWidth;
  int h = imHeight;
  int dmax = 0;
  int dmin = 0x7fff;

  if (isArray)			// don't need these for arrays
    {
      isPtArray = true;
      MEMFREE(imCoords);
      MEMFREE(imPtsColor);
    }
  else
    isPtArray = false;

  if (imPtsSize < 4*w*h*sizeof(float))
    {
      MEMFREE(imPts);
      imPtsSize = 4*w*h*sizeof(float);
      imPts = (float *)MEMALIGN(imPtsSize);
      MEMFREE(imCoords);
      MEMFREE(imPtsColor);
      if (!isArray)
	{
	  imPtsColor = (uint8_t *)MEMALIGN(3*w*h);
	  imCoords = (int *)MEMALIGN(2*w*h*sizeof(int));
	}
    }

  float *pt = imPts;
  pt_xyza_t *ppt = (pt_xyza_t *)imPts;
  int *pcoord;
  int y = iy;
  float cx = (float)RP[3];
  float cy = (float)RP[7];
  float f  = (float)RP[11];
  float itx = (float)RP[14];
  itx *= 1.0 / (float)dpp; // adjust for subpixel interpolation
  pcoord = imCoords;

  // set up range max/min disparity
  if (rangeMax > 0.0)
    {
      double dm = f / (rangeMax*itx);
      dmax = (int)dm;
    }
  if (rangeMin > 0.0)
    {
      double dm = f / (rangeMin*itx);
      dmin = (int)dm;
    }


  if (isArray)			// make an array of pts
    {
      numPts = w*h;
      for (int j=0; j<h; j++)
	{
	  int16_t *p = imDisp + j*w;

	  for (int i=0; i<w; i++, p++, ppt++)
	    {
	      if (*p > dmax && *p < dmin)
		{
		  float ax = (float)i + cx;
		  float ay = (float)j + cy;
		  float aw = 1.0 / (itx * (float)*p);
		  ppt->X = ax*aw; // X
		  ppt->Y = ay*aw; // Y
		  ppt->Z = f*aw; // Z
		  ppt->A = 0;
		}
	      else
		{
		  ppt->X = 0.0; // X
		  ppt->Y = 0.0; // Y
		  ppt->Z = 0.0; // Z
		  ppt->A = -1;	// invalid point
		}
	    }
	}
    }
  else				// make a vector of pts
    {
      for (int j=0; j<ih; j++, y++)
	{
	  int x = ix;
	  int16_t *p = imDisp + x + y*w;

	  for (int i=0; i<iw; i++, x++, p++)
	    {
	      if (*p > dmax && *p < dmin)
		{
		  float ax = (float)x + cx;
		  float ay = (float)y + cy;
		  float aw = 1.0 / (itx * (float)*p);
		  *pt++ = ax*aw;	// X
		  *pt++ = ay*aw;	// Y
		  *pt++ = f*aw;	// Z
		  // store point image coordinates
		  *pcoord++ = x;
		  *pcoord++ = y;
		  numPts++;
		}
	    }
	}
    }


  if (isArray) return true;

  if (imLeft->imRectColorType != COLOR_CODING_NONE) // ok, have color
    {
      y = iy;
      uint8_t *pcout = imPtsColor;
      for (int j=0; j<ih; j++, y++)
	{
	  int x = ix;
	  int16_t *p = imDisp + x + y*w;
	  uint8_t *pc = imLeft->imRectColor + (x + y*w)*3;

	  for (int i=0; i<iw; i++, x++, p++, pc+=3)
	    {
	      if (*p > dmax && *p < dmin)
		{
		  *pcout++ = *pc;
		  *pcout++ = *(pc+1);
		  *pcout++ = *(pc+2);
		}
	    }
	}
    }
  else if (imLeft->imRectType != COLOR_CODING_NONE) // ok, have mono
    {
      y = iy;
      uint8_t *pcout = imPtsColor;
      for (int j=0; j<ih; j++, y++)
	{
	  int x = ix;
	  int16_t *p = imDisp + x + y*w;
	  uint8_t *pc = imLeft->imRect + (x + y*w);

	  for (int i=0; i<iw; i++, p++, pc++)
	    {
	      if (*p > dmax && *p < dmin)
		{
		  *pcout++ = *pc;
		  *pcout++ = *pc;
		  *pcout++ = *pc;
		}
	    }
	}
    }


  //  printf("[Calc Pts] Number of pts: %d\n", numPts);
  return true;
}



//
// Conversion to 3D points
// Just a single point
//


bool
StereoData::calcPt(int x, int y, float *fx, float *fy, float *fz)
{
  doDisparity();
  if (!hasDisparity)
    return false;

  float cx = (float)RP[3];
  float cy = (float)RP[7];
  float f  = (float)RP[11];
  float itx = (float)RP[14];
  itx *= 1.0 / (float)dpp; // adjust for subpixel interpolation

  int16_t *p = imDisp + x + y*imWidth;

  if (*p > 0)
    {
      float ax = (float)x + cx;
      float ay = (float)y + cy;
      float aw = 1.0 / (itx * (float)*p);
      *fx = ax*aw;	// X
      *fy = ay*aw;	// Y
      *fz = f*aw;	// Z
    }

  if (*p)
    return true;
  else
    return false;
}



//
// gets params from a string
// "SVS"-type parameter strings use mm for the projection matrices, convert to m
// "OST"-type parameter strings use m for projection matrices
//

void
StereoData::extractParams(char *ps, bool store)
{
  std::string params;
  params = ps;
  double *pp;
  bool isSVS = false;

  if (store && ps != NULL)
    {
      if (imLeft->params)
	delete [] imLeft->params;
      char *bb = new char[strlen(ps)];
      strcpy(bb,ps);
      imLeft->params = bb;
    }

  PRINTF("\n\n[extractParams] Parameters:\n\n");

  if (strncmp(ps,"# SVS",5)==0) // SVS-type parameters
    {
      PRINTF("[dcam] SVS-type parameters\n");
      isSVS = true;


      // left image
      for (int i=0; i<9; i++) imLeft->K[i] = 0.0; // original camera matrix
      extract(params, "[left camera]", "f ", imLeft->K[0]); // have to use space after "f"
      extract(params, "[left camera]", "fy", imLeft->K[4]);
      extract(params, "[left camera]", "Cx", imLeft->K[2]);
      extract(params, "[left camera]", "Cy", imLeft->K[5]);
      imLeft->K[8] = 1.0;

      for (int i=0; i<5; i++) imLeft->D[i] = 0.0; // distortion params
      extract(params, "[left camera]", "kappa1", imLeft->D[0]);
      extract(params, "[left camera]", "kappa2", imLeft->D[1]);
      extract(params, "[left camera]", "tau1", imLeft->D[2]);
      extract(params, "[left camera]", "tau2", imLeft->D[3]);
      extract(params, "[left camera]", "kappa3", imLeft->D[4]);

      pp = (double *)imLeft->R; // rectification matrix
      for (int i=0; i<9; i++) pp[i] = 0.0;
      extract(params, "[left camera]", "rect",  pp, 9);

      pp = (double *)imLeft->P; // projection matrix
      for (int i=0; i<12; i++) pp[i] = 0.0;
      extract(params, "[left camera]", "proj",  pp, 12);
      if (isSVS)
	imLeft->P[3] *= .001;	// convert from mm to m

      // right image
      for (int i=0; i<9; i++) imRight->K[i] = 0.0; // original camera matrix
      extract(params, "[right camera]", "f ", imRight->K[0]); // ?? have to use "f " here
      extract(params, "[right camera]", "fy", imRight->K[4]);
      extract(params, "[right camera]", "Cx", imRight->K[2]);
      extract(params, "[right camera]", "Cy", imRight->K[5]);
      imRight->K[8] = 1.0;

      for (int i=0; i<5; i++) imRight->D[i] = 0.0; // distortion params
      extract(params, "[right camera]", "kappa1", imRight->D[0]);
      extract(params, "[right camera]", "kappa2", imRight->D[1]);
      extract(params, "[right camera]", "tau1", imRight->D[2]);
      extract(params, "[right camera]", "tau2", imRight->D[3]);
      extract(params, "[right camera]", "kappa3", imRight->D[4]);

      pp = (double *)imRight->R; // rectification matrix
      for (int i=0; i<9; i++) pp[i] = 0.0;
      extract(params, "[right camera]", "rect",  pp, 9);

      pp = (double *)imRight->P; // projection matrix
      for (int i=0; i<12; i++) pp[i] = 0.0;
      extract(params, "[right camera]", "proj",  pp, 12);
      imRight->P[3] *= .001;	// convert from mm to m

      // external params of undistorted cameras
      for (int i=0; i<3; i++) T[i] = 0.0;
      for (int i=0; i<3; i++) Om[i] = 0.0;
      extract(params, "[external]", "Tx", T[0]);
      extract(params, "[external]", "Ty", T[1]);
      extract(params, "[external]", "Tz", T[2]);
      extract(params, "[external]", "Rx", Om[0]);
      extract(params, "[external]", "Ry", Om[1]);
      extract(params, "[external]", "Rz", Om[2]);

      T[0] *= .001;
      T[1] *= .001;
      T[2] *= .001;

    }

  // OST-type parameters
  else
    {
      PRINTF("[dcam] OST-type parameters\n");
      isSVS = false;

      // left image
      for (int i=0; i<9; i++) imLeft->K[i] = 0.0; // original camera matrix
      extract(params, "[left camera]", "camera matrix", imLeft->K, 9);

      for (int i=0; i<5; i++) imLeft->D[i] = 0.0; // distortion params
      extract(params, "[left camera]", "distortion", imLeft->D, 5);

      pp = (double *)imLeft->R; // rectification matrix
      for (int i=0; i<9; i++) pp[i] = 0.0;
      extract(params, "[left camera]", "rectification",  imLeft->R, 9);

      pp = (double *)imLeft->P; // projection matrix
      for (int i=0; i<12; i++) pp[i] = 0.0;
      extract(params, "[left camera]", "projection",  imLeft->P, 12);

      // right image
      for (int i=0; i<9; i++) imRight->K[i] = 0.0; // original camera matrix
      extract(params, "[right camera]", "camera matrix", imRight->K, 9);

      for (int i=0; i<5; i++) imRight->D[i] = 0.0; // distortion params
      extract(params, "[right camera]", "distortion", imRight->D, 5);

      pp = (double *)imRight->R; // rectification matrix
      for (int i=0; i<9; i++) pp[i] = 0.0;
      extract(params, "[right camera]", "rectification",  pp, 9);

      pp = (double *)imRight->P; // projection matrix
      for (int i=0; i<12; i++) pp[i] = 0.0;
      extract(params, "[right camera]", "projection",  pp, 12);

      // external params of undistorted cameras
      for (int i=0; i<3; i++) T[i] = 0.0;
      for (int i=0; i<3; i++) Om[i] = 0.0;
      extract(params, "[externals]", "translation", T, 3);
      extract(params, "[externals]", "rotation", Om, 3);
    }



  // disparity resolution
  extract(params, "[stereo]", "dpp", dpp);
  PRINTF("[dcam] Disparity resolution: 1/%d pixel\n", dpp);
  extract(params, "[stereo]", "corrxsize", corrSize);
  PRINTF("[dcam] Correlation window: %d\n", corrSize);
  extract(params, "[stereo]", "convx", filterSize);
  PRINTF("[dcam] Prefilter window: %d\n", filterSize);
  extract(params, "[stereo]", "ndisp", numDisp);
  PRINTF("[dcam] Number of disparities: %d\n", numDisp);


  PRINTF("[dcam] Left camera matrix\n");
  for (int i=0; i<3; i++)
    {
      for (int j=0; j<3; j++)
	PRINTF(" %.4f",imLeft->K[i*3+j]);
      PRINTF("\n");
    }
  PRINTF("\n");

  PRINTF("[dcam] Left distortion vector\n");
  for (int i=0; i<5; i++)
    PRINTF(" %.4f",imLeft->D[i]);
  PRINTF("\n");
  PRINTF("\n");

  PRINTF("[dcam] Left rectification matrix\n");
  for (int i=0; i<3; i++)
    {
      for (int j=0; j<3; j++)
	PRINTF(" %.4f",imLeft->R[i*3+j]);
      PRINTF("\n");
    }
  PRINTF("\n");

  PRINTF("[dcam] Left projection matrix\n");
  for (int i=0; i<3; i++)
    {
      for (int j=0; j<4; j++)
	PRINTF(" %.4f",imLeft->P[i*4+j]);
      PRINTF("\n");
    }
  PRINTF("\n");

  // check for camera matrix
  if (imLeft->K[0] == 0.0)
    {
      hasRectification = false;
      PRINTF("[dcam] No rectification\n\n");
    }
  else
    {
      hasRectification = true;
      imLeft->hasRectification = true;
      imLeft->initRect = false;	// haven't initialized arrays, wait for image size
    }

  PRINTF("[dcam] Right camera matrix\n");
  for (int i=0; i<3; i++)
    {
      for (int j=0; j<3; j++)
	PRINTF(" %.4f",imRight->K[i*3+j]);
      PRINTF("\n");
    }
  PRINTF("\n");

  PRINTF("[dcam] Right distortion vector\n");
  for (int i=0; i<5; i++)
    PRINTF(" %.4f",imRight->D[i]);
  PRINTF("\n");
  PRINTF("\n");

  PRINTF("[dcam] Right rectification matrix\n");
  for (int i=0; i<3; i++)
    {
      for (int j=0; j<3; j++)
	PRINTF(" %.4f",imRight->R[i*3+j]);
      PRINTF("\n");
    }
  PRINTF("\n");

  PRINTF("[dcam] Right projection matrix\n");
  for (int i=0; i<3; i++)
    {
      for (int j=0; j<4; j++)
	PRINTF(" %.4f",imRight->P[i*4+j]);
      PRINTF("\n");
    }
  PRINTF("\n");

  // reprojection matrix
  double Tx = imRight->P[0] /  imRight->P[3];
  // first column
  RP[0] = 1.0;
  RP[4] = RP[8] = RP[12] = 0.0;

  // second column
  RP[5] = 1.0;
  RP[1] = RP[9] = RP[13] = 0.0;

  // third column
  RP[2] = RP[6] = RP[10] = 0.0;
  RP[14] = -Tx;

  // fourth column
  RP[3] = -imLeft->P[2];	// cx
  RP[7] = -imLeft->P[6];	// cy
  RP[11] = imLeft->P[0];	// fx, fy
  RP[15] = (imLeft->P[2] - imRight->P[2] - (double)offx) / Tx;

  PRINTF("[dcam] Reprojection matrix\n");
  for (int i=0; i<4; i++)
    {
      for (int j=0; j<4; j++)
	PRINTF(" %.4f",RP[i*4+j]);
      PRINTF("\n");
    }
  PRINTF("\n");


  PRINTF("[dcam] External translation vector\n");
  for (int i=0; i<3; i++)
    PRINTF(" %.4f",T[i]);
  PRINTF("\n");
  PRINTF("\n");

  PRINTF("[dcam] External rotation vector\n");
  for (int i=0; i<3; i++)
    PRINTF(" %.4f",Om[i]);
  PRINTF("\n");
  PRINTF("\n");


  // check for camera matrix
  if (imRight->K[0] == 0.0)
    hasRectification = false;
  else
    {
      imRight->hasRectification = true;
      imRight->initRect = false; // haven't initialized arrays, wait for image size
      PRINTF("[dcam] Has rectification\n\n");
    }
}


//
// Create parameters string and save it in the imLeft->params location
//

static int
PrintMatStr(double *mat, int n, int m, char *str)
{
  int c=0;
  for (int i=0; i<n; i++)
    {
      for (int j=0; j<m; j++)
	c += sprintf(&str[c],"%8.5f ", mat[i*m+j]);
      c += sprintf(&str[c],"\n");
    }
  return c;
}

static void
PrintMat(double *mat, int n, int m)
{
  for (int i=0; i<n; i++)
    {
      for (int j=0; j<m; j++)
	printf("%8.5f ", mat[i*m+j]);
      printf("\n");
    }
}


static int
PrintStr(int val, char *str)
{
  int c=0;
  c += sprintf(&str[c],"%d ", val);
  return c;
}

char *
StereoData::createParams(bool store)
{
  char *str = new char[4096];
  int n = 0;

  // header
  n += sprintf(str,"# oST version %d.%d parameters\n\n", OST_MAJORVERSION, OST_MINORVERSION);

  // stereo params
  n += sprintf(&str[n],"\n[stereo]\n");
  n += sprintf(&str[n],"\nndisp    ");
  n += PrintStr(numDisp,&str[n]);
  n += sprintf(&str[n],"\ndpp      ");
  n += PrintStr(dpp,&str[n]);
  n += sprintf(&str[n],"\ncorrsize ");
  n += PrintStr(corrSize,&str[n]);
  n += sprintf(&str[n],"\npresize  ");
  n += PrintStr(filterSize,&str[n]);

  // externals
  n += sprintf(&str[n],"\n\n[externals]\n");

  n += sprintf(&str[n],"\ntranslation\n");
  n += PrintMatStr(T,1,3,&str[n]);

  n += sprintf(&str[n],"\nrotation\n");
  n += PrintMatStr(Om,1,3,&str[n]);

  // left camera
  n += sprintf(&str[n],"\n[left camera]\n");

  n += sprintf(&str[n],"\ncamera matrix\n");
  n += PrintMatStr(imLeft->K,3,3,&str[n]);

  n += sprintf(&str[n],"\ndistortion\n");
  n += PrintMatStr(imLeft->D,1,5,&str[n]);

  n += sprintf(&str[n],"\nrectification\n");
  n += PrintMatStr(imLeft->R,3,3,&str[n]);

  n += sprintf(&str[n],"\nprojection\n");
  n += PrintMatStr(imLeft->P,3,4,&str[n]);

  // right camera
  n += sprintf(&str[n],"\n[right camera]\n");
  n += sprintf(&str[n],"\ncamera matrix\n");
  n += PrintMatStr(imRight->K,3,3,&str[n]);

  n += sprintf(&str[n],"\ndistortion\n");
  n += PrintMatStr(imRight->D,1,5,&str[n]);

  n += sprintf(&str[n],"\nrectification\n");
  n += PrintMatStr(imRight->R,3,3,&str[n]);

  n += sprintf(&str[n],"\nprojection\n");
  n += PrintMatStr(imRight->P,3,4,&str[n]);

  str[n] = 0;			// just in case

  if (store)
    {
      if (imLeft->params)
	delete [] imLeft->params;
      char *bb = new char[n];
      strcpy(bb,str);
      imLeft->params = bb;
    }

  return str;
}



//
// color processing
// two algorithms: linear interpolation, and edge-tracking interpolation
//

// convert from Bayer to RGB (3 bytes)

#define AVG(a,b) (((int)(a) + (int)(b))>>1)

void
ImageData::doBayerColorRGB()
{
  if (imRawType == COLOR_CODING_NONE)
  {
    return;		// nothing to colorize
  }

  if (imColorType != COLOR_CODING_NONE)
  {
    return;		// already done
  }

  // check allocation
  size_t size = imWidth*imHeight;
  if (imSize < size)
    {
      MEMFREE(im);
      im = (uint8_t *)MEMALIGN(size);
      imSize = size;
    }
  if (imColorSize < size*3)
    {
      MEMFREE(imColor);
      imColor = (uint8_t *)MEMALIGN(size*3);
      imColorSize = size*3;
    }
  switch (imRawType) {
    case COLOR_CODING_MONO8:
      memcpy(im, imRaw, size);
      imType = COLOR_CODING_MONO8;
      return;
    case COLOR_CODING_BAYER8_GRBG:
      convertBayerGRBGColorRGB(imRaw, imColor, im, imWidth, imHeight, colorConvertType);
      break;
    case COLOR_CODING_BAYER8_BGGR:
      convertBayerBGGRColorRGB(imRaw, imColor, im, imWidth, imHeight, colorConvertType);
      break;
      
    default:
      PRINTF("Unsupported color coding %i", imRawType);
      return;
  }
  imType = COLOR_CODING_MONO8;
  imColorType = COLOR_CODING_RGB8;
}


void
ImageData::doBayerMono()
{

  if (imRawType == COLOR_CODING_NONE)
  {
    return;		// nothing to colorize
  }

  if (imType != COLOR_CODING_NONE)
  {
    return;		// already done
  }

  // check allocation
  size_t size = imWidth*imHeight;
  if (imSize < size)
    {
      MEMFREE(im);
      im = (uint8_t *)MEMALIGN(size);
      imSize = size;
    }
  switch (imRawType) {
    case COLOR_CODING_MONO8:
      memcpy(im, imRaw, size);
      break;
    case COLOR_CODING_BAYER8_GRBG:
      convertBayerGRBGMono(imRaw, im, imWidth, imHeight, colorConvertType);
      break;
    case COLOR_CODING_BAYER8_BGGR:
      convertBayerBGGRMono(imRaw, im, imWidth, imHeight, colorConvertType);
      break;
      
    default:
      PRINTF("Unsupported color coding %i", imRawType);
      return;
  }
  imType = COLOR_CODING_MONO8;
}


// real funtion to do the job
// converts to RGB

void
ImageData::convertBayerGRBGColorRGB(uint8_t *src, uint8_t *dstc, uint8_t *dstm,
				    int width, int height, color_conversion_t colorAlg)
{
  uint8_t *s;
  int i, j;
  int ll = width;
  int ll2 = width*2;

  s = src;
  int pp2 = width*3*2;          // previous 2 color lines
  int pp = width*3;             // previous color line
  uint8_t *cd = dstc;		// color
  uint8_t *md = dstm;		// monochrome

  // simple, but has "zipper" artifacts
  if (colorAlg == COLOR_CONVERSION_BILINEAR)
    {
      for (i=0; i<height; i+=2)
	{
	  // red line (GRGR...)
	  for (j=0; j<width; j+=2, cd+=6, md+=2)
	    {
	      *md = *(cd+1) = *s++;	// green pixel
	      *(cd+3+0) = *s++;	// red pixel
	      *(cd+0) = AVG(*(cd+3+0), *(cd-3+0)); // interpolated red pixel
	      if (i > 1)
		{
		  *(cd-pp+0) = AVG(*(cd-pp2+0), *(cd+0)); // interpolated red pixel
		  *(cd-pp+3+0) = AVG(*(cd-pp2+3+0), *(cd+3+0)); // interpolated red pixel
		  *(md-ll) = *(cd-pp+1) = ((int)*(cd+1) + (int)*(cd-pp-3+1) + (int)*(cd-pp+3+1) + (int)*(cd-pp2+1)) >> 2;
		}
	    }

	  // blue line (BGBG...)
	  *(cd+2) = *s;		// blue pixel
	  for (j=0; j<width-2; j+=2, cd+=6, md+=2)
	    {
	      s++;
	      *(md+1) = *(cd+3+1) = *s++; // green pixel
	      *(cd+6+2) = *s;
	      *(cd+3+2) = AVG(*(cd+2), *(cd+6+2)); // interpolated blue pixel
	      if (i > 1)
		{
		  *(cd-pp+2) = AVG(*(cd-pp2+2), *(cd+2)); // interpolated blue pixel
		  *(cd-pp+3+2) = AVG(*(cd-pp2+3+2), *(cd+3+2)); // interpolated blue pixel
		  *(md-ll+1) = *(cd-pp+3+1) = ((int)*(cd+3+1) + (int)*(cd-pp+1) + (int)*(cd-pp+6+1) + (int)*(cd-pp2+3+1)) >> 2;
		}
	    }
	  // last pixels
	  s++;
	  *(md+1) = *(cd+3+1) = *s++;      // green pixel
	  *(cd+3+2) = *(cd+2);	// interpolated blue pixel
	  if (i > 1)
	    {
	      *(cd-pp+2) = AVG(*(cd-pp2+2), *(cd+2)); // interpolated blue pixel
	      *(cd-pp+3+2) = AVG(*(cd-pp2+3+2), *(cd+3+2)); // interpolated blue pixel
	    }
	  cd +=6;
	  md +=2;
	}
    }

  // EDGE color algorithm, better but slower
  else
    {
      int a,b,c,d;
      int dc, dv, dh;
      int ww;

      // do first two lines
      cd += pp2;
      s += ll2;

      for (i=0; i<height-4; i+=2)
	{
	  // GR line
	  // do first two pixels
	  cd += 6;
	  md += 2;
	  s += 2;

	  // do most of line
	  for (j=0; j<width-4; j+=2, cd+=6, md+=2)
	    {
	      // green pixels
	      *md = *(cd+1) = *s++;
	      dc = 2*(int)*(s);
	      dh = dc - (int)*(s-2) - (int)*(s+2);
	      if (dh < 0) dh = -dh;
	      dv = dc - (int)*(s-ll2) - (int)*(s+ll2);
	      if (dv < 0) dv = -dv;
	      if (dv > dh) // vert is stronger, use horz
		*(md+1) = *(cd+3+1) = ((int)*(s-1) + (int)*(s+1))>>1;
	      else	// horz is stronger, use vert
		*(md+1) = *(cd+3+1) = ((int)*(s-ll) + (int)*(s+ll))>>1;

	      // color pixels
	      *(cd+3+0) = *s;	// red pixel

	      a = (int)*(s) - (int)*(cd+3+1);
	      b = (int)*(s-2) - (int)*(cd-3+1);
	      c = (int)*(s-ll2) - (int)*(cd-pp2+3+1);
	      d = (int)*(s-ll2-2) - (int)*(cd-pp2-3+1);

	      ww = 2*(int)*(cd+1) + (a + b);
	      if (ww < 0) ww = 0;
	      *(cd+0) = ww>>1;	// interpolated red pixel

	      ww = 2*(int)*(cd-pp+3+1) + (a + c);
	      if (ww < 0) ww = 0;
	      *(cd-pp+3+0) = ww>>1; // interpolated red pixel

	      ww = 4*(int)*(cd-pp+1) + (a + b + c + d);
	      if (ww < 0) ww = 0;
	      *(cd-pp+0) = ww>>2; // interpolated red pixel

	      s++;
	    }
	  // last two pixels
	  cd += 6;
	  md += 2;
	  s += 2;

	  // BG line
	  // do first two pixels
	  cd += 6;
	  md += 2;
	  s += 2;

	  // do most of line
	  for (j=0; j<width-4; j+=2, cd+=6, md+=2)
	    {
	      dc = 2*(int)*s;
	      dh = dc - (int)*(s-2) - (int)*(s+2);
	      if (dh < 0) dh = -dh;
	      dv = dc - (int)*(s-ll2) - (int)*(s+ll2);
	      if (dv < 0) dv = -dv;
	      if (dh < dv) // vert is stronger, use horz
		*md = *(cd+1) = ((int)*(s-1) + (int)*(s+1))>>1;
	      else	// horz is stronger, use vert
		*md = *(cd+1) = ((int)*(s-ll) + (int)*(s+ll))>>1;

	      *(md+1) = *(cd+3+1) = *(s+1); // green pixel

	      // color pixels
	      *(cd+3+2) = *s;	// blue pixel

	      a = (int)*(s) - (int)*(cd+3+1);
	      b = (int)*(s-2) - (int)*(cd-3+1);
	      c = (int)*(s-ll2) - (int)*(cd-pp2+3+1);
	      d = (int)*(s-ll2-2) - (int)*(cd-pp2-3+1);

	      ww = 2*(int)*(cd+1) + (a + b);
	      if (ww < 0) ww = 0;
	      *(cd+2) = ww>>1;	// interpolated blue pixel

	      ww = 2*(int)*(cd-pp+3+1) + (a + c);
	      if (ww < 0) ww = 0;
	      *(cd-pp+3+2) = ww>>1; // interpolated blue pixel

	      ww = 4*(int)*(cd-pp+1) + (a + b + c + d);
	      if (ww < 0) ww = 0;
	      *(cd-pp+2) = ww>>2; // interpolated blue pixel

	      s+=2;
	    }
	  // last two pixels
	  cd += 6;
	  md += 2;
	  s += 2;
	}

      // last two lines
      for (j=0; j<width; j+=2)
	{
	}
    }

}

void
ImageData::convertBayerBGGRColorRGB(uint8_t *src, uint8_t *dstc, uint8_t *dstm,
				    int width, int height, color_conversion_t colorAlg)
{
  uint8_t *s;
  int i, j;
  int ll = width;
  int ll2 = width*2;

  s = src;
  int pp2 = width*3*2;          // previous 2 color lines
  int pp = width*3;             // previous color line
  uint8_t *cd = dstc;		// color
  uint8_t *md = dstm;		// monochrome

  // simple, but has "zipper" artifacts
  if (colorAlg == COLOR_CONVERSION_BILINEAR)
    {
      for (i=0; i<height; i+=2)
	{
          // blue line (BGBG...)
	  *(cd+2) = *s;		// blue pixel
	  for (j=0; j<width-2; j+=2, cd+=6, md+=2)
	    {
	      s++;
	      *(md+1) = *(cd+3+1) = *s++; // green pixel
	      *(cd+6+2) = *s;
	      *(cd+3+2) = AVG(*(cd+2), *(cd+6+2)); // interpolated blue pixel
	      if (i > 1)
		{
		  *(cd-pp+2) = AVG(*(cd-pp2+2), *(cd+2)); // interpolated blue pixel
		  *(cd-pp+3+2) = AVG(*(cd-pp2+3+2), *(cd+3+2)); // interpolated blue pixel
		  *(md-ll+1) = *(cd-pp+3+1) = ((int)*(cd+3+1) + (int)*(cd-pp+1) + (int)*(cd-pp+6+1) + (int)*(cd-pp2+3+1)) >> 2;
		}
	    }
	  // last pixels
	  s++;
	  *(md+1) = *(cd+3+1) = *s++;      // green pixel
	  *(cd+3+2) = *(cd+2);	// interpolated blue pixel
	  if (i > 1)
	    {
	      *(cd-pp+2) = AVG(*(cd-pp2+2), *(cd+2)); // interpolated blue pixel
	      *(cd-pp+3+2) = AVG(*(cd-pp2+3+2), *(cd+3+2)); // interpolated blue pixel
	    }
	  cd +=6;
	  md +=2;
          
	  // red line (GRGR...)
	  for (j=0; j<width; j+=2, cd+=6, md+=2)
	    {
	      *md = *(cd+1) = *s++;	// green pixel
	      *(cd+3+0) = *s++;	// red pixel
	      *(cd+0) = AVG(*(cd+3+0), *(cd-3+0)); // interpolated red pixel
	      if (i > 1)
		{
		  *(cd-pp+0) = AVG(*(cd-pp2+0), *(cd+0)); // interpolated red pixel
		  *(cd-pp+3+0) = AVG(*(cd-pp2+3+0), *(cd+3+0)); // interpolated red pixel
		  *(md-ll) = *(cd-pp+1) = ((int)*(cd+1) + (int)*(cd-pp-3+1) + (int)*(cd-pp+3+1) + (int)*(cd-pp2+1)) >> 2;
		}
	    }
	}
    }

  // EDGE color algorithm, better but slower
  else
    {
      int a,b,c,d;
      int dc, dv, dh;
      int ww;

      // do first two lines
      cd += pp2;
      s += ll2;

      for (i=0; i<height-4; i+=2)
	{
          // BG line
	  // do first two pixels
	  cd += 6;
	  md += 2;
	  s += 2;

	  // do most of line
	  for (j=0; j<width-4; j+=2, cd+=6, md+=2)
	    {
	      dc = 2*(int)*s;
	      dh = dc - (int)*(s-2) - (int)*(s+2);
	      if (dh < 0) dh = -dh;
	      dv = dc - (int)*(s-ll2) - (int)*(s+ll2);
	      if (dv < 0) dv = -dv;
	      if (dh < dv) // vert is stronger, use horz
		*md = *(cd+1) = ((int)*(s-1) + (int)*(s+1))>>1;
	      else	// horz is stronger, use vert
		*md = *(cd+1) = ((int)*(s-ll) + (int)*(s+ll))>>1;

	      *(md+1) = *(cd+3+1) = *(s+1); // green pixel

	      // color pixels
	      *(cd+3+2) = *s;	// blue pixel

	      a = (int)*(s) - (int)*(cd+3+1);
	      b = (int)*(s-2) - (int)*(cd-3+1);
	      c = (int)*(s-ll2) - (int)*(cd-pp2+3+1);
	      d = (int)*(s-ll2-2) - (int)*(cd-pp2-3+1);

	      ww = 2*(int)*(cd+1) + (a + b);
	      if (ww < 0) ww = 0;
	      *(cd+2) = ww>>1;	// interpolated blue pixel

	      ww = 2*(int)*(cd-pp+3+1) + (a + c);
	      if (ww < 0) ww = 0;
	      *(cd-pp+3+2) = ww>>1; // interpolated blue pixel

	      ww = 4*(int)*(cd-pp+1) + (a + b + c + d);
	      if (ww < 0) ww = 0;
	      *(cd-pp+2) = ww>>2; // interpolated blue pixel

	      s+=2;
	    }
	  // last two pixels
	  cd += 6;
	  md += 2;
	  s += 2;
          
	  // GR line
	  // do first two pixels
	  cd += 6;
	  md += 2;
	  s += 2;

	  // do most of line
	  for (j=0; j<width-4; j+=2, cd+=6, md+=2)
	    {
	      // green pixels
	      *md = *(cd+1) = *s++;
	      dc = 2*(int)*(s);
	      dh = dc - (int)*(s-2) - (int)*(s+2);
	      if (dh < 0) dh = -dh;
	      dv = dc - (int)*(s-ll2) - (int)*(s+ll2);
	      if (dv < 0) dv = -dv;
	      if (dv > dh) // vert is stronger, use horz
		*(md+1) = *(cd+3+1) = ((int)*(s-1) + (int)*(s+1))>>1;
	      else	// horz is stronger, use vert
		*(md+1) = *(cd+3+1) = ((int)*(s-ll) + (int)*(s+ll))>>1;

	      // color pixels
	      *(cd+3+0) = *s;	// red pixel

	      a = (int)*(s) - (int)*(cd+3+1);
	      b = (int)*(s-2) - (int)*(cd-3+1);
	      c = (int)*(s-ll2) - (int)*(cd-pp2+3+1);
	      d = (int)*(s-ll2-2) - (int)*(cd-pp2-3+1);

	      ww = 2*(int)*(cd+1) + (a + b);
	      if (ww < 0) ww = 0;
	      *(cd+0) = ww>>1;	// interpolated red pixel

	      ww = 2*(int)*(cd-pp+3+1) + (a + c);
	      if (ww < 0) ww = 0;
	      *(cd-pp+3+0) = ww>>1; // interpolated red pixel

	      ww = 4*(int)*(cd-pp+1) + (a + b + c + d);
	      if (ww < 0) ww = 0;
	      *(cd-pp+0) = ww>>2; // interpolated red pixel

	      s++;
	    }
	  // last two pixels
	  cd += 6;
	  md += 2;
	  s += 2;
	}

      // last two lines
      for (j=0; j<width; j+=2)
	{
	}
    }

}


// real function to do the job
// converts to monochrome

void
ImageData::convertBayerGRBGMono(uint8_t *src, uint8_t *dstm,
				int width, int height, color_conversion_t colorAlg)
{
  uint8_t *s;
  int i, j;
  int ll = width;
  int ll2 = width*2;

  s = src;
  uint8_t *md = dstm;		// monochrome

  // simple, but has "zipper" artifacts
  if (colorAlg == COLOR_CONVERSION_BILINEAR)
    {
      for (i=0; i<height; i+=2)
	{
	  // red line (GRGR...)
	  for (j=0; j<width; j+=2, md+=2)
	    {
	      *md = *s++;	// green pixel
	      s++;	// red pixel
	      if (i > 1)
		*(md-ll) = ((int)*(md) + (int)*(md-ll-1) + (int)*(md-ll+1) + (int)*(md-ll2)) >> 2;
	    }

	  // blue line (BGBG...)
	  for (j=0; j<width-2; j+=2, md+=2)
	    {
	      s++;
	      *(md+1) = *s++; // green pixel
	      if (i > 1)
		*(md-ll+1) = ((int)*(md+1) + (int)*(md-ll) + (int)*(md-ll+2) + (int)*(md-ll2+1)) >> 2;
	    }
	  // last pixels
	  s++;
	  *(md+1) = *s++;	// green pixel
	  md +=2;
	}
    }

  // EDGE color algorithm, better but slower
  else
    {
      int dc, dv, dh;

      // do first two lines
      s += ll2;

      for (i=0; i<height-4; i+=2)
	{
	  // GR line
	  // do first two pixels
	  md += 2;
	  s += 2;

	  // do most of line
	  for (j=0; j<width-4; j+=2, md+=2)
	    {
	      // green pixels
	      *md = *s++;
	      dc = 2*(int)*(s);
	      dh = dc - (int)*(s-2) - (int)*(s+2);
	      if (dh < 0) dh = -dh;
	      dv = dc - (int)*(s-ll2) - (int)*(s+ll2);
	      if (dv < 0) dv = -dv;
	      if (dv > dh) // vert is stronger, use horz
		*(md+1) = ((int)*(s-1) + (int)*(s+1))>>1;
	      else	// horz is stronger, use vert
		*(md+1) = ((int)*(s-ll) + (int)*(s+ll))>>1;

	      s++;
	    }
	  // last two pixels
	  md += 2;
	  s += 2;

	  // BG line
	  // do first two pixels
	  md += 2;
	  s += 2;

	  // do most of line
	  for (j=0; j<width-4; j+=2, md+=2)
	    {
	      dc = 2*(int)*s;
	      dh = dc - (int)*(s-2) - (int)*(s+2);
	      if (dh < 0) dh = -dh;
	      dv = dc - (int)*(s-ll2) - (int)*(s+ll2);
	      if (dv < 0) dv = -dv;
	      if (dh < dv) // vert is stronger, use horz
		*md = ((int)*(s-1) + (int)*(s+1))>>1;
	      else	// horz is stronger, use vert
		*md = ((int)*(s-ll) + (int)*(s+ll))>>1;

	      *(md+1) = *(s+1); // green pixel

	      s+=2;
	    }
	  // last two pixels
	  md += 2;
	  s += 2;
	}

      // last two lines
      for (j=0; j<width; j+=2)
	{
	}
    }

}

void
ImageData::convertBayerBGGRMono(uint8_t *src, uint8_t *dstm,
				int width, int height, color_conversion_t colorAlg)
{
  uint8_t *s;
  int i, j;
  int ll = width;
  int ll2 = width*2;

  s = src;
  uint8_t *md = dstm;		// monochrome

  // simple, but has "zipper" artifacts
  if (colorAlg == COLOR_CONVERSION_BILINEAR)
    {
      for (i=0; i<height; i+=2)
	{
          // blue line (BGBG...)
	  for (j=0; j<width-2; j+=2, md+=2)
	    {
	      s++;
	      *(md+1) = *s++; // green pixel
	      if (i > 1)
		*(md-ll+1) = ((int)*(md+1) + (int)*(md-ll) + (int)*(md-ll+2) + (int)*(md-ll2+1)) >> 2;
	    }
	  // last pixels
	  s++;
	  *(md+1) = *s++;	// green pixel
	  md +=2;
          
	  // red line (GRGR...)
	  for (j=0; j<width; j+=2, md+=2)
	    {
	      *md = *s++;	// green pixel
	      s++;	// red pixel
	      if (i > 1)
		*(md-ll) = ((int)*(md) + (int)*(md-ll-1) + (int)*(md-ll+1) + (int)*(md-ll2)) >> 2;
	    }
	}
    }

  // EDGE color algorithm, better but slower
  else
    {
      int dc, dv, dh;

      // do first two lines
      s += ll2;

      for (i=0; i<height-4; i+=2)
	{
          // BG line
	  // do first two pixels
	  md += 2;
	  s += 2;

	  // do most of line
	  for (j=0; j<width-4; j+=2, md+=2)
	    {
	      dc = 2*(int)*s;
	      dh = dc - (int)*(s-2) - (int)*(s+2);
	      if (dh < 0) dh = -dh;
	      dv = dc - (int)*(s-ll2) - (int)*(s+ll2);
	      if (dv < 0) dv = -dv;
	      if (dh < dv) // vert is stronger, use horz
		*md = ((int)*(s-1) + (int)*(s+1))>>1;
	      else	// horz is stronger, use vert
		*md = ((int)*(s-ll) + (int)*(s+ll))>>1;

	      *(md+1) = *(s+1); // green pixel

	      s+=2;
	    }
	  // last two pixels
	  md += 2;
	  s += 2;
          
	  // GR line
	  // do first two pixels
	  md += 2;
	  s += 2;

	  // do most of line
	  for (j=0; j<width-4; j+=2, md+=2)
	    {
	      // green pixels
	      *md = *s++;
	      dc = 2*(int)*(s);
	      dh = dc - (int)*(s-2) - (int)*(s+2);
	      if (dh < 0) dh = -dh;
	      dv = dc - (int)*(s-ll2) - (int)*(s+ll2);
	      if (dv < 0) dv = -dv;
	      if (dv > dh) // vert is stronger, use horz
		*(md+1) = ((int)*(s-1) + (int)*(s+1))>>1;
	      else	// horz is stronger, use vert
		*(md+1) = ((int)*(s-ll) + (int)*(s+ll))>>1;

	      s++;
	    }
	  // last two pixels
	  md += 2;
	  s += 2;
	}

      // last two lines
      for (j=0; j<width; j+=2)
	{
	}
    }

}
