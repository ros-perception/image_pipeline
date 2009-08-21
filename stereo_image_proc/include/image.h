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

#ifndef IMAGE_H
#define IMAGE_H

#include <stdlib.h>

#ifdef __APPLE__
#include <malloc/malloc.h>
#else
#include <malloc.h>
#endif

#include <stdarg.h>
#include <math.h>
#include <ctype.h>

#ifdef WIN32
#pragma warning(disable:4996)	// get rid of POSIX deprecation errors
#include <time.h>
#include "pstdint.h"		// MSVC++ doesn't have stdint.h
#else
#include <sys/time.h>
#include <stdint.h>
#endif

#include <cv.h>
#include <cxmisc.h>
#include <cvaux.h>
#include "stereolib.h"


// version of parameter files
#define OST_MAJORVERSION 5
#define OST_MINORVERSION 0


// alignment on allocation
#ifdef __APPLE__
#define MEMALIGN(x) malloc(x)
#define MEMFREE(x) {if (x) free(x);}
#else
#define MEMALIGN(x) memalign(16,x)
#define MEMFREE(x) {if (x) free(x);}
#endif


#ifndef COLOR_CODING_T
typedef enum {
  COLOR_CODING_MONO8 = 3000,
  COLOR_CODING_MONO16,
  COLOR_CODING_BAYER8_RGGB,
  COLOR_CODING_BAYER8_BGGR,
  COLOR_CODING_BAYER8_GBRG,
  COLOR_CODING_BAYER8_GRBG,
  COLOR_CODING_BAYER16_RGGB,
  COLOR_CODING_BAYER16_BGGR,
  COLOR_CODING_BAYER16_GBRG,
  COLOR_CODING_BAYER16_GRBG,
  COLOR_CODING_RGB8,		// RGB order
  COLOR_CODING_RGBA8,		// RGBA order
  COLOR_CODING_RGB16,		// RGB order
  COLOR_CODING_RGBA16,		// RGBA order

  // these are stereo interlace encodings
  // Videre stereo:
  //   Mono has left/right pixels interlaced
  //   Color has left/right pixels interlace, bayer pixels
  //   STOC modes have rectified images, raw encodings, disparity, etc
  VIDERE_STEREO_MONO,
  VIDERE_STEREO_RGGB,
  VIDERE_STEREO_GRBG,
  VIDERE_STEREO_BGGR,
  VIDERE_STOC_RECT_RECT,	// left and right rectified mono
  VIDERE_STOC_RECT_DISP,	// left rectified mono, right disparity
  VIDERE_STOC_RAW_DISP_MONO,	// left raw mono, right disparity
  VIDERE_STOC_RAW_DISP_RGGB,	// left raw color, right disparity
  VIDERE_STOC_RAW_DISP_GRBG,	// left raw color, right disparity
  VIDERE_STOC_RAW_RAW_MONO,	// left and right raw, mono
  VIDERE_STOC_RAW_RAW_RGGB,	// left and right raw, color
  VIDERE_STOC_RAW_RAW_GRBG,	// left and right raw, color

  COLOR_CODING_NONE		// no image info
} color_coding_t;
#define COLOR_CODING_T
#endif


#ifndef COLOR_CONVERSION_T
typedef enum {
  COLOR_CONVERSION_BILINEAR,
  COLOR_CONVERSION_EDGE
} color_conversion_t;
#define COLOR_CONVERSION_T
#endif


typedef enum
{
  NORMAL_ALGORITHM,
  SCANLINE_ALGORITHM,
  DP_ALGORITHM,
  MW_ALGORITHM,
  LS_ALGORITHM,
  NCC_ALGORITHM
} stereo_algorithm_t;


//
// structured points in a 4xN point array
//

typedef struct
{
  float X;
  float Y;
  float Z;
  int32_t A;			// negative for undefined point, otherwise arbitrary index
} pt_xyza_t;


namespace cam
{

  class StereoData;		// forward reference

  // monocular data structure
  // generally, all images should be on 16-byte alignment

  class ImageData
  {
    friend class StereoData;

  public:
    ImageData();
    ~ImageData();

    // image parameters
    int imWidth;
    int imHeight;

    // image data
    // these can be NULL if no data is present
    // the Type info is COLOR_CODING_NONE if the data is not current
    // the Size info gives the buffer size, for allocation logic
    // NOTE: all data buffers should be 16-byte aligned
    // @todo: can we just use IplImages for these...
    uint8_t *imRaw;		// raw image
    color_coding_t imRawType;	// type of raw data
    size_t imRawSize;
    uint8_t *im;		// monochrome image
    color_coding_t imType;
    size_t imSize;
    uint8_t *imColor;		// color image, always RGB32
    color_coding_t imColorType;
    size_t imColorSize;
    uint8_t *imRect;		// rectified monochrome image
    color_coding_t imRectType;
    size_t imRectSize;
    uint8_t *imRectColor;	// rectified color image, always RGB32
    color_coding_t imRectColorType;
    size_t imRectColorSize;

    // timing
    uint64_t im_time;		// us time when the frame finished DMA into the host

    // calibration parameters
    // row major order
    double D[5];		// distortion: k1, k2, t1, t2, k3
    double K[9];		// original camera matrix
    double R[9];		// rectification matrix
    double P[12];		// projection/camera matrix

    // raw parameter string
    char *params;		// on-camera parameters

    // buffers
    void releaseBuffers();	// get rid of all buffers

    // rectification
    bool hasRectification;	// true if valid rectification present
    bool doRectify();		// try to rectify images
    bool initRectify(bool force=false);	// initializes the rectification internals from the
                                //   calibration parameters

    // color conversion
    color_conversion_t colorConvertType; // BILINEAR or EDGE conversion
    void doBayerColorRGB();	// does Bayer => color and mono
    void doBayerMono();		// does Bayer => mono


  protected:
    // rectification arrays from OpenCV
    bool initRect;		// whether arrays are initialized or not
    CvMat *rK;
    CvMat *rD;
    CvMat *rR;
    CvMat *rKp;

    CvMat* rMapxy;		// rectification table, integer format
    CvMat* rMapa;
    CvMat* mx,* my;
    IplImage* srcIm;		// temps for rectification
    IplImage* dstIm;

  private:
    // various color converters
    void convertBayerGRBGColorRGB(uint8_t *src, uint8_t *dstc, uint8_t *dstm,
				  int width, int height, color_conversion_t colorAlg);
    void convertBayerBGGRColorRGB(uint8_t *src, uint8_t *dstc, uint8_t *dstm,
				  int width, int height, color_conversion_t colorAlg);
    void convertBayerGRBGMono(uint8_t *src, uint8_t *dstm,
                              int width, int height, color_conversion_t colorAlg);
    void convertBayerBGGRMono(uint8_t *src, uint8_t *dstm,
                              int width, int height, color_conversion_t colorAlg);
  };


  // stereo data structure

  class StereoData
  {
  public:
    StereoData();
    ~StereoData();

    // image parameters
    int imWidth;
    int imHeight;
    void setSize(int width, int height); // sets individual image sizes too

    // left and right image data
    ImageData *imLeft;
    ImageData *imRight;

    // rectification
    bool hasRectification;

    // disparity data
    int16_t *imDisp;		// disparity image, negative and zero are invalid pixels
    size_t imDispSize;		// size of image in bytes
    int dpp;			// disparity units per pixel, e.g., 16 is 1/16 pixel per disparity
    bool hasDisparity;		// true if disparity present
    int numDisp;		// number of search disparities, in pixels
    int offx;			// x offset of disparity search

    bool setHoropter(int offset); // set horopter offset
    bool setNumDisp(int ndisp);	// set number of disparities

    // Color conversion:
    void doBayerColorRGB();
    void doBayerMono();

    // disparity and rectification functions
    bool doRectify();		// rectify images
    bool doDisparity(stereo_algorithm_t alg=NORMAL_ALGORITHM); // calculate disparity image
    bool doSpeckle();		// speckle filter post-processing, automatically applied by doDisparity
    bool doCalcPts(bool isArray = false); // calculate 3D points
    bool calcPt(int x, int y, float *fx, float *fy, float *fz); // single point
    bool setRangeMax(double thresh);
    bool setRangeMin(double thresh);

    // valid stereo data rectangle
    int imDtop, imDleft;
    int imDwidth, imDheight;
    void setDispOffsets();	// reset them, based on stereo processing params

    // point cloud data
    // NOTE: imPts buffer should be 16-byte aligned
    // imPts elements will have the form of a pt_xyza_t for a pt array
    float *imPts;		// points, 3xN floats for vector version, 4xN for array version
                                // for isPtArray = true, these next two are not needed
    int *imCoords;		// image coordinates of the point cloud points, 2xN ints
    uint8_t *imPtsColor;	// color vector corresponding to points, RGB
    size_t imPtsSize;		// size of array in bytes, for storage manipulation
    int numPts;			// number of points in array
    bool isPtArray;		// true if the points are an image array, z=0.0 for no point
    pt_xyza_t *imPtArray() { return (pt_xyza_t *)imPts; } 

    // external parameters for undistorted images
    double T[3];		// pose of right camera in left camera coords
    double Om[3];		// rotation vector

    // reprojection matrix
    double RP[16];

    // buffers
    void releaseBuffers();	// get rid of all buffers

    // parameters
    void extractParams(char *params, bool store = false); // extracts params from string and puts in vars
                                // optionally stores into image object
    char *createParams(bool store = false); // takes parameters and puts them into a string
                                // optionally stores into image object

    // stereo processing params
    int corrSize;		// correlation window size, assumed square
    int filterSize;		// size of prefilter window, assumed square (0 if none)
    int horOffset;		// horopter offset

    // filter thresholds
    int textureThresh;		// percent
    int uniqueThresh;		// percent
    int smoothThresh;		// percent
    int speckleDiff;		// max difference between adjacent disparities in a region
    int speckleRegionSize;	// minimum size of region to be not a speckle
    double rangeMax;		// max Z value returned in pt cloud
    double rangeMin;		// max Z value returned in pt cloud
    bool unique_check;

    bool setTextureThresh(int thresh);
    bool setUniqueThresh(int thresh);
    bool setSmoothnessThresh(int thresh);
    bool setSpeckleDiff(int diff);
    bool setSpeckleRegionSize(int size);
    bool setCorrSize(int size);
    bool setUniqueCheck(bool val);

    // buffers for stereo
    uint8_t *buf, *flim, *frim;
    int maxxim, maxyim, maxdlen, maxcorr; // for changing buffer sizes

  private:
    // buffers for speckle filter
    uint8_t *rbuf;
    uint32_t *lbuf, *wbuf;

  };


  //
  // Plane finding class
  //

  class FindPlanes
  {
  public:
    FindPlanes();
    ~FindPlanes();

    // put points
    // skip interval is decimation for speed
    void SetPointCloud(pt_xyza_t *pts, int n, int skip=1);

    // find a plane, return its parameters and (decimated) inlier count
    int FindPlane(float *pparams, float thresh, int tries);

    // set all plane inliers to an index, return number found
    // also resets decimated point cloud
    int IndexPlane(int ind, float thresh, float *pparams);

  private:
    pt_xyza_t *pts3d;		// input vector of points
    int n_pts3d;		// number of points
    pt_xyza_t *pts3d_dec;	// decimated points
    int n_pts3d_dec;
  };

}


#endif	// IMAGE_H
