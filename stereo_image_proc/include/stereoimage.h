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

#ifndef STEREOIMAGE_H
#define STEREOIMAGE_H

#include "image.h"
#include "stereolib.h"

// version of parameter files
#define OST_MAJORVERSION 5
#define OST_MINORVERSION 0

namespace cam
{
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


#endif	// STEREOIMAGE_H
