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

/*
Fast Plane Fitting
Kurt Konolige
Willow Garage Inc.
email: konolige@willowgarage.com
*/


#include "image.h"


//
// fit a plane to three points
// pparams are the plane parameters in the plane equation
// returns true if p1,p2,p3 are non-collinear

#define MIN_NORMAL_LEN 0.001f	// in meters; not sure how to set this

bool
get_plane(float *pparams, pt_xyza_t *p1, pt_xyza_t *p2, pt_xyza_t *p3)
{
  float v1[3], v2[3];

  // get two vectors between points
  v1[0] = p3->X-p1->X;
  v1[1] = p3->Y-p1->Y;
  v1[2] = p3->Z-p1->Z;
  v2[0] = p2->X-p1->X;
  v2[1] = p2->Y-p1->Y;
  v2[2] = p2->Z-p1->Z;

  // cross-product for normal
  pparams[0] = v1[1]*v2[2] - v1[2]*v2[1];
  pparams[1] = v1[2]*v2[0] - v1[0]*v2[2];
  pparams[2] = v1[0]*v2[1] - v1[1]*v2[0];

  // length of normal
  float len = pparams[0]*pparams[0]+pparams[1]*pparams[1]+pparams[2]*pparams[2];
  if (len < MIN_NORMAL_LEN*MIN_NORMAL_LEN) return false;

  // normalize
  len = 1.0/sqrtf(len);
  pparams[0] *= len;
  pparams[1] *= len;
  pparams[2] *= len;
  pparams[3] = -(pparams[0]*p1->X+pparams[1]*p1->Y+pparams[2]*p1->Z); // intercept
  return true;
}


// return an index into point array
// n is size of array

inline pt_xyza_t *
rand_pt(pt_xyza_t *pts, int n)
{
  return &pts[rand()%n];
}

// return parameters of a plane for three random elements
//   that are not collinear
// n is size of array
// always assume array elements are valid

void
plane_hyp(pt_xyza_t *pts, int n, float *pparams)
{
  pt_xyza_t *pp1, *pp2, *pp3;
  bool ret;

  pp1 = rand_pt(pts,n); // first one is free
  pp2 = rand_pt(pts,n); // second one

  do
    pp2 = rand_pt(pts,n); // second one
  while (pp2 == pp1);

  while (1)
    {
      ret = false;
      pp3 = rand_pt(pts,n); // third one
      if (pp3 != pp1 && pp3 != pp2)
	ret = get_plane(pparams,pp1,pp2,pp3);
      if (ret)
	break;
    }
}


//
// find the number of inliers relative to a plane
// n is the size of the pt array
// thresh is the threshold for inliers, in meters
//

int 
num_inliers(pt_xyza_t *pts, int n, float *pparams, float thresh)
{
  int tot = 0;
  float tt = thresh*thresh;
  for (int i=0; i<n; i++, pts++)
    {
      float dist = pparams[0]*pts->X + pparams[1]*pts->Y + pparams[2]*pts->Z + pparams[3];
      if (dist*dist < tt)
	tot++;
    }
  return tot;
}  

//
// Class for finding planes in point clouds
// Assumes a vector of points of type pt_xyza_t
//   The "a" value is 0 if the point is invalid
//   The "a" value can be filled in with a plane index
//

using namespace cam;

// plane finding setup

FindPlanes::FindPlanes()
{
  n_pts3d = 0;
  n_pts3d_dec = 0;
  pts3d = NULL;
  pts3d_dec = NULL;
}


FindPlanes::~FindPlanes()
{
  if (pts3d) MEMFREE(pts3d);
  if (pts3d_dec) MEMFREE(pts3d_dec);
}


//
// point cloud setup
//

void
FindPlanes::SetPointCloud(pt_xyza_t *pts, int n, int skip)
{
  // set points
  pts3d = pts;
  n_pts3d = n;

  // check buffer
  if (n > n_pts3d_dec)
    {
      if (pts3d_dec) MEMFREE(pts3d_dec);
      pts3d_dec = (pt_xyza_t *)MEMALIGN(n*sizeof(pt_xyza_t));
    }
  
  // set up buffer
  n_pts3d_dec = 0;
  pt_xyza_t *dpts = pts3d_dec;
  for (int i=0; i<n; i+=skip, pts+=skip)
    {
      if (pts->A == 0)		// have a valid point
	{
	  dpts->X = pts->X;
	  dpts->Y = pts->Y;
	  dpts->Z = pts->Z;
	  dpts->A = 0;
	  dpts++;
	  n_pts3d_dec++;
	}
    }
}



//
// main function
// returns plane parameters and number of inliers (decimated)
//

int
FindPlanes::FindPlane(float *pparams, float thresh, int tries)
{
  int num = 0;			// number of inliers
  float params[4];

  if (n_pts3d_dec < 10) return 0;

  for (int i=0; i<tries; i++)
    {
      plane_hyp(pts3d_dec, n_pts3d_dec, params);
      int n = num_inliers(pts3d_dec, n_pts3d_dec, params, thresh);
      if (n > num)		// check if more inliers
	{
	  num = n;
	  for (int j=0; j<4; j++) pparams[j] = params[j];
	}
    }
  return num;
}


//
// index a plane that has been found
//

int
FindPlanes::IndexPlane(int ind, float thresh, float *pparams)
{
  int tot = 0;
  float tt = thresh*thresh;
  pt_xyza_t *pts = pts3d;
  if (!pts) return 0;

  for (int i=0; i<n_pts3d; i++, pts++)
    {
      if (pts->A != 0) continue; // either bad point, or already indexed
      float dist = pparams[0]*pts->X + pparams[1]*pts->Y + pparams[2]*pts->Z + pparams[3];
      if (dist*dist < tt)	// inlier
	{
	  pts->A = ind;
	  tot++;
	}
    }
  return tot;
}
