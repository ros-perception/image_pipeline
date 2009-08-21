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
Stereo Lib "2" for Open Stereo package
Federico Tombari
Willow Garage Inc.
email: tombari@willowgarage.com
CVLab - University of Bologna
email: federico.tombari@unibo.it
*/

#include "stereolib.h"
#define inline			// use this for Intel Compiler Debug mode
#include <stdio.h>

inline int min3(int a, int b, int c){
return (a<b) ? ((a<c)?a:c) : ((b<c)?b:c);
}

inline int min4(int a, int b, int c, int d){

int t1 = (a<b)?a:b;
int t2 = (c<d)?c:d;
return (t1<t2)?t1:t2;

}

inline int min(int a, int b){
return (a<b)?a:b;
}

//Application of the uniqueness constraint
void uniqueness_constraint_reducedrange(int w, short int* disp, int maxdisp, int r, int* mins, int row){

int x, d, x_oth, d_oth;

//Init array
int* unique = (int *)malloc(w*sizeof(int));
for(x=r; x<w-r; x++)
	unique[x] = -1;
	
for(x=maxdisp+r+1; x<w-r; x++){
	d = disp[row*w + x]/16;

	//UNIQUENESS CONSTRAINT
	if(d > FILTERED){		//this is needed since previous filters may have already discarded this correspondence
		if(unique[x-d] == -1)
			unique[x-d] = d;
		else{
			d_oth = unique[x-d];	
			x_oth = x - d + d_oth;
			if(mins[x_oth] > mins[x]){
 				unique[x-d] = d;
 				disp[row*w + x_oth] = FILTERED;
 			}
			else{
				disp[row*w + x] = FILTERED;
 			}
		}
	}
}//y

free(unique);
}

//Scanline-optimization along horizontal directions
void do_stereo_so(uint8_t *lim, uint8_t *rim, // input feature images
	  int16_t *disp,	// disparity output
	  int xim, int yim,	// size of images
	  uint8_t ftzero,	// feature offset from zero
	  int xwin, int ywin,	// size of corr window, usually square
	  int dlen,		// size of disparity search, multiple of 8
	  int pfilter_thresh,	// texture filter threshold
	  int ufilter_thresh,	// uniqueness filter threshold, percent
	  int smooth_thresh,    // smoothness threshold
	  int unique_c         // uniqueness check
	)
{

xwin = xwin-4; //bad hack to deal with small window sizes (e.g. 3x3)

int maxdisp = dlen;
int r = (xwin-1)/2;
int n = xwin;
//note: ywim is currently deprecated

int w = xim;
int h = yim;

unsigned char *L = lim;
unsigned char *R = rim;
	
//Parameters for regularization
int pi2 = smooth_thresh*4*n;
int pi1 = smooth_thresh*n;

//temp variables
int x,y,d,i,j;
int dbest=0;
int c_min, cost;

//FILTERS
//int *min_scores = (int *)malloc(w * sizeof(int ));
int ratio_filter = 100-ufilter_thresh;
int peak_filter = pfilter_thresh*n;

int sad_second_min;
int da, db, s1, s2, s3;

//data structured for Scanline Optimization
int **F = (int **)calloc(w, sizeof(int *));
int **B = (int **)calloc(w, sizeof(int *));
for(x=0; x<w; x++){
	F[x] = (int *)calloc(maxdisp, sizeof(int));
	B[x] = (int *)calloc(maxdisp, sizeof(int));
}

//BEGIN INCREMENTAL COMPUTATION FOR RADIUS >0
int **acc = (int **)calloc(w, sizeof(int *));
int **V = (int **)calloc(w, sizeof(int*));
for(d=0; d<w; d++){
	V[d] = (int *)calloc(maxdisp, sizeof(int)); 
	acc[d] = (int *)calloc(maxdisp, sizeof(int)); 	
}

//FIRST ROW
//STAGE 1 - init acc
for(d=0; d<maxdisp; d++){
	for(i=maxdisp; i<maxdisp+n; i++){
		for(j=0; j<n; j++)
			V[i][d] += abs( L[j*w+i] - R[j*w+i-d] );
		//acc[d] += V[i][d];			
	}		
}

//STAGE 2: other positions
for(x=maxdisp+r+1; x<w-r; x++){
	for(d=0; d<maxdisp; d++){
		for(j=0; j<n; j++)
			V[x+r][d] += abs( L[j*w + x+r] - R[ j*w + x+r-d] );
		//acc[d] = acc[d] + V[x+r][d] - V[x-r-1][d];
	}//d
}//x
unsigned char *lp, *rp, *lpp, *rpp;
int ind1 = r+r+maxdisp;
//END INCREMENTAL COMPUTATION FOR RADIUS >0

for(y=r+1; y<h-r; y++){

	//first position
	for(d=0; d<maxdisp; d++){
		acc[maxdisp+r][d] = 0;
		for(i=maxdisp; i<maxdisp+n; i++){
			V[i][d] = V[i][d] + abs( L[ (y+r)*w+i] - R[ (y+r)*w+i-d] ) - abs( L[ (y-r-1)*w+i] - R[ (y-r-1)*w+i-d] );
			acc[maxdisp+r][d] += V[i][d];			
		}	
	}
		
	//compute ACC for all other positions
	lp = (unsigned char *) L + (y+r)*w + ind1;
	rp =  (unsigned char *) R + (y+r)*w + ind1;
	lpp = (unsigned char *) L + (y-r-1)*w + ind1;
	rpp = (unsigned char *) R + (y-r-1)*w + ind1;
		
	for(x=maxdisp+r+1; x<w-r; x++){
			
		lp++;
		rp++;
		lpp++;
		rpp++;
				
		//int dbestprev = dbest;
		for(d=0; d<maxdisp; d++){
				
			V[x+r][d] = V[x+r][d] + abs( *lp - *rp ) - abs( *lpp - *rpp ); 
			rp--;
			rpp--;
			acc[x][d] = acc[x-1][d] + V[x+r][d] - V[x-r-1][d];
		}//d
		rp += maxdisp;
		rpp += maxdisp;
	}//x to compute ACC
		
	//FORWARD
	//Border
	for(d=0; d<maxdisp; d++)
		F[maxdisp+r][d] = acc[maxdisp+r][d];

	for(x=maxdisp+r; x<w-r; x++){

		c_min = F[x-1][0];
		dbest = 0;
		for(d=1; d<maxdisp; d++)
		if(F[x-1][d] < c_min){
			c_min = F[x-1][d];
			dbest = d;
		}

		F[x][0] =  acc[x][0] - c_min + min3(F[x-1][0], F[x-1][1]+pi1, c_min+pi2);
		for(d=1; d<maxdisp-1; d++){
			F[x][d] = acc[x][d] - c_min + min4(F[x-1][d], F[x-1][d-1]+pi1, F[x-1][d+1]+pi1, c_min+pi2);
		} //d
		F[x][maxdisp-1] = acc[x][maxdisp-1] - c_min + min3(F[x-1][maxdisp-1], F[x-1][maxdisp-2]+pi1, c_min+pi2);
	}//x1

	//BACKWARD
	//Border	
	for(d=0; d<maxdisp; d++)
		B[w-1-r][d] = acc[w-1-r][d];
	
	for(x=w-2-r; x>=maxdisp+r; x--){

		c_min = B[x+1][0];
		dbest = 0;
		for(d=1; d<maxdisp; d++)
		if(B[x+1][d] < c_min){
			c_min = B[x+1][d];
			dbest = d;
		}

 		B[x][0] =  acc[x][0] - c_min + min3(B[x+1][0], B[x+1][1]+pi1, c_min+pi2);
 		for(d=1; d<maxdisp-1; d++){
 			B[x][d] = acc[x][d] - c_min + min4(B[x+1][d], B[x+1][d-1]+pi1, B[x+1][d+1]+pi1, c_min+pi2);
 		} //d
 		B[x][maxdisp-1] = acc[x][maxdisp-1] - c_min + min3(B[x+1][maxdisp-1], B[x+1][maxdisp-2]+pi1, c_min+pi2);
	}//x1

	for(x=maxdisp+r; x<w-r; x++){
		c_min = 32000;

		for(d=0; d<maxdisp; d++){
			cost = F[x][d] + B[x][d];
			if(cost < c_min){
				c_min = cost;
				dbest = d;
			}
		}
		disp[y*w+x] = dbest;	
		
		//******* FILTERS ********
		//( 2)needed for uniqueness constraint filter )
		//min_scores[x] = c_min;
	
		//3) uniqueness filter
		sad_second_min = 32000;
		for(d=0; d<dbest-1; d++){
			cost = F[x][d] + B[x][d];
			if(cost<sad_second_min){
				sad_second_min = cost;
			}
		}
		for(d=dbest+2; d<maxdisp; d++){
			cost = F[x][d] + B[x][d];
			if(cost<sad_second_min){
				sad_second_min = cost;
			}
		}	
		if( c_min*100  > ratio_filter*sad_second_min)
			disp[y*w + x] = FILTERED;
	
		//4) Peak Filter
		s1 = F[x][dbest-1] + B[x][dbest-1];
		s2 = F[x][dbest] + B[x][dbest];
		s3 = F[x][dbest+1] + B[x][dbest+1];
		da = (dbest>1) ? ( s1 - s2 ) : (s3 - s2);
		db =  (dbest<maxdisp-2) ? (s3 - s2) : (s1 - s2);		
		if(da + db < peak_filter)
			disp[y*w + x] = FILTERED;
		//******* FILTERS ********
	
		//subpixel refinement
		if(disp[y*w + x] != FILTERED){
			double v = dbest + ( (s1-s3)/(2.0*(s1+s3-2.0*s2)) );
			disp[y*w + x] = (int)(0.5 + 16*v);
		}
	}
	//if(unique_c)
	//	uniqueness_constraint_reducedrange(w, disp, maxdisp, r, min_scores, y);
}

for(x=0; x<w; x++){
	free(acc[x]);
	free(F[x]);
	free(B[x]);
	free(V[x]);
}

free(F);
free(B);
//free(diff);
free(acc);

//free(min_scores);

}


//Scanline-optimization along horizontal directions
void do_stereo_mw(uint8_t *lim, uint8_t *rim, // input feature images
	  int16_t *disp,	// disparity output
	  int xim, int yim,	// size of images
	  uint8_t ftzero,	// feature offset from zero
	  int xwin, int ywin,	// size of corr window, usually square
	  int dlen,		// size of disparity search, multiple of 8
	  int pfilter_thresh,	// peak filter threshold
	  int ufilter_thresh,	// uniqueness filter threshold, percent
	  int unique_c         // uniqueness check
	  )
{

	int maxdisp = dlen;
	int r = (xwin-1)/2;
	int n = xwin;
	//note: ywim is currently deprecated
	
	int w = xim;
	int h = yim;
	
	unsigned char *L = lim;
	unsigned char *R = rim;
	
	int x,y,i,j,d;
	int sqn = n*n;

	int **acc = (int **)calloc(w, sizeof(int *));
	int **V = (int **)calloc(w, sizeof(int *));

	for(d=0; d<w; d++){
		V[d] = (int *)calloc(maxdisp, sizeof(int)); 
		acc[d] = (int *)calloc(maxdisp, sizeof(int)); 
	}

	int sad_min=0, dbest=0, temp, cost;
	int sad_max = 255 * sqn;

	//FILTERS
	//int *min_scores = (int *)malloc(w * sizeof(int ));
	
	int sad_second_min;
	int da, db;
	int ratio_filter = 100-ufilter_thresh;
	int peak_filter = pfilter_thresh;

	//INIT - FIRST ROW
	//STAGE 1 - init acc
	for(d=0; d<maxdisp; d++){
		for(i=maxdisp; i<maxdisp+n; i++){
			for(j=0; j<n; j++)
				V[i][d] += abs( L[j*w+i] - R[j*w+i-d] );
			acc[maxdisp+r][d] += V[i][d];			
		}		
	}

	//STAGE 2: other positions
	for(x=maxdisp+r+1; x<w-r; x++){
		for(d=0; d<maxdisp; d++){
			for(j=0; j<n; j++)
				V[x+r][d] += abs( L[j*w + x+r] - R[ j*w + x+r-d] );
			acc[x][d] = acc[x-1][d] + V[x+r][d] - V[x-r-1][d];
		}//d
	}//x

	unsigned char *lp, *rp, *lpp, *rpp;
	int ind1 = r+r+maxdisp;
	//OTHER ROWS
	for(y=r+1; y<h-r; y++){
		
		//first position
		for(d=0; d<maxdisp; d++){
			acc[maxdisp+r][d] = 0;
			for(i=maxdisp; i<maxdisp+n; i++){
				V[i][d] = V[i][d] + abs( L[ (y+r)*w+i] - R[ (y+r)*w+i-d] ) - abs( L[ (y-r-1)*w+i] - R[ (y-r-1)*w+i-d] );
				acc[maxdisp+r][d] += V[i][d];			
			}	
		}


		//other positions
		lp = (unsigned char *) L + (y+r)*w + ind1;
		rp =  (unsigned char *) R + (y+r)*w + ind1;
		lpp = (unsigned char *) L + (y-r-1)*w + ind1;
		rpp = (unsigned char *) R + (y-r-1)*w + ind1;

		for(x=maxdisp+r+1; x<w-r; x++){
			lp++;
			rp++;
			lpp++;
			rpp++;

			for(d=0; d<maxdisp; d++){
				V[x+r][d] = V[x+r][d] + abs( *lp - *rp ) - abs( *lpp - *rpp ); 
				rp--;
				rpp--;
				acc[x][d] = acc[x-1][d] + V[x+r][d] - V[x-r-1][d];				
			}//d
			
			rp += maxdisp;
			rpp += maxdisp;
		}	
		sad_second_min = sad_max;
		for(d=0; d<maxdisp; d++){
			if(d != dbest && acc[x][d]<sad_second_min){
				sad_second_min = acc[x][d];
			}
		}	
		if( sad_min*100  > ratio_filter*sad_second_min)
			disp[y*w + x] = FILTERED;

		//only right term exists
		for(x=maxdisp+r; x<maxdisp+n; x++){
			sad_min = sad_max;
			for(d=0; d<maxdisp; d++){
				temp = min(acc[x][d],acc[x+r][d]);
				if( temp < sad_min){
					sad_min = temp;
					dbest = d;
				}
			}
			disp[y*w + x] = dbest;
			
			//min_scores[x] = sad_min;
			// 3) uniqueness filter
			sad_second_min = 32000;
			for(d=0; d<dbest-1; d++){
				cost = acc[x][d];
				if(cost<sad_second_min){
					sad_second_min = cost;
				}
			}
			for(d=dbest+2; d<maxdisp; d++){
				cost = acc[x][d];
				if(cost<sad_second_min){
					sad_second_min = cost;
				}
			}	
			if( sad_min*100  > ratio_filter*sad_second_min)
				disp[y*w + x] = FILTERED;			

		
			//4) Peak Filter
			da = (dbest>1) ? ( acc[x][dbest-2] - acc[x][dbest] ) : (acc[x][dbest+2] - acc[x][dbest]);
			db =  (dbest<maxdisp-2) ? (acc[x][dbest+2] - acc[x][dbest]) : (acc[x][dbest-2] - acc[x][dbest]);		
			if(da + db < peak_filter)
				disp[y*w + x] = FILTERED;
			//******* FILTERS ********
			
			//subpixel refinement
			if(disp[y*w + x] != FILTERED){
				int s1 = acc[x][dbest-1];
				int s2 = acc[x][dbest];
				int s3 = acc[x][dbest+1];
				double v = dbest + ( (s1-s3)/(2.0*(s1+s3-2.0*s2)) );
				disp[y*w + x] = (int)(0.5 + 16*v);
			}
		}
		
		//both terms exist 
		for(x=maxdisp+n; x<w-n; x++){
			sad_min = sad_max;
			for(d=0; d<maxdisp; d++){
				temp = min3(acc[x][d], acc[x-r][d], acc[x+r][d]);
				if( temp < sad_min){
					sad_min = temp;
					dbest = d;
				}
			}
			disp[y*w + x] = dbest;
			
			//******* FILTERS ********
			//min_scores[x] = sad_min;
			// 3) uniqueness filter
			sad_second_min = 32000;
			for(d=0; d<dbest-1; d++){
				cost = acc[x][d];
				if(cost<sad_second_min){
					sad_second_min = cost;
				}
			}
			for(d=dbest+2; d<maxdisp; d++){
				cost = acc[x][d];
				if(cost<sad_second_min){
					sad_second_min = cost;
				}
			}	
			if( sad_min*100  > ratio_filter*sad_second_min)
				disp[y*w + x] = FILTERED;			
		
			//4) Peak Filter
			da = (dbest>1) ? ( acc[x][dbest-2] - acc[x][dbest] ) : (acc[x][dbest+2] - acc[x][dbest]);
			db =  (dbest<maxdisp-2) ? (acc[x][dbest+2] - acc[x][dbest]) : (acc[x][dbest-2] - acc[x][dbest]);		
			if(da + db < peak_filter)
				disp[y*w + x] = FILTERED;
			//******* FILTERS ********
			//#endif
			
			//subpixel refinement
			if(disp[y*w + x] != FILTERED){
				int s1 = acc[x][dbest-1];
				int s2 = acc[x][dbest];
				int s3 = acc[x][dbest+1];
				double v = dbest + ( (s1-s3)/(2.0*(s1+s3-2.0*s2)) );
				disp[y*w + x] = (int)(0.5 + 16*v);
			}
		}//x
		
		//only left term exists
		for(x=w-n; x<w-r; x++){
			sad_min = sad_max;
			for(d=0; d<maxdisp; d++){
				temp = min(acc[x][d],acc[x-r][d]);
				if( temp < sad_min){
					sad_min = temp;
					dbest = d;
				}
			}
			disp[y*w + x] = dbest;
			//min_scores[x] = sad_min;
			// 3) uniqueness filter
			sad_second_min = 32000;
			for(d=0; d<dbest-1; d++){
				cost = acc[x][d];
				if(cost<sad_second_min){
					sad_second_min = cost;
				}
			}
			for(d=dbest+2; d<maxdisp; d++){
				cost = acc[x][d];
				if(cost<sad_second_min){
					sad_second_min = cost;
				}
			}	
			if( sad_min*100  > ratio_filter*sad_second_min)
				disp[y*w + x] = FILTERED;			

			//4) Peak Filter
			da = (dbest>1) ? ( acc[x][dbest-2] - acc[x][dbest] ) : (acc[x][dbest+2] - acc[x][dbest]);
			db =  (dbest<maxdisp-2) ? (acc[x][dbest+2] - acc[x][dbest]) : (acc[x][dbest-2] - acc[x][dbest]);		
			if(da + db < peak_filter)
				disp[y*w + x] = FILTERED;
			//******* FILTERS ********
		
			//subpixel refinement
			if(disp[y*w + x] != FILTERED){
				int s1 = acc[x][dbest-1];
				int s2 = acc[x][dbest];
				int s3 = acc[x][dbest+1];
				double v = dbest + ( (s1-s3)/(2.0*(s1+s3-2.0*s2)) );
				disp[y*w + x] = (int)(0.5 + 16*v);
			}
		}
		//if(unique_c == 1)
		//	uniqueness_constraint_reducedrange(w, disp, maxdisp, r, min_scores, y);
	}//y

	for(d=0; d<w;d++){
		free(V[d]);
		free(acc[d]);
	}
	free(V);
	free(acc);

	//free(min_scores);
}

//Scanline-optimization along horizontal directions
void do_stereo_dp(uint8_t *lim, uint8_t *rim, // input feature images
	  int16_t *disp,	// disparity output
	  int xim, int yim,	// size of images
	  uint8_t ftzero,	// feature offset from zero
	  int xwin, int ywin,	// size of corr window, usually square
	  int dlen,		// size of disparity search, multiple of 8
	  int pfilter_thresh,	// texture filter threshold
	  int ufilter_thresh,	// uniqueness filter threshold, percent
	  int smooth_thresh,	//smoothness penalty term			
          int unique_c         // uniqueness check
	)
{


xwin = xwin-4; //bad hack to deal with small window sizes (e.g. 3x3)

int maxdisp = dlen;
int r = (xwin-1)/2;
int n = xwin;
//note: ywim is currently deprecated

int w = xim;
int h = yim;

unsigned char *L = lim;
unsigned char *R = rim;

//Parameters for regularization
int w2 = smooth_thresh*n*4;
int w1 = smooth_thresh*n;

//temp variables
int x,y,d,i,j;
int dbest=0;
int c_min, cost;

//FILTERS
//int *min_scores = (int *)malloc(w * sizeof(int ));

//int ratio_filter = 100-ufilter_thresh;
int peak_filter = pfilter_thresh;

//int sad_second_min;
int da, db, s1, s2, s3;

//data structured for Dynamic Programming
int **S = (int **)calloc(w, sizeof(int *));
int **B = (int **)calloc(w, sizeof(int *));
for(x=0; x<w; x++){
	S[x] = (int *)calloc(maxdisp, sizeof(int));
	B[x] = (int *)calloc(maxdisp, sizeof(int));
}

//BEGIN INCREMENTAL COMPUTATION FOR RADIUS >0
int **acc = (int **)calloc(w, sizeof(int *));
int **V = (int **)calloc(w, sizeof(int*));
for(d=0; d<w; d++){
	V[d] = (int *)calloc(maxdisp, sizeof(int)); 
	acc[d] = (int *)calloc(maxdisp, sizeof(int)); 	
}

//FIRST ROW
//STAGE 1 - init acc
for(d=0; d<maxdisp; d++){
	for(i=maxdisp; i<maxdisp+n; i++){
		for(j=0; j<n; j++)
			V[i][d] += abs( L[j*w+i] - R[j*w+i-d] );
		//acc[d] += V[i][d];			
	}		
}

//STAGE 2: other positions
for(x=maxdisp+r+1; x<w-r; x++){
	for(d=0; d<maxdisp; d++){
		for(j=0; j<n; j++)
			V[x+r][d] += abs( L[j*w + x+r] - R[ j*w + x+r-d] );
		//acc[d] = acc[d] + V[x+r][d] - V[x-r-1][d];
	}//d
}//x
unsigned char *lp, *rp, *lpp, *rpp;
int ind1 = r+r+maxdisp;
//END INCREMENTAL COMPUTATION FOR RADIUS >0

for(y=r+1; y<h-r; y++){

	//first position
	for(d=0; d<maxdisp; d++){
		acc[maxdisp+r][d] = 0;
		for(i=maxdisp; i<maxdisp+n; i++){
			V[i][d] = V[i][d] + abs( L[ (y+r)*w+i] - R[ (y+r)*w+i-d] ) - abs( L[ (y-r-1)*w+i] - R[ (y-r-1)*w+i-d] );
			acc[maxdisp+r][d] += V[i][d];			
		}	
	}
		
	//compute ACC for all other positions
	lp = (unsigned char *) L + (y+r)*w + ind1;
	rp =  (unsigned char *) R + (y+r)*w + ind1;
	lpp = (unsigned char *) L + (y-r-1)*w + ind1;
	rpp = (unsigned char *) R + (y-r-1)*w + ind1;
		
	for(x=maxdisp+r+1; x<w-r; x++){
			
		lp++;
		rp++;
		lpp++;
		rpp++;
				
		//int dbestprev = dbest;
		for(d=0; d<maxdisp; d++){	
			V[x+r][d] = V[x+r][d] + abs( *lp - *rp ) - abs( *lpp - *rpp ); 
			rp--;
			rpp--;
			acc[x][d] = acc[x-1][d] + V[x+r][d] - V[x-r-1][d];
		}//d
		rp += maxdisp;
		rpp += maxdisp;
	}//x to compute ACC

	//Stage 1 ->forward
	//Border
	for(d=0; d<maxdisp; d++)
		S[maxdisp+r][d] = acc[maxdisp+r][d];
	
	for(x=maxdisp+r+1; x<w-r; x++){
		c_min = S[x-1][0];
		dbest = 0;
		for(d=1; d<maxdisp; d++)
		if(S[x-1][d] < c_min){
			c_min = S[x-1][d];
			dbest = d;
		}
	
		S[x][0] = acc[x][0];
		if( S[x-1][0]<c_min+w2 ){
 			S[x][0] += S[x-1][0];
  			B[x][0] = d;
		}
		else{
			S[x][0] += c_min+w2;
  			B[x][0] = dbest;
		}
		S[x][maxdisp-1] = acc[x][maxdisp-1];
		if( S[x-1][maxdisp-1]<c_min+w2 ){
 			S[x][maxdisp-1] += S[x-1][maxdisp-1];
  			B[x][maxdisp-1] = d;
		}
		else{
			S[x][maxdisp-1] += c_min+w2;
  			B[x][maxdisp-1] = dbest;
		}

		for(d=1; d<maxdisp-1; d++){
			S[x][d] = acc[x][d];
			if( S[x-1][d]<c_min+w2 ){
 				if( S[x-1][d]<S[x-1][d-1]+w1 ){
 					if( S[x-1][d]<S[x-1][d+1]+w1 ){
   						S[x][d] += S[x-1][d];
   						B[x][d] = d;
 					}
 					else{
 						S[x][d] += S[x-1][d+1]+w1;
 						B[x][d] = d+1;
 					}
 				}
 				else{
 					if(S[x-1][d-1] < S[x-1][d+1]){
 						S[x][d] += S[x-1][d-1]+w1;
 						B[x][d] = d-1;
 					}
 					else{
 						S[x][d] += S[x-1][d+1]+w1;
 						B[x][d] = d+1;
 					}
 				}
 			}
 			else{
				if( c_min+w2 < S[x-1][d-1]+w1 ){
					if( c_min+w2 < S[x-1][d+1]+w1 ){
  						S[x][d] += c_min+w2;
  						B[x][d] = dbest;
					}
					else{
						S[x][d] += S[x-1][d+1]+w1;
						B[x][d] = d+1;
					}
				}
				else{
					if(S[x-1][d-1] < S[x-1][d+1]){
						S[x][d] += S[x-1][d-1]+w1;
						B[x][d] = d-1;
					}
					else{
						S[x][d] += S[x-1][d+1]+w1;
						B[x][d] = d+1;
					}
				}
 			}
 		} //d
 	}//x

	//Stage 2 -> backward
	//Border
	x = w-r-1;
	c_min = 32000;
	for(d=0; d<maxdisp; d++){
		cost = S[x][d];
		if(cost < c_min){
			c_min = cost;
			dbest = d;
		}
	}
	disp[y*w+x] = dbest;
	
	for(x=w-r-2; x>=maxdisp+r; x--){

 		dbest = B[x+1][dbest];
 		disp[y*w+x] = dbest;	
		
		//******* FILTERS ********
		//min_scores[x] = c_min;
	
		//3) uniqueness filter
// 		c_min = S[x][dbest];
// 		sad_second_min = 32000;
// 		for(d=0; d<dbest-1; d++){
// 			cost = S[x][d];
// 			if(cost<sad_second_min){
// 				sad_second_min = cost;
// 			}
// 		}
// 		for(d=dbest+2; d<maxdisp; d++){
// 			cost = S[x][d];
// 			if(cost<sad_second_min){
// 				sad_second_min = cost;
// 			}
// 		}	
// 		if( c_min*100  > ratio_filter*sad_second_min)
// 			disp[y*w + x] = FILTERED;
		
		//4) Peak Filter
		s1 = S[x][dbest-1];
		s2 = S[x][dbest];
		s3 = S[x][dbest+1];
		da = (dbest>1) ? ( s1 - s2 ) : (s3 - s2);
		db =  (dbest<maxdisp-2) ? (s3 - s2) : (s1 - s2);		
		if(da + db < peak_filter)
			disp[y*w + x] = FILTERED;
		//******* FILTERS ********
		
		//subpixel refinement
		if(disp[y*w + x] != FILTERED){
			double v = dbest + ( (s1-s3)/(2.0*(s1+s3-2.0*s2)) );
			disp[y*w + x] = (int)(0.5 + 16*v);
		}
	}
	//if(unique_c == 1)
	//	uniqueness_constraint_reducedrange(w, disp, maxdisp, r, min_scores, y);
}

for(x=0; x<w; x++){
	free(acc[x]);
	free(S[x]);
	free(B[x]);
	free(V[x]);
}

free(S);
free(B);
free(acc);

//free(min_scores);

}




inline int compute_penalty(int temp, int* acc, int pi1, int pi2, int maxdisp){

	int dbest=0;

	if(temp>0)
		acc[temp-1] -= pi2; 
	acc[temp] -= pi1;
	if(temp < maxdisp-1)
		acc[temp+1] -= pi2; 

	int sad_min = 32000;
	for(int d=0; d<maxdisp; d++){
		if(acc[d]  < sad_min){
			sad_min = acc[d];
			dbest = d;
		}
	}
	
	if(temp>0)
		acc[temp-1] += pi2; 
	acc[temp] += pi1;
	if(temp < maxdisp-1)
		acc[temp+1] += pi2; 
	
	return dbest;
}



//Local Smoothness along 4 directions
void do_stereo_ls(uint8_t *lim, uint8_t *rim, // input feature images
	  int16_t *disp,	// disparity output
	  int xim, int yim,	// size of images
	  uint8_t ftzero,	// feature offset from zero
	  int xwin, int ywin,	// size of corr window, usually square
	  int dlen,		// size of disparity search, multiple of 8
	  int pfilter_thresh,	// texture filter threshold
	  int ufilter_thresh,	// uniqueness filter threshold, percent
	  int smooth_thresh,	//smoothness penalty term			
          int unique_c         // uniqueness check
	)
{


xwin = xwin-4; //bad hack to deal with small window sizes (e.g. 3x3)

int maxdisp = dlen;
int r = (xwin-1)/2;
int n = xwin;
//note: ywim is currently deprecated

int w = xim;
int h = yim;

unsigned char *L = lim;
unsigned char *R = rim;

//Parameters for regularization
int pi1 = smooth_thresh*n*4;
int pi2 = pi1 - smooth_thresh*n;

//temp variables
int x,y,d,i,j;

//FILTERS
//int *min_scores = (int *)malloc(w * sizeof(int ));
int ratio_filter = 100-ufilter_thresh;
int peak_filter = pfilter_thresh;	
int sad_second_min;
int da, db;

int *acc = (int *)calloc(maxdisp, sizeof(int));
int **V = (int **)calloc(w, sizeof(int*));

for(d=0; d<w; d++)
	V[d] = (int *)calloc(maxdisp, sizeof(int)); 

unsigned char *bdv = (unsigned char *)calloc(w*h, sizeof(unsigned char));
unsigned char *bdh = (unsigned char *)calloc(w*h, sizeof(unsigned char));
unsigned char *bdu = (unsigned char *)calloc(w, sizeof(unsigned char));

int sad_min, dbest=0, dbestph=0, dbestpv=0;
int sad_max = 32000;

//FIRST ROW
//STAGE 1 - init acc
for(d=0; d<maxdisp; d++){
	for(i=maxdisp; i<maxdisp+n; i++){
		for(j=0; j<n; j++)
			V[i][d] += abs( L[j*w+i] - R[j*w+i-d] );
		acc[d] += V[i][d];			
	}		
}

//STAGE 2: other positions
for(x=maxdisp+r+1; x<w-r; x++){
	sad_min = sad_max;
	for(d=0; d<maxdisp; d++){
		for(j=0; j<n; j++)
			V[x+r][d] += abs( L[j*w + x+r] - R[ j*w + x+r-d] );
		acc[d] = acc[d] + V[x+r][d] - V[x-r-1][d];
		
		if(acc[d]  < sad_min){
			sad_min = acc[d];
			dbestpv = d;
		}
	}//d
	bdv[r*w+x] = dbestpv;
}//x

unsigned char *lp, *rp, *lpp, *rpp;
int ind1 = r+r+maxdisp;
int temp;

	//BEGIN 1st PASS - TOPLEFT 2 BOTTOMRIGHT
	//OTHER ROWS
	for(y=r+1; y<h-r; y++){
		
		sad_min = sad_max;
		//first position
		for(d=0; d<maxdisp; d++){
			acc[d] = 0;
			for(i=maxdisp; i<maxdisp+n; i++){
				V[i][d] = V[i][d] + abs( L[ (y+r)*w+i] - R[ (y+r)*w+i-d] ) - abs( L[ (y-r-1)*w+i] - R[ (y-r-1)*w+i-d] );
				acc[d] += V[i][d];			
			}	

			if(acc[d]  < sad_min){
 				sad_min = acc[d];
 				dbestph = d;
 			}
		}
		bdh[y*w +maxdisp+r] = dbestph;

		//compute ACC for all other positions
		lp = (unsigned char *) L + (y+r)*w + ind1;
		rp =  (unsigned char *) R + (y+r)*w + ind1;
		lpp = (unsigned char *) L + (y-r-1)*w + ind1;
		rpp = (unsigned char *) R + (y-r-1)*w + ind1;
		
		for(x=maxdisp+r+1; x<w-r; x++){
			lp++;
			rp++;
			lpp++;
			rpp++;
			for(d=0; d<maxdisp; d++){
				V[x+r][d] = V[x+r][d] + abs( *lp - *rp ) - abs( *lpp - *rpp ); 
				rp--;
				rpp--;
				acc[d] = acc[d] + V[x+r][d] - V[x-r-1][d];
			}//d
			rp += maxdisp;
			rpp += maxdisp;
		
			temp = bdh[y*w+x-1];
			dbest = compute_penalty(temp, acc, pi1, pi2, maxdisp);
			bdh[y*w+x] = dbest;
			
			temp = bdv[(y-1)*w+x];
			dbest = compute_penalty(temp, acc, pi1, pi2, maxdisp);
			bdv[y*w+x] = dbest;
		}//x to compute ACC
		
	}
	//END 1st PASS

	//BEGIN 2nd pass : bottomright 2 topleft

	//STAGE 2: other positions
	for(x=w-r-2; x>=maxdisp+r+1; x--){
		sad_min = sad_max;
		for(d=0; d<maxdisp; d++){
			acc[d] = acc[d] + V[x-r][d] - V[x+r+1][d];
			if(acc[d]  < sad_min){
 				sad_min = acc[d];
 				dbestpv = d;
 			}
		}//d
		bdu[x] = dbestpv;
	}//x

	for(y=h-r-2; y>=r; y--){

		sad_min = sad_max;
		//first position at right side
		for(d=0; d<maxdisp; d++){
			acc[d] = 0;
			for(i=w-1; i>=w-n; i--){
				V[i][d] = V[i][d] + abs( L[ (y-r)*w+i] - R[ (y-r)*w+i-d] ) - abs( L[ (y+r+1)*w+i] - R[ (y+r+1)*w+i-d] );
				acc[d] += V[i][d];			
			}	

			if(acc[d]  < sad_min){
 				sad_min = acc[d];
 				dbestph = d;
 			}
		}

		//compute ACC for all other positions
		for(x=w-r-2; x>=maxdisp+r+1; x--){
			for(d=0; d<maxdisp; d++){
				V[x-r][d] = V[x-r][d] + abs( L[(y-r)*w + x-r] - R[ (y-r)*w + x-r-d] ) - abs( L[ (y+r+1)*w+ x-r] - R[ (y+r+1)*w+ x-r-d] ); 
				acc[d] = acc[d] + V[x-r][d] - V[x+r+1][d];
			}//d
				
			//compute final disparity from 4 directions
			
			temp = bdu[x];
			if(temp>0)
				acc[temp-1] -= pi2;  
			acc[temp] -= pi1;  
			if(temp < maxdisp-1)
				acc[temp+1] -= pi2;  

			temp = dbestph;
			if(temp>0)
				acc[temp-1] -= pi2;  
			acc[temp] -= pi1;  
			if(temp < maxdisp-1)
				acc[temp+1] -= pi2;  

			temp = bdv[(y-1)*w+x];
			if(temp>0)
				acc[temp-1] -= pi2;  
			acc[temp] -= pi1;  
			if(temp < maxdisp-1)
				acc[temp+1] -= pi2;  
		
			temp = bdh[y*w+x-1];
			if(temp>0)
				acc[temp-1] -= pi2;  
			acc[temp] -= pi1;  
			if(temp < maxdisp-1)
				acc[temp+1] -= pi2;  
			
			sad_min = sad_max;
			for(d=0; d<maxdisp; d++){
				if(acc[d]  < sad_min){
					sad_min = acc[d];
					dbest = d;
				}
			}

			disp[y*w+x] = dbest*16;

			//( 2)needed for uniqueness constraint filter )
			//min_scores[x] = sad_min;
		
			//3) uniqueness filter
			sad_second_min = 32000;
			for(d=0; d<dbest-1; d++){
				if(acc[d]<sad_second_min){
					sad_second_min = acc[d];
				}
			}
			for(d=dbest+2; d<maxdisp; d++){
				if(acc[d]<sad_second_min){
					sad_second_min = acc[d];
				}
			}	
			if( sad_min*100  > ratio_filter*sad_second_min)
				disp[y*w + x] = FILTERED;
		
			//4) Peak Filter	
	 		da = (dbest>1) ? ( acc[dbest-2] - acc[dbest] ) : (acc[dbest+2] - acc[dbest]);
	 		db =  (dbest<maxdisp-2) ? (acc[dbest+2] - acc[dbest]) : (acc[dbest-2] - acc[dbest]);		
	 		if(da + db < peak_filter)
	 			disp[y*w + x] = FILTERED;
			// ******* FILTERS ********

			//subpixel refinement
			if(disp[y*w + x] != FILTERED){
				double v = dbest + ( (acc[dbest-1] - acc[dbest+1])/(2.0*(acc[dbest-1]+acc[dbest+1]-2.0*acc[dbest])) );
				disp[y*w + x] = (int)(0.5 + 16*v);
			}
	
			temp = bdu[x];
			if(temp>0)
				acc[temp-1] += pi2;  
			acc[temp] += pi1;  
			if(temp < maxdisp-1)
				acc[temp+1] += pi2;  

			temp = dbestph;
			if(temp>0)
				acc[temp-1] += pi2;  
			acc[temp] += pi1;  
			if(temp < maxdisp-1)
				acc[temp+1] += pi2;  

			temp = bdv[(y-1)*w+x];
			if(temp>0)
				acc[temp-1] += pi2;  
			acc[temp] += pi1;  
			if(temp < maxdisp-1)
				acc[temp+1] += pi2;  
			
			temp = bdh[y*w+x-1];
			if(temp>0)
				acc[temp-1] += pi2;  
			acc[temp] += pi1;  
			if(temp < maxdisp-1)
				acc[temp+1] += pi2;  			
			
			//UPDATE bdu
			temp = bdu[x];
			dbest = compute_penalty(temp, acc, pi1, pi2, maxdisp);
			bdu[x] = dbest;

			temp = dbestph;
			dbestph = compute_penalty(temp, acc, pi1, pi2, maxdisp);
					
		}//x to compute ACC
		//if(unique_c)
		//	uniqueness_constraint_reducedrange(w, disp, maxdisp, r, min_scores, y);
	}
	//END 2nd PASS

	for(d=0; d<w;d++){
		free(V[d]);
	}
	free(V);
	free(acc);	

	free(bdv);
	free(bdh);
	free(bdu);

	//free(min_scores);
}


//block-based using NCC
void do_stereo_ncc(uint8_t *lim, uint8_t *rim, // input feature images
	  int16_t *disp,	// disparity output
	  int xim, int yim,	// size of images
	  uint8_t ftzero,	// feature offset from zero
	  int xwin, int ywin,	// size of corr window, usually square
	  int dlen,		// size of disparity search, multiple of 8
	  int pfilter_thresh,	// texture filter threshold
	  int ufilter_thresh,	// uniqueness filter threshold, percent
          int unique_c         // uniqueness check
	)
{

int maxdisp = dlen;
int r = (xwin-1)/2;
int n = xwin;
//note: ywim is currently deprecated

int w = xim;
int h = yim;

unsigned char *L = lim;
unsigned char *R = rim;

//temp variables
int x,y,d,i,j;

// offset disparity image by prefilter kernel
disp += w*(YKERN-1)/2 + (XKERN-1)/2;

//FILTERS
// int *min_scores = (int *)malloc(w * sizeof(int ));
 int ratio_filter = 1000-ufilter_thresh;
 int peak_filter = pfilter_thresh;	
 double ncc_second_max;
 int da, db;

int *normR = (int *)calloc(w, sizeof(int));
int *acc = (int *)calloc(maxdisp, sizeof(int));
int **V = (int **)calloc(w, sizeof(int*));
for(d=0; d<w; d++){
	V[d] = (int *)calloc(maxdisp, sizeof(int)); 
}
int *Vn = (int *)calloc(w, sizeof(int));

double *temp = (double *)calloc(maxdisp, sizeof(double));
double ncc_max;
int dbest=0;

//FIRST ROW
//STAGE 1 - init acc
for(d=0; d<maxdisp; d++){
	for(i=maxdisp; i<maxdisp+n; i++){
		for(j=0; j<n; j++)
			V[i][d] += L[j*w+i] * R[j*w+i-d];
		acc[d] += V[i][d];			
	}		
}

//NORM R - FIRST ROW
for(i=0; i<w; i++)
for(j=0; j<n; j++)
	Vn[i] += R[j*w+i] * R[j*w+i];		
		
//STAGE 2: other positions
for(x=maxdisp+r+1; x<w-r; x++){
	for(d=0; d<maxdisp; d++){
		for(j=0; j<n; j++)
			V[x+r][d] += L[j*w + x+r] * R[ j*w + x+r-d];
		acc[d] = acc[d] + V[x+r][d] - V[x-r-1][d];
	}//d
}//x

unsigned char *lp, *rp, *lpp, *rpp;
int ind1 = r+r+maxdisp;
//OTHER ROWS
for(y=r+1; y<h-r-YKERN/2; y++){
	
	//first position
	for(d=0; d<maxdisp; d++){
		acc[d] = 0;
		for(i=maxdisp; i<maxdisp+n; i++){
			V[i][d] = V[i][d] + (L[ (y+r)*w+i]*R[ (y+r)*w+i-d]) - (L[ (y-r-1)*w+i]*R[ (y-r-1)*w+i-d]);
			acc[d] += V[i][d];			
		}	
	}

	//compute normR for the whole row
	normR[r] = 0;
	for(i=0; i<n; i++){
		Vn[i] = Vn[i] + (R[ (y+r)*w+i]*R[ (y+r)*w+i]) - (R[ (y-r-1)*w+i]*R[ (y-r-1)*w+i]);
		normR[r] += Vn[i];				
	}
	for(x=r+1; x<w-r-XKERN/2; x++){
		Vn[x+r] = Vn[x+r] + (R[(y+r)*w + x+r]*R[ (y+r)*w + x+r]) - (R[ (y-r-1)*w+ x+r]*R[ (y-r-1)*w+ x+r]);
		normR[x] = normR[x-1] + Vn[x+r] - Vn[x-r-1];
	}

	//other positions
	lp = (unsigned char *) L + (y+r)*w + ind1;
	rp =  (unsigned char *) R + (y+r)*w + ind1;
	lpp = (unsigned char *) L + (y-r-1)*w + ind1;
	rpp = (unsigned char *) R + (y-r-1)*w + ind1;

	for(x=maxdisp+r+1; x<w-r-XKERN/2; x++){
		ncc_max = 0.0;
		
		lp++;
		rp++;
		lpp++;
		rpp++;

		for(d=0; d<maxdisp; d++){
			
			V[x+r][d] = V[x+r][d] + (*lp) * (*rp) - (*lpp) * (*rpp); 
			
			rp--;
			rpp--;
			
			acc[d] = acc[d] + V[x+r][d] - V[x-r-1][d];
			
			temp[d] = acc[d] / sqrt((double)(normR[x-d])) ; 
			if( temp[d] > ncc_max){
				ncc_max = temp[d];
				dbest = d;
			}
		}//d
		
		rp += maxdisp;
		rpp += maxdisp;
		
		disp[y*w + x] = dbest*16;
		
		//( 2)needed for uniqueness constraint filter )
		//min_scores[x] = sad_min;
	
		//3) uniqueness filter
 		ncc_second_max = 0;
		for(d=0; d<dbest-1; d++){
 			if(temp[d]>ncc_second_max){
 				ncc_second_max = temp[d];
 			}
 		}
 		for(d=dbest+2; d<maxdisp; d++){
 			if(temp[d]>ncc_second_max){
 				ncc_second_max = temp[d];
 			}
 		}
 
 		if( ncc_second_max*1000  > ratio_filter*ncc_max)
 			disp[y*w + x] = FILTERED;
 	
 		//4) Peak Filter
 		da = (dbest>1) ? ( temp[dbest] - temp[dbest-2] ) : (temp[dbest] - temp[dbest+2]);
 		db =  (dbest<maxdisp-2) ? (temp[dbest] - temp[dbest+2]) : (temp[dbest] - temp[dbest-2]);		
 		if(da + db < peak_filter)
			disp[y*w + x] = FILTERED;
		//******* FILTERS ********

		//subpixel refinement
		if(disp[y*w + x] != FILTERED){
			double v = dbest + ( (temp[dbest-1] - temp[dbest+1])/(2.0*(temp[dbest-1]+temp[dbest+1]-2.0*temp[dbest])) );
			disp[y*w + x] = (int)(0.5 + 16*v);
		}
	}//x
	//if(par.unique == 1)
	//	uniqueness_constraint_reducedrange(par, min_scores, y);
}//y

for(d=0; d<w;d++){
	free(V[d]);
}
free(V);
free(acc);
free(Vn);
free(normR);
free(temp);

//free(min_scores);
}





