#include "yuv422.h"

#define CLIP_CHAR(c) ((c)>255?255:(c)<0?0:(c))

namespace image_proc {

void yuv422ToGray(const cv::Mat& yuv, cv::Mat& gray)
{
  unsigned width = gray.cols;
  unsigned height = gray.rows;
  unsigned gray_skip = gray.step[0] - width;
  unsigned yuv_skip = yuv.step[0] - width*2;
  unsigned char* gray_buffer = gray.datastart;
  const unsigned char* yuv_buffer = yuv.datastart;

  // u y1 v y2
  for( unsigned yIdx = 0; yIdx < height;
       ++yIdx, gray_buffer += gray_skip, yuv_buffer += yuv_skip )
  {
    for( unsigned xIdx = 0; xIdx < width;
         ++xIdx, ++gray_buffer, yuv_buffer += 2 )
    {
      *gray_buffer = yuv_buffer[1];
    }
  }
}

void yuv422ToColor(const cv::Mat& yuv, cv::Mat& color)
{
  unsigned width = color.cols;
  unsigned height = color.rows;
  unsigned bgr_skip = color.step[0] - width*3;
  unsigned yuv_skip = yuv.step[0] - width*2;
  unsigned char* bgr_buffer = color.datastart;
  const unsigned char* yuv_buffer = yuv.datastart;

  // 0  1   2  3
  // u  y1  v  y2
  for( unsigned yIdx = 0; yIdx < height;
       ++yIdx, bgr_buffer += bgr_skip, yuv_buffer += yuv_skip )
  {
    for( unsigned xIdx = 0; xIdx < width;
         xIdx += 2, bgr_buffer += 6, yuv_buffer += 4 )
    {
      int v = yuv_buffer[2] - 128;
      int u = yuv_buffer[0] - 128;

      bgr_buffer[0] =  CLIP_CHAR (yuv_buffer[1] + ((u * 33292 + 8192 ) >> 14));
      bgr_buffer[1] =  CLIP_CHAR (yuv_buffer[1] + ((v * -9519 - u * 6472 + 8192 ) >> 14));
      bgr_buffer[2] =  CLIP_CHAR (yuv_buffer[1] + ((v * 18678 + 8192 ) >> 14));

      bgr_buffer[3] =  CLIP_CHAR (yuv_buffer[3] + ((u * 33292 + 8192 ) >> 14));
      bgr_buffer[4] =  CLIP_CHAR (yuv_buffer[3] + ((v * -9519 - u * 6472 + 8192 ) >> 14));
      bgr_buffer[5] =  CLIP_CHAR (yuv_buffer[3] + ((v * 18678 + 8192 ) >> 14));
    }
  }
}

} // namespace image_proc
