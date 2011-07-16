#include "yuv422.h"

namespace image_proc {

void yuv422ToGray(const cv::Mat& yuv, cv::Mat& gray)
{
#if 0
  // u y1 v y2
  if (width > image_md_->XRes () || height > image_md_->YRes ())
    THROW_OPENNI_EXCEPTION ("Upsampling not supported. Request was: %d x %d -> %d x %d", image_md_->XRes (), image_md_->YRes (), width, height);

  if (image_md_->XRes () % width != 0 || image_md_->YRes () % height != 0)
      THROW_OPENNI_EXCEPTION ("Downsampling only possible for integer scales in both dimensions. Request was %d x %d -> %d x %d.", image_md_->XRes (), image_md_->YRes (), width, height);

  unsigned gray_line_skip = 0;
  if (gray_line_step != 0)
    gray_line_skip = gray_line_step - width;

  register unsigned yuv_step = image_md_->XRes() / width;
  register unsigned yuv_x_step = yuv_step << 1;
  register unsigned yuv_skip = (image_md_->YRes() / height - 1) * ( image_md_->XRes() << 1 );
  register const XnUInt8* yuv_buffer = (image_md_->Data() + 1);

  for( register unsigned yIdx = 0; yIdx < image_md_->YRes(); yIdx += yuv_step, yuv_buffer += yuv_skip, gray_buffer += gray_line_skip )
  {
    for( register unsigned xIdx = 0; xIdx < image_md_->XRes(); xIdx += yuv_step, ++gray_buffer, yuv_buffer += yuv_x_step )
    {
      *gray_buffer = *yuv_buffer;
    }
  }
#endif
}

void yuv422ToColor(const cv::Mat& yuv, cv::Mat& color)
{
#if 0
  // 0  1   2  3
  // u  y1  v  y2

  if (image_md_->XRes() != width && image_md_->YRes() != height)
  {
    if (width > image_md_->XRes () || height > image_md_->YRes ())
      THROW_OPENNI_EXCEPTION ("Upsampling not supported. Request was: %d x %d -> %d x %d", image_md_->XRes (), image_md_->YRes (), width, height);

    if ( image_md_->XRes () % width != 0 || image_md_->YRes () % height != 0
        || (image_md_->XRes () / width) & 0x01 || (image_md_->YRes () / height & 0x01) )
        THROW_OPENNI_EXCEPTION ("Downsampling only possible for power of two scale in both dimensions. Request was %d x %d -> %d x %d.", image_md_->XRes (), image_md_->YRes (), width, height);
  }

  register const XnUInt8* yuv_buffer = image_md_->Data();

  unsigned rgb_line_skip = 0;
  if (rgb_line_step != 0)
    rgb_line_skip = rgb_line_step - width * 3;

  if (image_md_->XRes() == width && image_md_->YRes() == height)
  {
    for( register unsigned yIdx = 0; yIdx < height; ++yIdx, rgb_buffer += rgb_line_skip )
    {
      for( register unsigned xIdx = 0; xIdx < width; xIdx += 2, rgb_buffer += 6, yuv_buffer += 4 )
      {
        int v = yuv_buffer[2] - 128;
        int u = yuv_buffer[0] - 128;

        rgb_buffer[0] =  CLIP_CHAR (yuv_buffer[1] + ((v * 18678 + 8192 ) >> 14));
        rgb_buffer[1] =  CLIP_CHAR (yuv_buffer[1] + ((v * -9519 - u * 6472 + 8192 ) >> 14));
        rgb_buffer[2] =  CLIP_CHAR (yuv_buffer[1] + ((u * 33292 + 8192 ) >> 14));

        rgb_buffer[3] =  CLIP_CHAR (yuv_buffer[3] + ((v * 18678 + 8192 ) >> 14));
        rgb_buffer[4] =  CLIP_CHAR (yuv_buffer[3] + ((v * -9519 - u * 6472 + 8192 ) >> 14));
        rgb_buffer[5] =  CLIP_CHAR (yuv_buffer[3] + ((u * 33292 + 8192 ) >> 14));
      }
    }
  }
  else
  {
    register unsigned yuv_step = image_md_->XRes() / width;
    register unsigned yuv_x_step = yuv_step << 1;
    register unsigned yuv_skip = (image_md_->YRes() / height - 1) * ( image_md_->XRes() << 1 );
    
    for( register unsigned yIdx = 0; yIdx < image_md_->YRes(); yIdx += yuv_step, yuv_buffer += yuv_skip, rgb_buffer += rgb_line_skip )
    {
      for( register unsigned xIdx = 0; xIdx < image_md_->XRes(); xIdx += yuv_step, rgb_buffer += 3, yuv_buffer += yuv_x_step )
      {
        int v = yuv_buffer[2] - 128;
        int u = yuv_buffer[0] - 128;

        rgb_buffer[0] =  CLIP_CHAR (yuv_buffer[1] + ((v * 18678 + 8192 ) >> 14));
        rgb_buffer[1] =  CLIP_CHAR (yuv_buffer[1] + ((v * -9519 - u * 6472 + 8192 ) >> 14));
        rgb_buffer[2] =  CLIP_CHAR (yuv_buffer[1] + ((u * 33292 + 8192 ) >> 14));
      }
    }
  }
#endif
}

} // namespace image_proc
