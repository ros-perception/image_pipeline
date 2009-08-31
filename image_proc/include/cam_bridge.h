
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

#ifndef CAM_BRIDGE_H
#define CAM_BRIDGE_H

namespace cam_bridge
{

  color_coding_t GetColorCoding(const sensor_msgs::Image& msg)
  {
    using namespace sensor_msgs::image_encodings;
    
    if (msg.encoding == MONO8)       return COLOR_CODING_MONO8;
    if (msg.encoding == MONO16)      return COLOR_CODING_MONO16;
    if (msg.encoding == BAYER_RGGB8) return COLOR_CODING_BAYER8_RGGB;
    if (msg.encoding == BAYER_BGGR8) return COLOR_CODING_BAYER8_BGGR;
    if (msg.encoding == BAYER_GBRG8) return COLOR_CODING_BAYER8_GBRG;
    if (msg.encoding == BAYER_GRBG8) return COLOR_CODING_BAYER8_GRBG;
    if (msg.encoding == RGB8)        return COLOR_CODING_RGB8;
    if (msg.encoding == RGBA8)       return COLOR_CODING_RGBA8;

    ROS_ERROR("cam_bridge: Encoding '%s' is not supported", msg.encoding.c_str());
    return COLOR_CODING_NONE;
  }

  std::string ColorCodingToImageEncoding(color_coding_t coding)
  {
    using namespace sensor_msgs::image_encodings;
    
    if (coding == COLOR_CODING_MONO8)       return MONO8;
    if (coding == COLOR_CODING_MONO16)      return MONO16;
    if (coding == COLOR_CODING_BAYER8_RGGB) return BAYER_RGGB8;
    if (coding == COLOR_CODING_BAYER8_BGGR) return BAYER_BGGR8;
    if (coding == COLOR_CODING_BAYER8_GBRG) return BAYER_GBRG8;
    if (coding == COLOR_CODING_BAYER8_GRBG) return BAYER_GRBG8;
    if (coding == COLOR_CODING_RGB8)        return RGB8;
    if (coding == COLOR_CODING_RGBA8)       return RGBA8;

    ROS_WARN("cam_bridge: Don't know image encoding string for color coding %i", coding);
    return "";
  }

  void extractImage(std::vector<uint8_t> data, size_t* sz, uint8_t **d)
  {
    size_t new_size = data.size();

    if (*sz < new_size)
    {
      MEMFREE(*d);
      *d = (uint8_t *)MEMALIGN(new_size);
      *sz = new_size;
    }
    memcpy((char*)(*d), (char*)(&data[0]), new_size);
  }
  void extractImage(std::vector<uint8_t> data, size_t* sz, int16_t **d)
  {
    size_t new_size = data.size();

    if (*sz < new_size)
    {
      MEMFREE(*d);
      *d = (int16_t *)MEMALIGN(new_size);
      *sz = new_size;
    }
    memcpy((char*)(*d), (char*)(&data[0]), new_size);
  }


  void RawToCamData(const sensor_msgs::Image& im_msg, 
                    const sensor_msgs::CameraInfo& info_msg, 
		    uint8_t type, cam::ImageData* im)
  {

    im->imRawType = COLOR_CODING_NONE;
    im->imType = COLOR_CODING_NONE;
    im->imColorType = COLOR_CODING_NONE;
    im->imRectType = COLOR_CODING_NONE;
    im->imRectColorType = COLOR_CODING_NONE;

    if (type == cam::IMAGE_RAW)
    {
      extractImage(im_msg.data, &im->imRawSize, &im->imRaw);
      im->imRawType = GetColorCoding(im_msg);
    }
    else if (type == cam::IMAGE)
    {
      extractImage(im_msg.data, &im->imSize, &im->im);
      im->imType = COLOR_CODING_MONO8;
    }
    else if (type == cam::IMAGE_COLOR)
    {
      extractImage(im_msg.data, &im->imColorSize, &im->imColor);
      im->imColorType = GetColorCoding(im_msg);
    }
    else if (type == cam::IMAGE_RECT)
    {
      extractImage(im_msg.data, &im->imRectSize, &im->imRect);
      im->imRectType = GetColorCoding(im_msg);
    }
    else if (type == cam::IMAGE_RECT_COLOR)
    {
      extractImage(im_msg.data, &im->imRectColorSize, &im->imRectColor);
      im->imRectColorType = GetColorCoding(im_msg);
    }

    // @todo: this OK when right image empty (disparity image requested instead)?
    im->imHeight = im_msg.height;
    im->imWidth  = im_msg.width;

    // @todo: possible to NOT have rectification?
    memcpy((char*)(im->D), (char*)(&info_msg.D[0]),  5*sizeof(double));
    memcpy((char*)(im->K), (char*)(&info_msg.K[0]),  9*sizeof(double));
    memcpy((char*)(im->R), (char*)(&info_msg.R[0]),  9*sizeof(double));
    memcpy((char*)(im->P), (char*)(&info_msg.P[0]),  12*sizeof(double));
    im->hasRectification = true;
  }
}


#endif	// CAM_BRIDGE_H
