#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

class ImageFlipper{
private:
    ros::NodeHandle _nh;
    image_transport::ImageTransport _it;
    image_transport::Publisher _pub;
    image_transport::Subscriber _sub;
    sensor_msgs::ImagePtr _msg;
    cv::Mat _cvmat;
    int _flip_value;

public:
    ImageFlipper(int flip_value)
        :_it(_nh)
    {
        _flip_value = flip_value;
        _pub = _it.advertise("flipped", 1);
        _sub = _it.subscribe("image", 1, &ImageFlipper::image_cb, this);
        if (ros::names::remap("image") == "image") {
          ROS_WARN("Topic 'image' has not been remapped! Typical command-line usage:\n"
                   "\t$ rosrun image_flip image_flip image:=<image topic> flipped:=<flipped image topic> <horizontal/vertical/both>");
        }
        ROS_INFO_STREAM("Flipping "
                        <<  ( (_flip_value == 1)?"horizontally ":"" )
                        <<  ( (_flip_value == 0)?"vertically ":"" )
                        <<  ( (_flip_value == -1)?"both horizontally and vertically ":"" )
                        << "topic "
                        << _sub.getTopic() << " into " << _pub.getTopic());
    }

    void image_cb(const sensor_msgs::ImageConstPtr& incoming_img){
        // Do nothing if no one is subscribed
        if (_pub.getNumSubscribers() < 1)
            return;

        // ROS image message to opencv
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
          cv_ptr = cv_bridge::toCvCopy(incoming_img, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
        }
        // From http://docs.opencv.org/modules/core/doc/operations_on_arrays.html#void flip(InputArray src, OutputArray dst, int flipCode)
        // FLIP_HORIZONTAL == 1, FLIP_VERTICAL == 0 or FLIP_BOTH == -1
        cv::flip(cv_ptr->image, _cvmat, _flip_value);

        // go back to ros message
        _msg = cv_bridge::CvImage(incoming_img->header, "bgr8", _cvmat).toImageMsg();

        // publish image
        _pub.publish(_msg);
    }


};



int main(int argc, char** argv)
{
    // This version of ros::init cleans argc + argv from ros related args
    ros::init(argc, argv, "image_flip", ros::init_options::AnonymousName);
    // Initialize only if we are given horizontal vertical or both orientation to flip
    // It can be checked just with the first character
    if (argc <= 1 || (argv[1][0] != 'h' && argv[1][0] != 'v' && argv[1][0] != 'b'))
        ROS_ERROR_STREAM("No correct flip orientation given! Typical command-line usage:\n"
                         "\t$ rosrun image_flip image_flip image:=<image topic> flipped:=<flipped image topic> <h(orizontal)/v(ertical)/b(oth)>\n"
                         "\te.g.:\n"
                         "\t$ rosrun image_flip image_flip image:=/camera/image_raw flipped:=/camera/image_vertical_flip_raw v");
    else{
        char flip = argv[1][0];
        int flip_value;
        if (flip == 'h')
            flip_value = 1;
        else if (flip == 'v')
            flip_value = 0;
        else
            flip_value = -1;

        ImageFlipper imageflipper(flip_value);
        ros::spin();
        }
}
