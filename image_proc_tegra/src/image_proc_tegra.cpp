#include "ros/ros.h"
#include "nodelet/loader.h"

int main(int argc, char **argv){
  ros::init(argc, argv, "image_proc_tegra");
  nodelet::Loader nodelet;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;
  std::string nodelet_name = ros::this_node::getName();
  
  nodelet.load(nodelet_name, nodelet_name + "/RectifyNodelet", remap, nargv);
  ros::spin();
  return 0;
}
