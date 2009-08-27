#include <pluginlib/class_list_macros.h>
#include "compressed_image_transport/compressed_publisher.h"
#include "compressed_image_transport/compressed_subscriber.h"

PLUGINLIB_REGISTER_CLASS(compressed_pub, compressed_image_transport::CompressedPublisher, image_transport::PublisherPlugin)

PLUGINLIB_REGISTER_CLASS(compressed_sub, compressed_image_transport::CompressedSubscriber, image_transport::SubscriberPlugin)
