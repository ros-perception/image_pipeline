#include <pluginlib/class_list_macros.h>
#include "theora_image_transport/theora_publisher.h"
#include "theora_image_transport/theora_subscriber.h"

PLUGINLIB_REGISTER_CLASS(theora_pub, theora_image_transport::TheoraPublisher, image_transport::PublisherPlugin)

PLUGINLIB_REGISTER_CLASS(theora_sub, theora_image_transport::TheoraSubscriber, image_transport::SubscriberPlugin)
