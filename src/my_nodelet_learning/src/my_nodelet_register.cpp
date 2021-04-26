#include <pluginlib/class_list_macros.h>
#include <my_nodelet_learning/my_nodelet_class.h>

PLUGINLIB_EXPORT_CLASS(myNodeletClass::myNodeLet, nodelet::Nodelet)
PLUGINLIB_EXPORT_CLASS(serialPubSub::serialPubSubNodelet, nodelet::Nodelet)
