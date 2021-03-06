/** @file listener.cpp 
* @brief Subscriber for beginner tutorials
* @Copyright MIT License 2021 Markose Jacob
*/
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "begineer_tutorials/modifyMessages.h"

/** @brief This is the interrupt handler 
 * @param String message that arises at the interrupt
 * @return None
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO_STREAM("Listener : " << msg->data.c_str());
}
int main(int argc, char** argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listener");
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;
  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("chatter", 5, chatterCallback);
  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */

  ros::Duration(5.0).sleep();
  ros::ServiceClient client =
   n.serviceClient<begineer_tutorials::modifyMessages>("modifyMessages");
  begineer_tutorials::modifyMessages srv;
  srv.request.newMsg = " \"Changed msg to Assignment completed successfully\"";
  if (client.call(srv)) {
    ROS_INFO_STREAM("Response received : "
    << static_cast<int>(srv.response.resp));
  } else {
    ROS_ERROR_STREAM("Failed to call service ");
  }
  ros::spin();
  return 0;
}
