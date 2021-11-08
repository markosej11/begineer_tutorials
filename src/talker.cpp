/** @file talker.cpp
* @brief Publisher file 
* @Copyright MIT License 2021 Markose Jacob
*/
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "begineer_tutorials/modifyMessages.h"
// #include "ros/console.h"
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */

std::string pubMessage;

bool modifyMyMessage(begineer_tutorials::modifyMessages::Request &req, \
          const begineer_tutorials::modifyMessages::Response &res) {
  ROS_INFO("Request recieved to change string to %s", req.newMsg);
  try {
    pubMessage = req.newMsg;
    ROS_WARN("Successfully changed the string message");
    return true;
  }
  catch (const std::exception&) {
    ROS_ERROR("Couldn't change the string message");
  }
  return false;
}


int main(int argc, char **argv) {
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
  ros::init(argc, argv, "talker");
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("modifyMessage", modifyMyMessage);
  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  int frequency = 3;
  bool result = n.getParam("freqency", frequency);
  if (result){
    ROS_INFO_STREAM("successfully got the param");
  }
  ros::Rate loop_rate(frequency);

  ROS_DEBUG_STREAM_ONCE("This is a Debug Stream" << " Message");
  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  pubMessage = "Week 10 assignment ";
  while (ros::ok()) {
    std_msgs::String msg;
    std::stringstream ss;
    ss <<pubMessage <<count;
    msg.data = ss.str();
    ROS_INFO("Talker : %s", msg.data.c_str());
    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);
    ROS_WARN_STREAM_ONCE("This is a Warn Stream" << " Message");
    ROS_INFO_STREAM_ONCE("This is a Info Stream" << " Message");
    ROS_ERROR_STREAM_ONCE("This is a Error Stream" << " Message");
    ROS_FATAL_STREAM_ONCE("This is a Fatal Stream" << " Message");
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
