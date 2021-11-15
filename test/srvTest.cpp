/** @file srvTest.cpp
* @brief test file for talker 
* @Copyright MIT License 2021 Markose Jacob
*/
#include <gtest/gtest.h>
#include <begineer_tutorials/modifyMessages.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <bits/stdc++.h>
#include <sstream>
#include <ros/console.h>


TEST(TestSuite, testCase1) {
  ROS_INFO("Started TEST");
  ros::NodeHandle nh;
  ros::ServiceClient client =
    nh.serviceClient<begineer_tutorials::modifyMessages>("modifyMessages");
  begineer_tutorials::modifyMessages srv;
  srv.request.newMsg = "\"Changed msg to Assignment completed successfully\"";
  ros::Rate sleep_rate(5);
  if (client.call(srv)) {
    ROS_INFO("Response received : %d", static_cast<bool>(srv.response.resp));
    sleep_rate.sleep();
    ros::Duration dur(5);
    EXPECT_EQ(static_cast<bool>(srv.response.resp), true);
  } else {
    ROS_ERROR("Failed to call service ");
  }
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  std::cout << "Started test node" << std::endl;
  ros::init(argc, argv, "srvTest");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
