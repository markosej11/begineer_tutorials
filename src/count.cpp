#include <ros/ros.h>

int main (int argc, char **argv) {
    ros::init(argc, argv, "count_and_log");
    ros::NodeHandle nh;

    // Generate l o g messages of varying s e v e r i t y r e g u l a r l y .
    ros::Rate rate (10);
    for(int i = 1;ros::ok();i++){
        ROS_DEBUG_STREAM("Counted  to " << i ) ;
        if((i%3) == 0){
            ROS_INFO_STREAM( i << " ␣ i s ␣ d i v i s i b l e ␣by␣ 3 . " ) ;
        }
        if((i%5) == 0){
            ROS_WARN_STREAM( i << " ␣ i s ␣ d i v i s i b l e ␣by␣ 5 . " ) ;
        }
        if((i%10) == 0){
            ROS_ERROR_STREAM( i << " ␣ i s ␣ d i v i s i b l e ␣by␣ 10 . " ) ;
        }
        if((i%20) == 0){
            ROS_FATAL_STREAM( i << " ␣ i s ␣ d i v i s i b l e ␣by␣ 20 . " ) ;
        }
        rate.sleep();
    }
}
