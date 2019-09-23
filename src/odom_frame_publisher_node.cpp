// headers in this package
#include <odom_frame_publisher/odom_frame_publisher.h>

// headers for ros
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "odom_frame_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    OdomFramePublisher publisher(nh,pnh);
    ros::spin();
    return 0;
}