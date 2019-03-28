#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/utils.h>
using namespace ros;

void callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    geometry_msgs::Pose pose = msg->pose.pose;

    double yam = tf2::getYaw(pose.orientation);

    ROS_INFO_STREAM(yam);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_print");
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("/amcl_pose", 10, callback);

    ros::spin();
    return 0;
}