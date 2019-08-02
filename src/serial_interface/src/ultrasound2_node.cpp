#include <ros/ros.h>
#include <serial/serial.h>
#include <string>
#include <vector>
#include <std_msgs/Header.h>
#include <tf/transform_broadcaster.h>

#include "sensor_msgs/LaserScan.h"

#define DEG2RAD(x) ((x)*M_PI/180.)

using namespace std;
using namespace ros;

class Ultrasound
{
public:
    Ultrasound();
    ros::NodeHandle node;
    std::vector<double> distance_vector;
    void exec();
private:
    int number;
    serial::Serial ros_ser;
    void sendCommond(int i);
    ros::Publisher pub_LaserScan;
};
Ultrasound::Ultrasound()
{
    ros::NodeHandle nh_("~");
    nh_.param<int>("number", number, 12);

    try {
        ros_ser.setPort("/dev/ultrasound_serial");
        ros_ser.setBaudrate(9600);
        serial::Timeout to = serial::Timeout::simpleTimeout(200);
        ros_ser.setTimeout(to);
        ros_ser.open();
    } catch (serial::IOException &e) {
        ROS_ERROR_STREAM("Unable to open port.");
        exit(-1);
    }

    pub_LaserScan = node.advertise<sensor_msgs::LaserScan>("scan_ultrasound",100);
}
void Ultrasound::sendCommond(int i)
{
    unsigned char commond[3] = {0xe8, 0x02, 0};
    commond[2] = 16 * ((i + 2) / 2) + 4 + i % 2 * 8;
    ros_ser.write(commond, 3);
}
void Ultrasound::exec()
{
    unsigned char receive[2];
    while(ros::ok())
    {
        distance_vector.resize(12, 2);

        int node_count = 36;

        sensor_msgs::LaserScan scan_msg;
        scan_msg.header.stamp = ros::Time::now();
        scan_msg.header.frame_id = "/ultrasoundLaser";
        scan_msg.angle_min = -1 * M_PI;
        scan_msg.angle_max = M_PI;
        scan_msg.angle_increment = DEG2RAD(10);
        scan_msg.range_min = 0;
        scan_msg.range_max = 10;
        scan_msg.ranges.resize(node_count);
        scan_msg.intensities.resize(node_count);

        for(int index = 0; index < number; index++)
        {
            if(index == 5 || index == 6 || index == 11) //没有用到
                continue;
            double distance = 2;

            sendCommond(index);
            size_t size_result = ros_ser.read(receive, 2);
            if(size_result == 2)
            {
                distance = ((int)receive[0] * 256 + (int)receive[1]) / 1000.0;
                if(distance > 2)
                        distance = 2;
            }
            else
                ROS_INFO("timeout1");

            if(distance < 1)//测得小于2米后再测一遍.
            {
                distance = 2;
                sendCommond(index);
                size_t size_result = ros_ser.read(receive, 2);
                if(size_result == 2)
                {
                    distance = ((int)receive[0] * 256 + (int)receive[1]) / 1000.0;
                    if(distance > 2)
                        distance = 2;
                }
                else
                    ROS_INFO("timeout2");
            }
            distance_vector[index] = distance;
        }
        ROS_INFO_STREAM(" 1: " << distance_vector[0] << " 2: " << distance_vector[1] << 
                        " 3: " << distance_vector[2] << " 4: " << distance_vector[3] << 
                        " 5: " << distance_vector[4] << " 8: " << distance_vector[7] << 
                        " 9: " << distance_vector[8] << " 10: " << distance_vector[9] << 
                        " 11: " << distance_vector[10]);

        scan_msg.scan_time = (ros::Time::now() - scan_msg.header.stamp).toSec();
        scan_msg.time_increment = scan_msg.scan_time / (node_count - 1);

        //先全部置inf，注意：如果初始化是0,则表示范围内无障碍，故不能置0。inf表示无数据
        for (size_t i = 0; i < node_count; i++){
            scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
        }
        if(distance_vector[0] < 1)
        {
            for(int i = 15; i <= 20; i++)
            {
                scan_msg.ranges[i] = 0.25 + distance_vector[0];
                scan_msg.intensities[i] = 0.5;
            }
        }
        if(distance_vector[1] < 1)
        {
            for(int i = 15; i <= 17; i++)
            {
                scan_msg.ranges[i] = 0.25 + distance_vector[1];
                scan_msg.intensities[i] = 0.5;
            }
        }
        if(distance_vector[2] < 1)
        {
            for(int i = 18; i <= 20; i++)
            {
                scan_msg.ranges[i] = 0.25 + distance_vector[2];
                scan_msg.intensities[i] = 0.5;
            }
        }

        if(distance_vector[3] < 1)
        {
            for(int i = 8; i <= 9; i++)
            {
                scan_msg.ranges[i] = 0.3 + distance_vector[3];
                scan_msg.intensities[i] = 0.5;
            }
        }

        if(distance_vector[8] < 1)
        {

            for(int i = 26; i <= 27; i++)
            {
                scan_msg.ranges[i] = 0.3 + distance_vector[8];
                scan_msg.intensities[i] = 0.5;
            }
        }

        if(distance_vector[4] < 1 || distance_vector[7] < 1)
        {
            for(int i = 5; i <= 5; i++)
            {
                scan_msg.ranges[i] = 0.55 + std::min(distance_vector[4], distance_vector[7]);
                scan_msg.intensities[i] = 0.5;
            }
        }

        if(distance_vector[9] < 1 || distance_vector[10] < 1)
        {

            for(int i = 30; i <= 30; i++)
            {
                scan_msg.ranges[i] = 0.55 + std::min(distance_vector[9], distance_vector[10]);
                scan_msg.intensities[i] = 0.5;
            }
        }

        pub_LaserScan.publish(scan_msg);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ultrasound2_node");
    Ultrasound ultrasound;
    ultrasound.exec();
    return 0;
}
