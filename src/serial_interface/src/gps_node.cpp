#include <ros/ros.h>
#include <boost/asio/serial_port.hpp>
#include <boost/asio.hpp>
#include <string>
#include <vector>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>

using namespace std;

//字符串分割到数组  参数1：要分割的字符串；参数2：作为分隔符的字符；参数3：存放分割后的字符串的vector向量
void Split(const std::string& src, const std::string& separator, std::vector<std::string>& dest)
{
	std::string str = src;
	std::string substring;
	std::string::size_type start = 0, index;
	dest.clear();
	index = str.find_first_of(separator,start);
	do
	{
		if (index != string::npos)
		{
			substring = str.substr(start,index-start );
			dest.push_back(substring);
			start =index+separator.size();
			index = str.find(separator,start);
			if (start == string::npos) break;
		}
	}while(index != string::npos);
 
	//the last part
	substring = str.substr(start);
	dest.push_back(substring);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gps_node");
    ros::NodeHandle node;
    ros::Publisher gps_pub = node.advertise<geometry_msgs::Pose>("/gps", 10);
    ros::Publisher imu_pub = node.advertise<sensor_msgs::Imu>("/imu", 10);

    boost::asio::io_service io;
	boost::asio::serial_port sp(io, "/dev/ttyS0");

    sp.set_option(boost::asio::serial_port::baud_rate(115200));
    sp.set_option(boost::asio::serial_port::flow_control());
    sp.set_option(boost::asio::serial_port::parity());
    sp.set_option(boost::asio::serial_port::stop_bits());
    sp.set_option(boost::asio::serial_port::character_size(8));

    boost::asio::streambuf sb;
    constexpr double imu_linear_scale = 0.200 / 65536 / 125 / 1000 * 9.80665 * 125;
    constexpr double imu_angular_scale = 0.008/ 65536 / 125 * 3.14159 / 180 * 125;
    while(1)
    {
        std::size_t n = 0;
        try{
            n = boost::asio::read_until(sp, sb, '\n');
        }catch(boost::system::system_error& e)
        {
            return 0;
        }
        boost::asio::streambuf::const_buffers_type bufs = sb.data();
        std::string line(boost::asio::buffers_begin(bufs), boost::asio::buffers_begin(bufs) + n);
        sb.consume(n);
        if(line.substr(1, 8) == "INSPVAXA")
        {
            std::string subline = line.substr(line.find_first_of(';') + 1);
            std::vector<std::string> param_vector;
            Split(subline, ",", param_vector);
            //if(param_vector[0] == "INS_SOLUTION_GOOD"  && (param_vector[1] == "INS_RTKFIXED" || param_vector[1] == "INS_RTKFLOAT"))
            {
                geometry_msgs::Pose gpsPose;
                gpsPose.position.x = std::stod(param_vector[2]);//纬度
                gpsPose.position.y = std::stod(param_vector[2]);//经度
                gps_pub.publish(gpsPose);
            }
        }
        else if(line.substr(1, 8) == "RAWIMUSA")
        {
            std::string subline = line.substr(line.find_first_of(';') + 1);
            subline = subline.substr(0, subline.find_first_of('*'));
            std::vector<std::string> param_vector;
            Split(subline, ",", param_vector);

            sensor_msgs::Imu imu_data;
            imu_data.header.stamp = ros::Time::now();
            imu_data.header.frame_id = "base_link";


            tf::Quaternion quat = tf::createQuaternionFromYaw(0);
            imu_data.orientation.x = quat.x();
            imu_data.orientation.y = quat.y();
            imu_data.orientation.z = quat.z();
            imu_data.orientation.w = quat.w();
            
            imu_data.linear_acceleration.x = imu_linear_scale * std::stol(param_vector[4]);
            imu_data.linear_acceleration.y = (-1) * imu_linear_scale * std::stol(param_vector[5]);
            imu_data.linear_acceleration.z = (-1) * imu_linear_scale * std::stol(param_vector[3]);

            imu_data.angular_velocity.x = imu_angular_scale * std::stol(param_vector[7]);
            imu_data.angular_velocity.y = (-1) * imu_angular_scale * std::stol(param_vector[8]);
            imu_data.angular_velocity.z = (-1) * imu_angular_scale * std::stol(param_vector[6]);

            imu_pub.publish(imu_data);
        }
    }
    return 0;
}