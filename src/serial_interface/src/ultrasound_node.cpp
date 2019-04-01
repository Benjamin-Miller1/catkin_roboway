#include <ros/ros.h>
#include <boost/asio/serial_port.hpp>
#include <boost/asio.hpp>
#include <string>
#include <vector>
#include <thread>
#include <std_msgs/Header.h>
#include <sensor_msgs/Range.h>
#include "nlohmann/json.hpp"
#include <fstream>
#include <tf/transform_broadcaster.h>
using namespace std;
using json = nlohmann::json;
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

class Ultrasound
{
public:
    Ultrasound();
    ros::NodeHandle node;
    std::vector<ros::Publisher> publisher_vector;
    void serial_read();
    void exec();
    void publish(int index, int distance);
    json config;
    int number;
};
Ultrasound::Ultrasound()
{
    string config_file;
    ros::param::get("~config_file", config_file);
    std::ifstream i;
    i.open(config_file);
    i >> config;
    number = config["number"];
    for(int i = 0; i < number; i++)
    {
        if(config["config"][i]["function"] == "obstacle")
            publisher_vector.push_back(node.advertise<sensor_msgs::Range>("/ultrasound" + std::to_string(i), 10));
    }
}
void Ultrasound::publish(int index, int distance)
{
    sensor_msgs::Range msg;
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "/ultrasound" + std::to_string(index);
    msg.header = header;
    msg.field_of_view = 1;
    msg.min_range = 0;
    msg.max_range = config["config"][index]["maxRange"];

    msg.range = distance / 1000.0;

    publisher_vector[index].publish(msg);
}
void Ultrasound::exec()
{
    std::thread publish_thread(&Ultrasound::serial_read, this);
    publish_thread.detach();
    ros::spin();
}
void Ultrasound::serial_read()
{
    boost::asio::io_service io;
	boost::asio::serial_port sp(io, "/dev/ttyS1");

    sp.set_option(boost::asio::serial_port::baud_rate(115200));
    sp.set_option(boost::asio::serial_port::flow_control());
    sp.set_option(boost::asio::serial_port::parity());
    sp.set_option(boost::asio::serial_port::stop_bits());
    sp.set_option(boost::asio::serial_port::character_size(8));

    boost::asio::streambuf sb;
    while(1)
    {
        std::size_t n = 0;
        try{
            n = boost::asio::read_until(sp, sb, '\n');
        }catch(boost::system::system_error& e)
        {
            return;
        }
        boost::asio::streambuf::const_buffers_type bufs = sb.data();
        std::string line(boost::asio::buffers_begin(bufs), boost::asio::buffers_begin(bufs) + n);
        sb.consume(n);
        //ROS_INFO_STREAM(line);
        if(line.substr(1, 10) == "ultrasound")
        {
            line = line.substr(line.find_first_of(',') + 1);
            std::vector<std::string> param_vector;
            Split(line, ",", param_vector);
            for(int i = 0; i < param_vector.size(); i++)
            {
                try {
                    int distance = 950;
                    if(param_vector[i] != "NA")
                        distance = std::stoi(param_vector[i]);
                    for(int j = 0; j < number; j++)
                    {
                        if((config["config"][j]["index"] == i) && (config["config"][j]["function"] == "obstacle")) //协议中的顺序对应到超声波frame上
                        {
                            this->publish(j, distance);
                            break;
                        }
                    }
                }catch(std::invalid_argument&)
                {
                    continue;
                }
            }
        }
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ultrasound_node");
    Ultrasound ultrasound;
    ultrasound.exec();
    return 0;
}