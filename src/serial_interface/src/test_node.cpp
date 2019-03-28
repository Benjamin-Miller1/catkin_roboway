#include "nlohmann/json.hpp"
#include <fstream>
#include <ros/ros.h>
#include <iostream>
using json = nlohmann::json;
using namespace std;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "test");
    std::ifstream i;
    json config;
    i.open("/home/autolabor/code/catkin_roboway/src/bringup/param/ultrasound.json");

    i >> config;
    cout << config;
    return 0;
}