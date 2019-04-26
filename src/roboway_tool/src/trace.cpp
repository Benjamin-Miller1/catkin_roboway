#include <tf/transform_listener.h>

#include "tf/transform_datatypes.h"
#include <tf2/utils.h>
#include <fstream>
#include <vector>
using namespace std;

struct Point;
struct Point
{
    Point(double x_ = 0, double y_ = 0):x(x_), y(y_){};
    Point(const Point &point){x = point.x; y = point.y;};
    
    double x;//单位分米
    double y;
};

bool isRepeat(vector<Point> &point_vector , const Point &point)
{
    if(point_vector.size() == 0)
        return false;

    float distance = (point.x - point_vector.back().x) * (point.x - point_vector.back().x) 
                    + (point.y - point_vector.back().y) * (point.y - point_vector.back().y);

    if(distance > 0.01)
        return false;
    else
        return true;
}
int main(int argc, char **argv)
{
    if(argc < 2)
    {
        ROS_INFO("missing parameter of map name");
        exit(-1);
    }
    ros::init(argc, argv, "client");
    std::stringstream ss;
    ss << "/home/roboway/workspace/catkin_roboway/src/bringup/map/" << argv[1] << "_trace.dat";
    ofstream fout(ss.str().c_str(), ios::binary | ios::trunc);
    tf::TransformListener listener;
    tf::StampedTransform base_link_pose;
    vector<Point> point_vector;

    ros::Rate loop_rate(2);

    while(ros::ok())
    {
        try{
            listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(10.0));
            listener.lookupTransform("map", "base_link", ros::Time(0), base_link_pose);
                                //ros::Time(0)表示最近的一帧坐标变换，不能写成ros::Time::now()
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
        }
        ROS_INFO_STREAM("x="<<base_link_pose.getOrigin().x()<<",y="<<base_link_pose.getOrigin().y());
        Point point(base_link_pose.getOrigin().x(), base_link_pose.getOrigin().y());
        if(!isRepeat(point_vector, point))
        {
            point_vector.push_back(point);

            fout.write((char*)&(point.x),sizeof(point.x));
            fout.write((char*)&(point.y),sizeof(point.y));
            fout.flush();
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }

}