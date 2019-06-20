#include <ros/ros.h>
#include <client/commond.h>
#include <agent/commond.h>
#include "nlohmann/json.hpp"
using json = nlohmann::json;
#include <thread>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <vector>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>

#include "tf/transform_datatypes.h"
#include <tf2/utils.h>

using namespace std;

struct Pose
{
    Pose():x(0), y(0), yaw(0), type(-1){}
    double x;
    double y;
    double yaw;
    int type;//0:起点 1:终点 -1:普通
};

class Client
{
public:
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

    Client();
    ros::NodeHandle node;
    void exec();
    bool commondCallback(client::commond::Request & request, client::commond::Response & response);
    void get_pose_callback();
    ~Client();
private:
    enum status_T
    {START, END, STARTTOEND, ENDTOSTART, STOP, PARKING, PARKINGTOSTART};
    status_T status{START};//启动了导航业务意味着在起点
    ros::ServiceClient agent_client;

    vector<Pose> pathVector;
    void createPath();
    int moveSinglePath(vector<Pose> &singlePath);
    void IPC_reachEndPoint();
    void IPC_reachStartPoint();
    void navigation();
    MoveBaseClient *ac{nullptr};

    tf::StampedTransform base_link_pose;
    double pi = 3.1415;
};

void Client::get_pose_callback()
{   
    tf::TransformListener listener;
    while(true)
    {
        try{
            listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(10.0));
            listener.lookupTransform("map", "base_link", ros::Time(0), base_link_pose);
                                //ros::Time(0)表示最近的一帧坐标变换，不能写成ros::Time::now()
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
        }

        double roll, pitch, yaw;
        tf::Matrix3x3(base_link_pose.getRotation()).getRPY(roll, pitch, yaw);

        //ROS_INFO_STREAM("x="<<base_link_pose.getOrigin().x()<<",y="<<base_link_pose.getOrigin().y() << ", yaw=" << yaw);
        ros::Duration(0.1).sleep();
    }
}

int Client::moveSinglePath(vector<Pose> &singlePath)
{
	int goalIndex = 0;
	while(1 && (status == STARTTOEND || status == ENDTOSTART))
	{
        //设置目标
	    move_base_msgs::MoveBaseGoal goal;
	    goal.target_pose.header.frame_id = "map";
	    goal.target_pose.header.stamp = ros::Time::now();
	    goal.target_pose.pose.position.x = singlePath[goalIndex].x;
	    goal.target_pose.pose.position.y = singlePath[goalIndex].y;
	    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(singlePath[goalIndex].yaw);
	    ROS_INFO("Sending goal  x=%f, y=%f, yaw=%f", singlePath[goalIndex].x,singlePath[goalIndex].y, singlePath[goalIndex].yaw);
	    while(1)
	    {
	        ac->sendGoal(goal);
	        // Wait for the action to return
	        while(!ac->waitForResult(ros::Duration(1.0)))//一直检测当前的状态
	        {
                if(status == STOP)
                {
                    //取消导航
                    ac->cancelGoal();
                    while(status == STOP)
                    {
                        ros::Duration(1.0).sleep();
                        ROS_INFO("stopping");
                    }
                }
                double roll, pitch, yaw;
                tf::Matrix3x3(base_link_pose.getRotation()).getRPY(roll, pitch, yaw);

                //计算剩余时间
                double remainDistance = 0;
                remainDistance += std::sqrt((base_link_pose.getOrigin().x() - singlePath[goalIndex].x) * (base_link_pose.getOrigin().x() - singlePath[goalIndex].x)
                                      + (base_link_pose.getOrigin().y() - singlePath[goalIndex].y) * (base_link_pose.getOrigin().y() - singlePath[goalIndex].y));
                for(int goalPoint = goalIndex; goalPoint < singlePath.size() - 1; goalPoint++)
                {
                    remainDistance += std::sqrt((singlePath[goalIndex + 1].x - singlePath[goalIndex].x) * (singlePath[goalIndex + 1].x - singlePath[goalIndex].x)
                                      + (singlePath[goalIndex + 1].y - singlePath[goalIndex].y) * (singlePath[goalIndex + 1].y - singlePath[goalIndex].y));
                }
                double remainTimeInMin = remainDistance / 0.5 / 60;

                ROS_INFO_STREAM("running" << "x:" << base_link_pose.getOrigin().x() << " y:" << base_link_pose.getOrigin().y() << " yaw:" << yaw << " remainTime: " << remainDistance / 0.5);
	        }
	        ROS_INFO("finish one point");

	        if(ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	        {
	            ROS_INFO("reach one point");
	            if(goalIndex == singlePath.size() - 1)//如果是走到了最后一个点,发送消息到后台
	            {
                    return 0;
	            }
	            else //下一个点
	            {
	                goalIndex++;
	                break;
	            }
	        }
	    }
	}
}

void Client::IPC_reachEndPoint()
{
    agent::commond srv;
    srv.request.type = 2;
    srv.request.value = status;
    agent_client.call(srv);
}
void Client::IPC_reachStartPoint()
{
    agent::commond srv;
    srv.request.type = 2;
    srv.request.value = status;
    agent_client.call(srv);
}
void Client::navigation()
{
    status_T status_last;
    status_T status_current;

    while(1)
    {
        if(status == STARTTOEND)
        {
            vector<Pose> singlePathVector;
            for(int i = 1; i < pathVector.size(); i++)
      		{
      			singlePathVector.push_back(pathVector[i]);
      			if(pathVector[i].type == 1)//该点为终点，退出循环
      				break;
      		}
            
 			//开始行走
            if(moveSinglePath(singlePathVector) == 0)
            {
                ROS_INFO("reach end and change status to end");
                status = END;
                IPC_reachEndPoint();
            }
        }

        if(status == ENDTOSTART)
        {
        	vector<Pose> singlePathVector;
            int endpoint = 0;
            for(int i = 0; i < pathVector.size(); i++)
      		{
                if(pathVector[i].type == 1)//如果是终点
                {
                    endpoint = i;
                    break;
                }
      		}
            
            for(int i = endpoint + 1; i < pathVector.size(); i++)
            {
                singlePathVector.push_back(pathVector[i]);
                if(pathVector[i].type == 0)//该点为起点，退出循环
                    break;
            }

 			//开始行走
            if(moveSinglePath(singlePathVector) == 0)
            {
                ROS_INFO("reach start and change status to start");
                status = START;
                IPC_reachStartPoint();
            }

        }
        ros::Duration(0.1).sleep();
    }
}

void Client::createPath()
{
    json path_json;
    string path_file;
    ros::param::get("~path_file", path_file);
    std::ifstream i;
    i.open(path_file);
    i >> path_json;
    int number = path_json["number"];
    for(int i = 0; i < number; i++)
    {
        Pose pose;
        pose.x = path_json["pose"][i]["x"];
        pose.y = path_json["pose"][i]["y"];
        pose.yaw = path_json["pose"][i]["yaw"];
        pose.type = path_json["pose"][i]["type"];
        pathVector.push_back(pose);
        ROS_INFO("%f, %f, %f, %d  ", pose.x, pose.y, pose.yaw, pose.type);
    }

}
bool Client::commondCallback(client::commond::Request & request, client::commond::Response & response)
{
    ROS_INFO("client receive commond %d", request.value);
    status = static_cast<status_T>(request.value);
}

Client::Client()
{
}

void Client::exec()
{
    createPath();

    ac = new MoveBaseClient("move_base", true);

    ac->waitForServer();

    std::thread navigation_thread(&Client::navigation, this);
    navigation_thread.detach();

    std::thread pose_thread(&Client::get_pose_callback, this);
    pose_thread.detach();

    ros::ServiceServer service = node.advertiseService("client/commond", &Client::commondCallback, this);
    agent_client = node.serviceClient<agent::commond>("agent/commond");

    IPC_reachStartPoint();

    ros::spin();
}

Client::~Client()
{
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "client");
    Client client;
    client.exec();
    return 0;
}