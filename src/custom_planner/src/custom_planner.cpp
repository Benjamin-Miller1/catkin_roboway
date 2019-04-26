#include <custom_planner/custom_planner.h>
#include <pluginlib/class_list_macros.h>

#include "nlohmann/json.hpp"
using json = nlohmann::json;
#include <fstream>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(custom_planner::CustomPlanner, nav_core::BaseGlobalPlanner)

namespace custom_planner {

  CustomPlanner::CustomPlanner()
  : costmap_ros_(NULL){}

  CustomPlanner::CustomPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  : costmap_ros_(NULL){
    initialize(name, costmap_ros);
  }
  
  void CustomPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  {

    json path_json;
    string path_file;
    ros::param::get("~path_file", path_file);
    ROS_INFO_STREAM("path_file: " << path_file);
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
        if(pose.type == 0)
          startPoint = pose;
        else if(pose.type == 1)
          endPoint = pose;
    }
    ROS_INFO("create a path");
  }

  bool CustomPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
      const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){

    static int status = 1;  //0去往终点 1去往起点
    static Pose lastPose;//为了使方向不发生改变，保留经去的最近的一个点
    plan.clear();

    if(goal.pose.position.x == endPoint.x && goal.pose.position.y == endPoint.x)
    {
      if(status == 1)
      {
        status = 0;
        for(int i = 1; i < pathVector.size(); i++)
        {
          planVector.push_back(pathVector[i]);
          if(pathVector[i].type == 1)//该点为终点，退出循环
            break;
        }
        lastPose = pathVector[0];
      }
    }

    if(goal.pose.position.x == startPoint.x && goal.pose.position.y == startPoint.x)
    {
      if(status == 0)
      {
        status = 1;
        //找到终点
        int endpoint = 0;
        for(int i = 0; i < pathVector.size(); i++)
        {
            if(pathVector[i].type == 1)//如果是终点
            {
                endpoint = i;
                break;
            }
        }
        //创建总路径
        for(int i = endpoint + 1; i < pathVector.size(); i++)
        {
            planVector.push_back(pathVector[i]);
            if(pathVector[i].type == 0)//该点为起点，退出循环
                break;
        }
        lastPose = pathVector[endpoint];
        lastPose.yaw = atan2(pathVector[endpoint + 1].y - pathVector[endpoint].y , pathVector[endpoint + 1].x - pathVector[endpoint].x);
      }
    }

    tf::Stamped<tf::Pose> tf_pose;
    tf::poseStampedMsgToTF(start, tf_pose);
    double currentYaw = tf::getYaw(tf_pose.getRotation());
    ROS_INFO("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

    //每经过一个点删除最前的点，更新plan
    if(planVector[0] != endPoint || planVector[0] != startPoint)
    {
      ROS_INFO_STREAM("current yaw: " << currentYaw << " goal yaw: " << planVector[0].yaw);
      if(std::abs(start.pose.position.x - planVector[0].x) < 1 && std::abs(start.pose.position.y - planVector[0].y) < 1)
      {
          lastPose = planVector[0];
          planVector.erase(planVector.begin());
      }
      else
      {
        //如果跳过了一个目标（没有检测到与某一个目标距离在0.5m以内），如何处理
      }
    }
 
    geometry_msgs::PoseStamped pose_;
    pose_.pose.position.x = start.pose.position.x;
    pose_.pose.position.y = start.pose.position.y;
    pose_.pose.orientation = start.pose.orientation;
    //pose_.pose.orientation = tf::createQuaternionMsgFromYaw(lastPose.yaw);//需要注意
    plan.push_back(pose_);

    for(auto pose : planVector)
    {
      pose_.pose.position.x = pose.x;
      pose_.pose.position.y = pose.y;
      pose_.pose.orientation = tf::createQuaternionMsgFromYaw(pose.yaw);
      plan.push_back(pose_);
    }

    return true;
  }

};
