#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <agent/commond.h>
#include <client/commond.h>
#include "motor_control/motor_commond.h"
#include <string>
#include <signal.h>
#include <unistd.h>
#include <netdb.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <thread>
#include "nlohmann/json.hpp"
using json = nlohmann::json;
using namespace std;

class Agent
{
public:
    Agent();
    ~Agent();
    void exec();
private:
    ros::NodeHandle node;
    ros::ServiceClient motor_control_client;
    ros::ServiceClient client_client;
    void RPC_sendStatus(int status);

    bool commondCallback(agent::commond::Request & request, agent::commond::Response & response);
    bool dealGui(agent::commond::Request & request, agent::commond::Response & response);
    bool dealKeyboard(int value);
    bool dealClient(int value);
    void startNavigation();
    void stopNavigation();

    pid_t slam_fpid;
    pid_t moveBase_fpid;
    pid_t client_fpid;
    int status;

    string carId;
    bool isSendMsgToServer;
    string serveraddrstr;
    string serverportstr;
    int sockfd = -1;

    string startPoint_longitude;
    string startPoint_latitude;
    string endPoint_longitude;
    string endPoint_latitude;
    string realTimePoint_longitude;
    string realTimePoint_latitude;
    double remainTimeInMin;

    enum status_T {START, END, STARTTOEND, ENDTOSTART, STOP, PARKING, PARKINGTOSTART};
    status_T status_navigation{PARKINGTOSTART};

    void init_socket();
    void readCommond();
    void send_started();
    void send_reachStartPoint();
    void send_realTimePoint(const ros::TimerEvent&);
    void send_reachEndPoint();

    void gps_callback(const geometry_msgs::Pose::ConstPtr& msg);

    string mapName;
    string currentMapName;
};

void Agent::init_socket()
{
    /*1.创建socket*/
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if(sockfd < 0)
    {
		perror("socket error");
		exit(1);
    }
    
    /*向serveraddr中填入服务器端ip、port和地址族类型(IPV4)*/
    struct sockaddr_in serveraddr;
    memset(&serveraddr, 0, sizeof(serveraddr));
    serveraddr.sin_family = AF_INET;
    serveraddr.sin_port = htons(atoi(serverportstr.data()));
    //将IP地址转换成网络字节序后填入serveraddr中
    inet_pton(AF_INET, serveraddrstr.data(), &serveraddr.sin_addr.s_addr);
 
    /*2.客户端调用connect函数连接到服务器端*/
    if(connect(sockfd, (struct sockaddr*)&serveraddr, sizeof(serveraddr)) < 0)
    {
        ROS_INFO("connect server fail!");
    }
}
void Agent::RPC_sendStatus(int status)
{
    ROS_INFO("sendStatus to client %d", status);
    client::commond srv;
    srv.request.value = status;
    client_client.call(srv);
}

void Agent::startNavigation()
{
    //odom清零
    motor_control::motor_commond motor_commond_srv;
    motor_commond_srv.request.commond = 3;
    motor_control_client.call(motor_commond_srv);

    //修改launch文件中地图相关配置
    std::stringstream ss;
    ss << "python /home/roboway/catkin_roboway/src/bringup/script/modify_launch.py " << carId << " " << mapName;
    std::string commond = ss.str();
    system(commond.c_str());
    currentMapName = mapName;

    //启动导航相关进程
    moveBase_fpid = fork();
    ROS_INFO("moveBase pid   %d", moveBase_fpid);
    if(moveBase_fpid == 0)//子进程
    {
        if(execlp("roslaunch", "roslaunch", "bringup", "move_base.launch", NULL) < 0 )
        {
            ROS_INFO("execlp error");
            exit(1);
        }
    }

    client_fpid = fork();
    ROS_INFO("client pid   %d", client_fpid);
    if(client_fpid == 0)//父进程
    {
        if(execlp("roslaunch", "roslaunch", "bringup", "client.launch", NULL) < 0 )
        {
            ROS_INFO("execlp error");
            exit(1);
        }
    }

    status = 2;
}

void Agent::stopNavigation()
{
    if(client_fpid != -1)
    {
        kill(client_fpid, 15);
        client_fpid = -1;
        kill(moveBase_fpid, 15);
        moveBase_fpid = -1;
    }
    status = 0;
    status_navigation = PARKINGTOSTART;
    mapName.clear();
}

void Agent::readCommond()
{
    char buf[1024] = {0};
    int size;
    while((size = read(sockfd, buf, sizeof(buf))) > 0)
    {
        try{
            json commond = json::parse(buf);
            ROS_INFO_STREAM(commond);
            if(commond["communicationId"] != carId)
                continue;
            auto commandType = commond["commandType"];
            if(commandType == "connection")
            {
                startPoint_longitude = commond["startPoint"]["longitude"];
                startPoint_latitude = commond["startPoint"]["latitude"];
                endPoint_longitude = commond["endPoint"]["longitude"];
                endPoint_latitude = commond["endPoint"]["latitude"];

                ROS_INFO("car has connect to server, running to start");
            }
            else if(commandType == "start")
            {
                //接收mapName,同时启动导航进程;
                
                status_navigation = STARTTOEND;
                if(client_fpid == -1)//第一次需要启动导航相关进程, 不能发送RPC消息
                {
                    startNavigation();
                }
                else//如果启动了,查看是否需要更换地图, 如果不更换说明走了一个循环,则需要发送RPC/
                {
                    if(currentMapName == mapName)
                        RPC_sendStatus(status_navigation);//通知client节点
                    else
                    {
                        stopNavigation();
                        startNavigation();
                    }
                }
                send_started();//通知后台server
                ROS_INFO("running to end");
            }
            else if(commandType == "back")
            {
                //返回到起点
                if(status_navigation == END || status_navigation == STOP)
                {
                    status_navigation = ENDTOSTART;
                    RPC_sendStatus(status_navigation);//通知client节点
                    ROS_INFO("running to start");
                }
            }
            else if(commandType == "parking")
            {
                //暂时不处理
            }

            memset(buf, 0, sizeof(buf));
        }
        catch (json::exception& e)
        {
            // output exception information
            ROS_INFO_STREAM("parse failure");
            continue;
        }
    }
}
void Agent::send_started()
{
    if(!isSendMsgToServer)
        return;
    json commond;
    commond["position"]["longitude"] = startPoint_longitude;
    commond["position"]["latitude"] = startPoint_latitude;
    commond["location"] = "started";
    commond["communicationId"] = carId;
    commond["arriveTime"] = std::to_string(remainTimeInMin);

    string commondString = commond.dump();
    commondString = "$" + commondString + "$";
    write(sockfd, commondString.c_str(), commondString.size());
    ROS_INFO_STREAM(commond);
}

void Agent::send_reachStartPoint()
{
    if(!isSendMsgToServer)
        return;
    json commond;
    commond["position"]["longitude"] = startPoint_longitude;
    commond["position"]["latitude"] = startPoint_latitude;
    commond["location"] = "startPoint";
    commond["communicationId"] = carId;

    string commondString = commond.dump();
    commondString = "$" + commondString + "$";
    write(sockfd, commondString.c_str(), commondString.size());
    ROS_INFO_STREAM(commond);
}
void Agent::send_realTimePoint(const ros::TimerEvent&)
{
    if((status != STARTTOEND) || (status != ENDTOSTART))
        return;
    if(!isSendMsgToServer)
        return;
    json commond;
    commond["position"]["longitude"] = realTimePoint_longitude;
    commond["position"]["latitude"] = realTimePoint_latitude;
    commond["location"] = "running";
    commond["communicationId"] = carId;
    commond["arriveTime"] = std::to_string(remainTimeInMin);

    string commondString = commond.dump();
    commondString = "$" + commondString + "$";
    write(sockfd, commondString.c_str(), commondString.size());
    ROS_INFO_STREAM(commond);
}
void Agent::send_reachEndPoint()
{
    if(!isSendMsgToServer)
        return;
    json commond;
    commond["position"]["longitude"] = endPoint_longitude;
    commond["position"]["latitude"] = endPoint_latitude;
    commond["location"] = "endPoint";
    commond["communicationId"] = carId;

    string commondString = commond.dump();
    commondString = "$" + commondString + "$";
    write(sockfd, commondString.c_str(), commondString.size());
    ROS_INFO_STREAM(commond);
}
void Agent::gps_callback(const geometry_msgs::Pose::ConstPtr& msg)
{
    double latitude = std::stod(startPoint_latitude);
    double longitude = std::stod(startPoint_longitude);
    
    if(std::abs(msg->position.x - latitude ) > 1 || std::abs(msg->position.y - longitude ) > 1)
    {
        realTimePoint_latitude = startPoint_latitude;
        realTimePoint_longitude = startPoint_longitude;
    }
    else
    {
        realTimePoint_latitude = std::to_string(msg->position.x);
        realTimePoint_longitude = std::to_string(msg->position.y);
    }
}
bool Agent::dealGui(agent::commond::Request & request, agent::commond::Response & response)
{
    switch(request.value)
    {
        case 0:
            slam_fpid = fork();
            ROS_INFO("fork pid   %d", slam_fpid);
            if(slam_fpid == 0)//子进程
            {
                if(execlp("roslaunch", "roslaunch", "bringup", "slam.launch", NULL) < 0 )
                {
                    ROS_INFO("execlp error");
                    exit(1);
                }
            }
            status = 1;
            response.returnValue = true;
            break;
        case 1:
            kill(slam_fpid, 15);
            slam_fpid = 0;
            status = 0;
            response.returnValue = true;
            break;
        case 2:
            //GUI启动了导航, 说明车子已在起点,通知server
            mapName = request.mapName;
            if(status_navigation == PARKINGTOSTART)
            {
                send_reachStartPoint();
                status_navigation = START;
                response.returnValue = true;
            }
            break;
        case 3:
            stopNavigation();
            response.returnValue = true;
            break;
        case 4:
            response.returnValue = std::stoi(carId);
            response.returnString = carId;
            break;
        default:
            break;
    }
    return true;
}
bool Agent::dealKeyboard(int value)
{
    if(value == 1)//出发
    {
        if(status_navigation == START || status_navigation == STOP)
        {
            status_navigation = STARTTOEND;
            if(client_fpid == -1)//第一次需要启动导航相关进程, 不能发送RPC消息
            {
                startNavigation();
            }
            else//如果启动了,查看是否需要更换地图, 如果不更换说明走了一个循环,则需要发送RPC/
            {
                if(currentMapName == mapName)
                    RPC_sendStatus(status_navigation);//通知client节点
                else
                {
                    stopNavigation();
                    startNavigation();
                }
            }
            send_started();//通知后台server
        }
    }
    else if(value == 2) //返回
    {
        if(status_navigation == END || status_navigation == STOP)
        {
           status_navigation = ENDTOSTART;
           RPC_sendStatus(status_navigation);//通知client节点
        }
    }
    else if(value == 3)//导航停止
    {
        if(status_navigation == STARTTOEND || status_navigation == ENDTOSTART)
        {
            status_navigation = STOP;
        }
    }
}
bool Agent::dealClient(int value)
{
    status_T status = static_cast<status_T>(value);
    switch (status)
    {
        case END:
            status_navigation = status;
            send_reachEndPoint();
            break;
        case START:
            status_navigation = status;
            send_reachStartPoint();
            break;
        default:
            break;
    }
    return true;
}
bool Agent::commondCallback(agent::commond::Request & request, agent::commond::Response & response)
{
    ROS_INFO("agent receive commond  %d, %d ", request.type, request.value);

    switch(request.type)
    {
        case 0:
            dealGui(request, response);
            break;
        case 1:
            dealKeyboard(request.value);
            break;
        case 2:
            dealClient(request.value);
            break;
        default:
            break;
    }
    return true;
}

Agent::Agent()
{
    status = 0;
    moveBase_fpid = -1;
    client_fpid = -1;

    ros::param::get("~carId", carId);
    ros::param::get("~isSendMsgToServer", isSendMsgToServer);
    ros::param::get("~ipaddress", serveraddrstr);
    ros::param::get("~port", serverportstr);
    
    realTimePoint_longitude = "";
    realTimePoint_latitude = "";

    if(isSendMsgToServer)
    {
        init_socket();
        string startCode = "$@@@@" + carId + "$";
        const char* startCode_ = startCode.c_str();
        write(sockfd, startCode_, strlen(startCode_));
    }
}
Agent::~Agent()
{
    close(sockfd);
}

void Agent::exec()
{
    ros::ServiceServer service = node.advertiseService("agent/commond", &Agent::commondCallback, this);
    client_client = node.serviceClient<client::commond>("client/commond");
    motor_control_client = node.serviceClient<motor_control::motor_commond>("motor_control/commond");
    ros::Subscriber cmd_sub = node.subscribe<geometry_msgs::Pose>("/gps", 10, &Agent::gps_callback, this);
    ros::Timer sendRealTimePointTimer = node.createTimer(ros::Duration(5), &Agent::send_realTimePoint, this);

    std::thread receive_thread(&Agent::readCommond, this);
    receive_thread.detach();

    ros::spin();
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "agent");
    Agent agent;
    agent.exec();
    return 0;
}