#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <agent/commond.h>
#include <client/commond.h>
#include "motor_control/motor_commond.h"
#include <string>
#include <iomanip>
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
    bool dealClient(agent::commond::Request & request);

    double start_x{0};
    double start_y{0};
    double start_yaw{0};
    
    void startNavigation();
    void stopNavigation();

    pid_t slam_fpid;
    pid_t moveBase_fpid;
    pid_t client_fpid;
    int status;

    string carId;
    bool isSendMsgToServer;
    bool isOnline;
    bool waitForHeartBeat{false};

    string serveraddrstr;
    string serverportstr;
    int sockfd = -1;

    string startPoint_longitude;
    string startPoint_latitude;
    string endPoint_longitude;
    string endPoint_latitude;
    string realTimePoint_longitude;
    string realTimePoint_latitude;
    int remainTimeInMin;

    enum status_T {START, END, STARTTOEND, ENDTOSTART, STOP, PARKING, PARKINGTOSTART};
    status_T status_navigation{PARKINGTOSTART};

    bool init_socket();
    void readCommond();
    void send_carID();
    void send_started();
    void send_reachStartPoint();
    void send_running();
    void send_reachEndPoint();
    void send_heartBeat(const ros::TimerEvent&);

    void gps_callback(const geometry_msgs::Pose::ConstPtr& msg);

    string mapName;
    string currentMapName;
};

bool Agent::init_socket()
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
        close(sockfd);
        sockfd = -1;
        return false;
    }
    else
    {
        ROS_INFO("connect server success!");
        return true;
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
    {
        std::stringstream ss;
        ss << "python /home/roboway/catkin_roboway/src/bringup/script/modify_launch.py " << carId << " " << mapName;
        std::string commond = ss.str();
        system(commond.c_str());
        currentMapName = mapName;
    }
    
    //修改amcl的初始位置
    {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2);
        ss << "python /home/roboway/catkin_roboway/src/bringup/script/modify_amcl.py " << start_x << " " << start_y << " " << start_yaw;
        std::string commond = ss.str();
        system(commond.c_str());
    }

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
}

void Agent::readCommond()
{
    char buf[1024] = {0};
    int size;
    static bool isFirstConnect = true;
    while(1)
    {
        if(sockfd == -1)
        {
            ros::Duration(1).sleep();
            ROS_INFO_STREAM("sockfd = -1");
            continue;
        }

        if((size = read(sockfd, buf, sizeof(buf))) > 0)
        {
            try{
                json commond = json::parse(buf);
                ROS_INFO_STREAM(commond);
                auto commandType = commond["commandType"];
                if(commandType == "connection")
                {
                    ROS_INFO("car has connect to server, running to start");
                    send_carID();
                    if(isFirstConnect)
                    {
                        isFirstConnect = false;
                        send_reachEndPoint();//车子启动后更新后台状态为终点,防止后台认为车子在起点导致可以发送start命令,在gui的启动业务后会向服务器发startpoint.
                    }
                }
                else if(commandType == "start")
                {
                    //接收mapName,同时启动导航进程;
                    mapName = commond["mapNumber"];
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
                else if(commandType == "heartbeat")
                {
                    waitForHeartBeat = false;
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
        else
        {
            ROS_INFO_STREAM("read return " << size);
            ros::Duration(0.1).sleep();
        }
    }
}
void Agent::send_carID()
{
    if(!isSendMsgToServer)
        return;
    json commond;
    commond["communicationId"] = carId;
    string commondString = commond.dump();
    write(sockfd, commondString.c_str(), commondString.size());
    ROS_INFO_STREAM(commond);
}

void Agent::send_started()
{
    if(!isSendMsgToServer)
        return;
    json commond;
    commond["position"]["longitude"] = startPoint_longitude;
    commond["position"]["latitude"] = startPoint_latitude;
    commond["carOperationState"] = "started";
    commond["communicationId"] = carId;
    commond["arriveTime"] = std::to_string(remainTimeInMin);

    string commondString = commond.dump();
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
    commond["carOperationState"] = "startPoint";
    commond["communicationId"] = carId;

    string commondString = commond.dump();
    write(sockfd, commondString.c_str(), commondString.size());
    ROS_INFO_STREAM(commond);
}
void Agent::send_running()
{
    if(!isSendMsgToServer)
        return;
    json commond;
    commond["position"]["longitude"] = realTimePoint_longitude;
    commond["position"]["latitude"] = realTimePoint_latitude;
    commond["carOperationState"] = "running";
    commond["communicationId"] = carId;

    string commondString = commond.dump();
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
    commond["carOperationState"] = "endPoint";
    commond["communicationId"] = carId;

    string commondString = commond.dump();
    write(sockfd, commondString.c_str(), commondString.size());
    ROS_INFO_STREAM(commond);
}
void Agent::send_heartBeat(const ros::TimerEvent&)
{
    static int waitForHeartBeatTimes = 0;
    if(!isSendMsgToServer)
        return;
    if(waitForHeartBeat)
    {
        waitForHeartBeatTimes++;
        if(waitForHeartBeatTimes >= 2)
        {
            //中断
            ROS_INFO("tcp disconnected!");
            if(sockfd != -1)
            {
                shutdown(sockfd,SHUT_RD);//可以让阻塞的read函数返回
                sockfd = -1;
            }

            if(init_socket() == true)//恢复连接后将标志位置位
            {
                waitForHeartBeat = false;
                waitForHeartBeatTimes = 0;
            }
        }
        return;
    }
    waitForHeartBeatTimes = 0;
    waitForHeartBeat = true;
    json commond;
    
    commond["carOperationState"] = "heartbeat";
    commond["communicationId"] = carId;

    string commondString = commond.dump();
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
            start_x = 0;
            start_y = 0;
            start_yaw = 0;
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
bool Agent::dealClient(agent::commond::Request & request)
{
    if(request.value > PARKINGTOSTART)//client去终点后回应的剩余时间
    {
        remainTimeInMin = request.x;
        send_started();//通知后台server
        send_running();
        ROS_INFO("running to end");
        return true;
    }
    status_T status = static_cast<status_T>(request.value);
    switch (status)
    {
        case END:
            status_navigation = status;
            send_reachEndPoint();
            break;
        case START:
            status_navigation = status;
            start_x = request.x;
            start_y = request.y;
            start_yaw = request.yaw;
            ROS_INFO_STREAM("x: " << start_x << " y: " << start_y << " yaw: " << start_yaw);

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
            dealClient(request);
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
        isOnline = true;
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
    ros::Timer sendHeartBeatTimer = node.createTimer(ros::Duration(5), &Agent::send_heartBeat, this);

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