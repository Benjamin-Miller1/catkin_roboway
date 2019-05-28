#include <ros/ros.h>
#include <agent/commond.h>
#include <string>
#include <signal.h>

class Agent
{
public:
    Agent();
    ~Agent();
    void exec();
private:
    ros::NodeHandle node;
    bool commondCallback(agent::commond::Request & request, agent::commond::Response & response);
    std::string pathString;
    pid_t slam_fpid;
    pid_t moveBase_fpid;
    pid_t client_fpid;
    int status;
};

bool Agent::commondCallback(agent::commond::Request & request, agent::commond::Response & response)
{
    ROS_INFO("here  %d", request.type);
    switch(request.type)
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
            response.returntype = true;
            break;
        case 1:
            kill(slam_fpid, 15);
            response.returntype = true;
            status = 0;
            break;
        case 2:
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
            response.returntype = true;
            break;
        case 3:
            kill(client_fpid, 15);
            kill(moveBase_fpid, 15);
            status = 0;
            response.returntype = true;
            break;
        case 4:
            response.returntype = status;
            break;
        default:
            break;
    }
    return true;
}

Agent::Agent()
{
    status = 0;
}
Agent::~Agent()
{
}

void Agent::exec()
{
    ros::ServiceServer service = node.advertiseService("agent/commond", &Agent::commondCallback, this);
    ros::spin();
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "agent");
    Agent agent;
    agent.exec();
    return 0;
}