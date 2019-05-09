#include <ros/ros.h>
#include <agent/commond.h>

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
    
};

bool Agent::commondCallback(agent::commond::Request & request, agent::commond::Response & response)
{
    ROS_INFO("here  %d", request.type);
    system("touch a.cc");

    switch(request.type)
    {
        case 0:

            break;
        default:
            break;
    }
    return true;
}

Agent::Agent()
{
    ros::param::get("~path", pathString);
    ROS_INFO_STREAM(pathString);
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