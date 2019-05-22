#include <ros/ros.h>
#include <agent/commond.h>

#include <QProcess>

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
    QProcess *slamProcess;
    QProcess *moveBaseProcess;
    QProcess *clientProcess;
    int status;
};

bool Agent::commondCallback(agent::commond::Request & request, agent::commond::Response & response)
{
    ROS_INFO("here  %d", request.type);

    switch(request.type)
    {
        case 0:
            if(QProcess::NotRunning == slamProcess->state())
            {
                slamProcess->start("roslaunch bringup slam.launch");
                status = 1;
            }
            response.returntype = true;
            break;
        case 1:
            if(QProcess::NotRunning != slamProcess->state())
            {
                slamProcess->terminate();
                slamProcess->waitForFinished();
                status = 0;
            }
            response.returntype = true;
            break;
        case 2:
            if(QProcess::NotRunning == moveBaseProcess->state())
            {
                moveBaseProcess->start("roslaunch bringup move_base.launch");
                clientProcess->start("roslaunch bringup client.launch");
                status = 2;
            }
            response.returntype = true;
            break;
        case 3:
            if(QProcess::NotRunning != moveBaseProcess->state())
            {
                clientProcess->terminate();
                clientProcess->waitForFinished();
                moveBaseProcess->terminate();
                moveBaseProcess->waitForFinished();
                status = 0;
            }
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
    slamProcess = new QProcess();
    moveBaseProcess = new QProcess();
    clientProcess = new QProcess();
}
Agent::~Agent()
{
    if(QProcess::NotRunning != slamProcess->state())
    {
        slamProcess->terminate();
        slamProcess->waitForFinished();
    }
    delete slamProcess;
    if(QProcess::NotRunning != clientProcess->state())
    {
        clientProcess->terminate();
        clientProcess->waitForFinished();
    }
    delete clientProcess;

    if(QProcess::NotRunning != moveBaseProcess->state())
    {
        moveBaseProcess->terminate();
        moveBaseProcess->waitForFinished();
    }
    delete moveBaseProcess;
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