#include "motor_control/motor_driver.h"
#include "motor_control/motor_control.h"
#include <tf2_ros/transform_broadcaster.h>

#include <thread>
#include <numeric>
#include <fstream>
#include <boost/bind.hpp>
#include <iostream>
#include <string.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

CanDevice::CanDevice(const char* devname):
    local_socket_(::socket(PF_CAN, SOCK_RAW, CAN_RAW)),
    interface_name_(devname), timer(io_service)
{
    if (local_socket_ < 0) {
        throw std::system_error(errno, std::system_category());
    }
    asio::generic::raw_protocol raw_protocol(PF_CAN, CAN_RAW);
    raw_socket = std::make_shared<asio::generic::raw_protocol::socket>(io_service, raw_protocol);
    raw_socket->bind(endpoint());
}

CanDevice::protocol::endpoint CanDevice::endpoint() const
{
    struct ifreq ifr;

    std::strncpy(ifr.ifr_name, interface_name_.c_str(), IF_NAMESIZE);
    ifr.ifr_name[IF_NAMESIZE - 1] = '\0';
    ioctl(local_socket_, SIOCGIFINDEX, &ifr);

    struct sockaddr_can addr = {0};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    return protocol::endpoint(&addr, sizeof(sockaddr_can));
}

void CanDevice::readCompleted(const boost::system::error_code& error,
        const size_t bytesTransferred)
{
    if(!error)
    {
        result=resultSuccess;
        this->bytesTransferred=bytesTransferred;
        return;
    }
    result=resultError;
}
void CanDevice::timeoutExpired(const boost::system::error_code& error)
{
     if(!error && result==resultInProgress) result=resultTimeoutExpired;
}
void CanDevice::write(const char *data, size_t size)
{
    raw_socket->send(asio::buffer(data,size));
}
bool CanDevice::read(char *data, size_t size)
{
    raw_socket->async_receive(asio::buffer(data, size), boost::bind(
                &CanDevice::readCompleted,this,asio::placeholders::error,
                asio::placeholders::bytes_transferred));
    timer.expires_from_now(boost::posix_time::millisec(100));
    timer.async_wait(boost::bind(&CanDevice::timeoutExpired,this, asio::placeholders::error));

    result=resultInProgress;
    bytesTransferred=0;
    for(;;)
    {
        io_service.run_one();
        switch(result)
        {
            case resultSuccess:
                timer.cancel();
                return true;
            case resultTimeoutExpired:
                //raw_socket->cancel();
                return false;
            case resultError:
                timer.cancel();
                //raw_socket->cancel();
                return false;
        }
    }
}

CanDevice::~CanDevice()
{
    if (local_socket_ >= 0)
        ::close(local_socket_);
}

bool print_can_frame(const struct can_frame * const frame)
{
    const unsigned char *data = frame->data;
    const unsigned int dlc = frame->can_dlc;
    unsigned int i;

    printf("%03X  [%u] ", frame->can_id, dlc);
    for (i = 0; i < dlc; ++i)
    {
        printf(" %02X", data[i]);
    }
    printf("\r\n");
}

bool MotorControl::can_send_enable(unsigned char address)
{
    struct can_frame frame;
    frame.can_id  = address;
    frame.can_dlc = 8;
    memcpy(frame.data, PRIM_Enable(address), 8);
    canDevice.write((char *)&frame, sizeof(struct can_frame));
    return canDevice.read((char *)&frame, sizeof(struct can_frame));
}
bool MotorControl::can_send_disable(unsigned char address)
{
    struct can_frame frame;
    frame.can_id  = address;
    frame.can_dlc = 8;
    memcpy(frame.data, PRIM_Disable(address), 8);
    canDevice.write((char *)&frame, sizeof(struct can_frame));
    return canDevice.read((char *)&frame, sizeof(struct can_frame));
}

bool MotorControl::can_send_clear_error(unsigned char address)
{
    int error_code;
    struct can_frame frame;
    frame.can_id  = address;
    frame.can_dlc = 8;

    memcpy(frame.data, PRIM_GetError(address), 8);
    canDevice.write((char *)&frame, sizeof(struct can_frame));
    canDevice.read((char *)&frame, sizeof(struct can_frame));
    PRIM_ExplainError(frame.can_id, frame.data, &error_code);

    if(error_code == 0)
        return true;
    
    can_send_disable(address);

    memcpy(frame.data, PRIM_ClearError(address), 8);
    canDevice.write((char *)&frame, sizeof(struct can_frame));
    canDevice.read((char *)&frame, sizeof(struct can_frame));

    can_send_enable(address);
}

bool MotorControl::can_send_velocity(unsigned char address, int velocity)
{
    struct can_frame frame;
    frame.can_id  = address;
    frame.can_dlc = 8;
    memcpy(frame.data, PRIM_SetVelocity(address, velocity), 8);
    canDevice.write((char *)&frame, sizeof(struct can_frame));
    return canDevice.read((char *)&frame, sizeof(struct can_frame));
}
double MotorControl::can_read_velocity(unsigned char address)
{
    double velocity;
    struct can_frame frame;
    frame.can_id  = address;
    frame.can_dlc = 8;
    memcpy(frame.data, PRIM_GetActVelocity(address), 8);
    canDevice.write((char *)&frame, sizeof(struct can_frame));
    canDevice.read((char *)&frame, sizeof(struct can_frame));
    PRIM_ExplainActVelocity(frame.can_id, frame.data, &velocity);
    return velocity;
}
bool MotorControl::can_send_getvelocity(unsigned char address)
{
    struct can_frame frame;
    frame.can_id  = address;
    frame.can_dlc = 8;
    memcpy(frame.data, PRIM_GetActVelocity(address), 8);
    canDevice.write((char *)&frame, sizeof(struct can_frame));
    if(!canDevice.read((char *)&frame, sizeof(struct can_frame)))
      return false;
    else
    {
      handle_feedback((char *)&frame);
      return true;
    }
    
}
bool MotorControl::can_send_getposition(unsigned char address)
{
    struct can_frame frame;
    frame.can_id  = address;
    frame.can_dlc = 8;
    memcpy(frame.data, PRIM_GetActualPos(address), 8);
    canDevice.write((char *)&frame, sizeof(struct can_frame));
    if(!canDevice.read((char *)&frame, sizeof(struct can_frame)))
      return false;
    else
    {
      handle_feedback((char *)&frame);
      return true;
    }
}

MotorControl::MotorControl(): canDevice("can0")
{
    double wheel_length = 0.705;
    control_rate_ = 10;
    ros::NodeHandle nh_("~");
    nh_.param<double>("model_param", model_param_, 0.5);
    nh_.param<double>("wheel", wheel_length, 0.68);
    nh_.param<bool>("output_tf", output_tf, true);
    nh_.param<bool>("is_publish_odom", is_publish_odom, true);
    round_per_meter = 1 / wheel_length * 30;//电机圈数（30为减速电机）每米    42.55
    RPM_MAX = 3000;
}
void MotorControl::send_speed_callback()
{
    double left_d, right_d;//左右轮的direct速度 m/s
    double linear_speed = current_twist_.linear.x;//小车的直线速度 m/s
    double angular_speed = current_twist_.angular.z;//小车的角速度 rad/s

    left_d = (linear_speed - model_param_ * angular_speed);//左轮直线速度 m/s
    right_d = (linear_speed + model_param_ * angular_speed);//右轮直线速度 m/s
    
    int left, right;//左右轮的rpm
    left = static_cast<int>(left_d * round_per_meter * 60 * -1);//左轮设定RPM,右轮反转乘以-1
    right = static_cast<int>(right_d * round_per_meter * 60);//右轮设定RPM

    /*如果某个轮的速度超出最大值RPM_MAX，left与right同比例缩小(ratio),保持转弯半径不变*/
    int current_max_rpm = std::max(std::abs(left), std::abs(right));
    if(current_max_rpm >= RPM_MAX)
    {
      double ratio = (double)std::min(std::max(std::abs(left), std::abs(right)), RPM_MAX) / (double)current_max_rpm;
      left = left * ratio;
      right = right * ratio;
    }

    if(!is_motor_on)
      return;
    is_motor_on = can_send_velocity(1, left) && can_send_velocity(3, left) && can_send_velocity(2, right) && can_send_velocity(4, right);

    //ROS_INFO_STREAM("send -> left: " << left << "; right: " << right);
}

bool MotorControl::handle_feedback(const char *data)
{
    struct can_frame *frame = (struct can_frame *)data;

    if(frame->can_id > 4)
      return false;

    switch (frame->data[1])
    {
      case 0x05:
        if(frame->can_id == 1)
        {
          PRIM_ExplainActualPos(frame->can_id, frame->data, &position_left_latest);
        }
        else if(frame->can_id == 2)
        {
          PRIM_ExplainActualPos(frame->can_id, frame->data, &position_right_latest);
        }
        break;
      case 0x3F:
        if(frame->can_id == 1)
        {
          PRIM_ExplainActVelocity(frame->can_id, frame->data, &velocity_left_latest);
        }
        else if(frame->can_id == 2)
        {
          PRIM_ExplainActVelocity(frame->can_id, frame->data, &velocity_right_latest);
        }
        break;
      default:
        break;
    }
    return 0;//没有超时
}

void MotorControl::twist_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
    if(!canMove_base)
    {
      current_twist_.linear.x = 0;
      current_twist_.angular.z = 0;
      return;
    }

    current_twist_ = *msg.get();
    //限制move_base的原地旋转速度
    if((std::abs(current_twist_.linear.x) < 0.05) && (std::abs(current_twist_.angular.z) > 0.2))
    {
        current_twist_.angular.z = current_twist_.angular.z > 0 ? 0.2 : -0.2;
    }
    send_speed_callback();
    //ROS_INFO_STREAM("receive /cmd_vel msg.  v: " << current_twist_.linear.x << "w: " << current_twist_.angular.z);
}
void MotorControl::twist_joy_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
    current_twist_ = *msg.get();
    send_speed_callback();
    //ROS_INFO_STREAM("receive /cmd_vel_joy msg.  v: " << current_twist_.linear.x << "w: " << current_twist_.angular.z);
}

void MotorControl::check_motor_callback(const ros::TimerEvent&)
{
    if(!is_motor_on)
    {
      if(!can_send_enable(1))
      {
          ROS_INFO_STREAM("is_motor_on1: " << is_motor_on);
          return;
      }
      if(!can_send_enable(2))
      {
          ROS_INFO_STREAM("is_motor_on2: " << is_motor_on);
          return;
      }
      if(!can_send_enable(3))
      {
          ROS_INFO_STREAM("is_motor_on3: " << is_motor_on);
          return;
      }
      if(!can_send_enable(4))
      {
          ROS_INFO_STREAM("is_motor_on4: " << is_motor_on);
          return;
      }
    }
    is_motor_on = true;
    //ROS_INFO_STREAM("is_motor_on: " << is_motor_on);
}
void MotorControl::get_odometry_callback(const ros::TimerEvent&)
{
    if(!is_motor_on)
      return;
    is_motor_on = can_send_getvelocity(1) && can_send_getposition(1) && can_send_getvelocity(2) && can_send_getposition(2);

    if(is_motor_on)
    {
        publish_odom();
    }
    //double a1 = can_read_velocity(1);
    //double 
    //ROS_INFO_STREAM("1: " <<  << " 2: " << can_read_velocity(2) << " 3: " << can_read_velocity(3) << " 4: " << can_read_velocity(4));
}

void MotorControl::publish_odom()
{
  if(!motor_in_control)//第一次获取有效数据时等待下一次有效数据。
  {
    accumulation_th_ = 0;
    accumulation_x_ = 0;
    accumulation_y_ = 0;
    position_left_handle = position_left_latest;
    position_right_handle = position_right_latest;
    motor_in_control = true;
    return;
  }

  double delta_left_position = (-1) * (position_left_latest - position_left_handle);
  double delta_right_position = (position_right_latest - position_right_handle);
  //ROS_INFO_STREAM("receive position -> left: " << position_left_latest << "; right: " << position_right_latest);
  //ROS_INFO_STREAM("receive velocity -> left: " << velocity_left_latest << "; right: " << velocity_right_latest);

  position_left_handle = position_left_latest;
  position_right_handle = position_right_latest;

  double left_velocity = velocity_left_latest / (round_per_meter * 60 * -1);//左轮直线速度m/s
  double right_velocity = velocity_right_latest / (round_per_meter * 60 );//左轮直线速度m/s

  delta_left_position /= (pulse_per_round * round_per_meter);//左轮前进距离 m
  delta_right_position /= (pulse_per_round * round_per_meter);//右轮前进距离 m

  //ROS_INFO_STREAM("delta position -> left: " << delta_left_position << "; right: " << delta_right_position);

  double delta_theta = (delta_right_position - delta_left_position)/ ( model_param_ * 2);

  double delta_dis = (delta_right_position + delta_left_position) / 2.0;

  double v_dis = (right_velocity + left_velocity) / 2;
  double v_theta = (right_velocity - left_velocity)/ model_param_;

  double delta_x = cos(delta_theta) * delta_dis;
  double delta_y = -sin(delta_theta) * delta_dis;

  accumulation_x_ += (cos(accumulation_th_) * delta_x - sin(accumulation_th_) * delta_y);
  accumulation_y_ += (sin(accumulation_th_) * delta_x + cos(accumulation_th_) * delta_y);
  accumulation_th_ += delta_theta;

  auto now_ = ros::Time::now();
  transformStamped_.header.stamp = now_;
  transformStamped_.header.frame_id = odom_frame_;
  transformStamped_.child_frame_id = base_frame_;
  transformStamped_.transform.translation.x = accumulation_x_;
  transformStamped_.transform.translation.y = accumulation_y_;
  transformStamped_.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, accumulation_th_);
  transformStamped_.transform.rotation.x = q.x();
  transformStamped_.transform.rotation.y = q.y();
  transformStamped_.transform.rotation.z = q.z();
  transformStamped_.transform.rotation.w = q.w();

  if(output_tf)
    br_.sendTransform(transformStamped_);
  
  odom_.header.frame_id = odom_frame_;
  odom_.child_frame_id = base_frame_;
  odom_.header.stamp = now_;
  odom_.pose.pose.position.x = accumulation_x_;
  odom_.pose.pose.position.y = accumulation_y_;
  odom_.pose.pose.position.z = 0;
  odom_.pose.pose.orientation.x = q.getX();
  odom_.pose.pose.orientation.y = q.getY();
  odom_.pose.pose.orientation.z = q.getZ();
  odom_.pose.pose.orientation.w = q.getW();
  odom_.twist.twist.linear.x = v_dis;
  odom_.twist.twist.linear.y = 0;
  odom_.twist.twist.angular.z = v_theta;
  if(is_publish_odom)
    odom_pub_.publish(odom_);

  //ROS_INFO_STREAM("accumulation_x: " << accumulation_x_ << "; accumulation_y: " << accumulation_y_ <<"; accumulation_th: " << accumulation_th_);
}

bool MotorControl::commondCallback(motor_control::motor_commond::Request & request, motor_control::motor_commond::Response & response)
{
    ROS_INFO("a commond to motor_control %d", request.commond);
    switch (request.commond)
    {
        case 0 ://禁用move_base
            canMove_base = 0;
            current_twist_.linear.x = 0;
            current_twist_.angular.z = 0;
            break;
        case 1 ://允许move_base
            canMove_base = 1;
            break;
        case 2 : //清除故障
            can_send_clear_error(1);
            can_send_clear_error(2);
            can_send_clear_error(3);
            can_send_clear_error(4);
            break;
        case 3 : //odom清零
            accumulation_th_ = 0;
            accumulation_x_ = 0;
            accumulation_y_ = 0;
            break;
        default :
            break;
    }
    return true;
}
void MotorControl::exec()
{
    odom_pub_ = node.advertise<nav_msgs::Odometry>("/odom", 10);
    ros::Subscriber cmd_sub = node.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &MotorControl::twist_callback, this);//处理move_base
    ros::Subscriber cmd_joy_sub = node.subscribe<geometry_msgs::Twist>("/cmd_vel_joy", 10, &MotorControl::twist_joy_callback, this);//处理joy
    ros::Timer get_odometry_timer = node.createTimer(ros::Duration(1.0/control_rate_), &MotorControl::get_odometry_callback, this);
    ros::Timer check_motor_timer = node.createTimer(ros::Duration(1.0), &MotorControl::check_motor_callback, this);
    ros::ServiceServer service = node.advertiseService("motor_control/commond", &MotorControl::commondCallback, this);
    ros::spin();
}
MotorControl::~MotorControl()
{
}

int main(int argc, char **argv){
  ros::init(argc, argv, "MotorControl");
  MotorControl motorControl;
  motorControl.exec();
  return 0;
}