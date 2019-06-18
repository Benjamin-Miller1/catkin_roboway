#include "motor_control/motor_driver.h"
#include "motor_control/motor_control.h"
#include <tf2_ros/transform_broadcaster.h>
#include <dynamic_reconfigure/server.h>

#include <thread>
#include <numeric>
#include <fstream>
#include <boost/bind.hpp>
#include <iostream>
#include <string.h>

bool print_data(const unsigned char *data)
{
    for (int i = 0; i < 8; ++i)
    {
        printf(" %02X", data[i]);
    }
    printf("\r\n");
}

void MotorControl::serialWrite(unsigned char *data, size_t size)
{
    ros_ser.write(data, size);
}
bool MotorControl::serialRead(unsigned char *data, size_t size)
{
    size_t size_result = ros_ser.read(data, size);
    if(size_result == size)
        return true;
    else
    {
        ROS_INFO_STREAM("serial read timeout.");
        return false;
    }
}

bool MotorControl::can_send_enable(unsigned char address)
{
    unsigned char data[8];
    memcpy(data, PRIM_Enable(address), 8);
    serialWrite(data, sizeof(data));
    return serialRead(data, sizeof(data));
}
bool MotorControl::can_send_disable(unsigned char address)
{
    unsigned char data[8];
    memcpy(data, PRIM_Disable(address), 8);
    serialWrite(data, sizeof(data));
    return serialRead(data, sizeof(data));
}

bool MotorControl::can_send_clear_error(unsigned char address)
{
    int error_code;
    unsigned char data[8];

    memcpy(data, PRIM_GetError(address), 8);
    serialWrite(data, sizeof(data));
    serialRead(data, sizeof(data));
    PRIM_ExplainError(address, data, &error_code);

    if(error_code == 0)
        return true;
    
    can_send_disable(address);

    memcpy(data, PRIM_ClearError(address), 8);
    serialWrite(data, sizeof(data));
    serialRead(data, sizeof(data));

    can_send_enable(address);
}

bool MotorControl::can_send_velocity(unsigned char address, int velocity)
{
    unsigned char data[8];
    memcpy(data, PRIM_SetVelocity(address, velocity), 8);
    serialWrite(data, sizeof(data));
    return serialRead(data, sizeof(data));
}

bool MotorControl::can_send_getvelocity(unsigned char address)
{
    unsigned char data[8];
    memcpy(data, PRIM_GetActVelocity(address), 8);
    serialWrite(data, sizeof(data));
    if(!serialRead(data, sizeof(data)))
      return false;
    else
    {
      handle_feedback(data);
      return true;
    }
    
}
bool MotorControl::can_send_getposition(unsigned char address)
{
    unsigned char data[8];
    memcpy(data, PRIM_GetActualPos(address), 8);
    serialWrite(data, sizeof(data));
    if(!serialRead(data, sizeof(data)))
      return false;
    else
    {
      handle_feedback(data);
      return true;
    }
}

MotorControl::MotorControl()
{
    try {
        ros_ser.setPort("/dev/motor_serial");
        ros_ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(30);
        ros_ser.setTimeout(to);
        ros_ser.open();
    } catch (serial::IOException &e) {
        ROS_ERROR_STREAM("Unable to open port.");
        exit(-1);
    }
}
void MotorControl::send_speed_callback()
{
    double left_d, right_d;//左右轮的direct速度 m/s
    double linear_speed = current_twist_.linear.x;//小车的直线速度 m/s
    double angular_speed = current_twist_.angular.z;//小车的角速度 rad/s

    left_d = (linear_speed - model_param * angular_speed);//左轮直线速度 m/s
    right_d = (linear_speed + model_param * angular_speed);//右轮直线速度 m/s
    
    int left, right;//左右轮的rpm
    left = static_cast<int>(left_d * round_per_meter * 60);//左轮设定RPM,右轮反转乘以-1
    right = static_cast<int>(right_d * round_per_meter * 60 * -1);//右轮设定RPM

    /*如果某个轮的速度超出最大值RPM_MAX，left与right同比例缩小(ratio),保持转弯半径不变*/
    int current_max_rpm = std::max(std::abs(left), std::abs(right));
    if(current_max_rpm >= RPM_MAX)
    {
      double ratio = (double)std::min(std::max(std::abs(left), std::abs(right)), RPM_MAX) / (double)current_max_rpm;
      left = left * ratio;
      right = right * ratio;
    }

    can_send_velocity(5, left);
    if(wheelcount == 4)
        can_send_velocity(3, left);
    can_send_velocity(2, right);
    if(wheelcount == 4)
        can_send_velocity(4, right);

    //ROS_INFO_STREAM("send -> left: " << left << "; right: " << right);
}

bool MotorControl::handle_feedback(unsigned char *data)
{

    int address = data[0];
    if(address > 5)
      return false;

    switch (data[1])
    {
      case 0x05:
        if(address == 5)
        {
          PRIM_ExplainActualPos(address, data, &position_left_latest);
        }
        else if(address == 2)
        {
          PRIM_ExplainActualPos(address, data, &position_right_latest);
        }
        break;
      case 0x3F:
        if(address == 5)
        {
          PRIM_ExplainActVelocity(address, data, &velocity_left_latest);
        }
        else if(address == 2)
        {
          PRIM_ExplainActVelocity(address, data, &velocity_right_latest);
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
void MotorControl::send_speed_callback(const ros::TimerEvent&)
{
    send_speed_callback();
}
void MotorControl::get_odometry_callback(const ros::TimerEvent&)
{
    can_send_getvelocity(5);
    can_send_getposition(5);
    can_send_getvelocity(2);
    can_send_getposition(2);

    publish_odom();
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

  double delta_left_position = (position_left_latest - position_left_handle);
  double delta_right_position = (-1) * (position_right_latest - position_right_handle);
  //ROS_INFO_STREAM("receive position -> left: " << position_left_latest << "; right: " << position_right_latest);
  //ROS_INFO_STREAM("receive velocity -> left: " << velocity_left_latest << "; right: " << velocity_right_latest);

  position_left_handle = position_left_latest;
  position_right_handle = position_right_latest;

  double left_velocity = velocity_left_latest / (round_per_meter * 60);//左轮直线速度m/s
  double right_velocity = velocity_right_latest / (round_per_meter * 60 * -1);//左轮直线速度m/s

  delta_left_position /= (pulse_per_round * round_per_meter);//左轮前进距离 m
  delta_right_position /= (pulse_per_round * round_per_meter);//右轮前进距离 m

  //ROS_INFO_STREAM("delta position -> left: " << delta_left_position << "; right: " << delta_right_position);

  double delta_theta = (delta_right_position - delta_left_position)/ ( model_param * 2);

  double delta_dis = (delta_right_position + delta_left_position) / 2.0;

  double v_dis = (right_velocity + left_velocity) / 2;
  double v_theta = (right_velocity - left_velocity)/ model_param;

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
            can_send_clear_error(5);
            can_send_clear_error(2);
            if(wheelcount == 4) {
                can_send_clear_error(3);
                can_send_clear_error(4);
            }
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

void MotorControl::dynamic_callback(motor_control::paramConfig &config, uint32_t level)
{
	ROS_INFO("Reconfigure Request: model_param: %f  wheel_length: %f ",
                     config.model_param, config.wheel);
    model_param = config.model_param;
    wheel_length = config.wheel;
    round_per_meter = 1 / wheel_length * ratio;
}

void MotorControl::exec()
{
    ros::NodeHandle nh_("~");

    dynamic_reconfigure::Server<motor_control::paramConfig> server;
	dynamic_reconfigure::Server<motor_control::paramConfig>::CallbackType f
             = boost::bind(&MotorControl::dynamic_callback, this, _1, _2);
	server.setCallback(f);

    nh_.param<double>("model_param", model_param, 0.5);//越小转得越快
    nh_.param<double>("wheel", wheel_length, 0.68);//越小走得越远
    nh_.param<bool>("output_tf", output_tf, true);
    nh_.param<bool>("is_publish_odom", is_publish_odom, true);
    nh_.param<int>("ratio", ratio, 30);
    nh_.param<int>("wheelcount", wheelcount, 4);
    
    round_per_meter = 1 / wheel_length * ratio;//电机转动圈数每米

    RPM_MAX = 3000;

    can_send_enable(5);
    can_send_enable(2);
    if(wheelcount == 4) {
        can_send_enable(3);
        can_send_enable(4);
    }


    odom_pub_ = node.advertise<nav_msgs::Odometry>("/odom", 10);
    ros::Subscriber cmd_sub = node.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &MotorControl::twist_callback, this);//处理move_base
    ros::Subscriber cmd_joy_sub = node.subscribe<geometry_msgs::Twist>("/cmd_vel_joy", 10, &MotorControl::twist_joy_callback, this);//处理joy
    ros::Timer get_odometry_timer = node.createTimer(ros::Duration(1.0/10), &MotorControl::get_odometry_callback, this);
    ros::Timer send_speed_timer = node.createTimer(ros::Duration(1.0/10), &MotorControl::send_speed_callback, this);
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
