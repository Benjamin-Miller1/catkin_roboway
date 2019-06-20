#ifndef KEYBOARD_CONTROL_NODE_H
#define KEYBOARD_CONTROL_NODE_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <fcntl.h>
#include <dirent.h>
#include <linux/input.h>
#include <sys/stat.h>

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <agent/commond.h>
#include <motor_control/motor_commond.h>
#include <string>

namespace autolabor_tool {
class KeyboardControl
{
public:
  KeyboardControl();
  ~KeyboardControl();

  void run();
private:
  ros::NodeHandle nh_;
  ros::Timer twist_pub_timer_;
  ros::Publisher twist_pub_;
  ros::ServiceClient agent_client;
  ros::ServiceClient motor_control_client;

  int fd_;
  struct input_event ev_;

  int linear_state_, angular_state_;
  bool numberKey;
  std::string port_name_;
  double rate_;
  double linear_scale_, angular_scale_;
  double linear_min_, linear_max_, linear_step_;
  double angular_min_, angular_max_, angular_step_;
  bool send_flag_;

  bool init();
  void parseKeyboard();
  void buttonTwistCheck(int value, int& state, int down, int up);
  void buttonScaleCheck(int value, double& scale, double step, double limit);
  void publishCmdVel();
};

}

#endif // KEYBOARD_CONTROL_NODE_H

namespace autolabor_tool {

KeyboardControl::KeyboardControl():linear_state_(0), angular_state_(0), port_name_(""){
  ros::NodeHandle private_node("~");

  private_node.param("linear_min", linear_min_, 0.2);
  private_node.param("linear_max", linear_max_, 1.2);
  private_node.param("linear_step", linear_step_, 0.2);

  private_node.param("angular_min", angular_min_, 0.15);
  private_node.param("angular_max", angular_max_, 1.35);
  private_node.param("angular_step", angular_step_, 0.25);

  private_node.param("rate", rate_, 10.0);

  linear_scale_ = linear_min_;
  angular_scale_ = angular_min_;
  send_flag_ = false;
}

KeyboardControl::~KeyboardControl(){
  close(fd_);
}

void KeyboardControl::buttonTwistCheck(int value, int& state, int down, int up){
  if (value == 1){
    state += down;
  }else if (value == 0){
    state += up;
  }
}

void KeyboardControl::buttonScaleCheck(int value, double &scale, double step, double limit){
  if (value == 1){
    if (step > 0){
      scale = std::min(scale + step, limit);
    }else{
      scale = std::max(scale + step, limit);
    }
  }
}

void KeyboardControl::parseKeyboard(){
  static bool upPressEvent = false;
  static bool downPressEvent = false;
  static bool leftPressEvent = false;
  static bool rightPressEvent = false;

  unsigned int KEYBOARD_UP;
  unsigned int KEYBOARD_DOWN;
  unsigned int KEYBOARD_LEFT;
  unsigned int KEYBOARD_RIGHT;
  unsigned int LINEAR_INC;
  unsigned int LINEAR_DEC;
  unsigned int ANGULAR_INC;
  unsigned int ANGULAR_DEC;
  unsigned int STOP;
  unsigned int RUN;
  unsigned int REACH;
  unsigned int START;
  unsigned int BACK;
  unsigned int CAN_MOVEBASE;
  unsigned int CLEAR_ERROR;
  if(numberKey)
  {
    KEYBOARD_UP = KEY_KP5;
    KEYBOARD_DOWN = KEY_KP2;
    KEYBOARD_LEFT = KEY_KP1;
    KEYBOARD_RIGHT = KEY_KP3;
    LINEAR_INC = KEY_KP7;
    LINEAR_DEC = KEY_KP4;
    ANGULAR_INC = KEY_KP9;
    ANGULAR_DEC = KEY_KP6;
    STOP = KEY_KP0;
    RUN = KEY_KP8;
    REACH = KEY_KPSLASH;
    START = KEY_KPASTERISK;
    BACK = KEY_KPMINUS;
    CAN_MOVEBASE = KEY_KPPLUS;
    CLEAR_ERROR = KEY_KPENTER;
  }
  else
  {
    KEYBOARD_UP = KEY_U;
    KEYBOARD_DOWN = KEY_J;
    KEYBOARD_LEFT = KEY_H;
    KEYBOARD_RIGHT = KEY_K;
    LINEAR_INC = KEY_6;
    LINEAR_DEC = KEY_Y;
    ANGULAR_INC = KEY_8;
    ANGULAR_DEC = KEY_I;
    STOP = KEY_F9;
    RUN = KEY_F8;
    REACH = KEY_9;
    START = KEY_0;
    BACK = KEY_MINUS;
    CAN_MOVEBASE = KEY_BACKSPACE;
    CLEAR_ERROR = KEY_ENTER;
  }

  while (true) {
    read(fd_, &ev_, sizeof(struct input_event));
    if (ev_.type == EV_KEY){
      ROS_INFO_STREAM("INFO: [key]: " << ev_.code << ", [value]: " << ev_.value);

      if (ev_.code == KEYBOARD_UP) {
        if(ev_.value == 1)
          upPressEvent = true;
        else if(ev_.value == 0)
        {
          if(upPressEvent)
            upPressEvent = false;
          else
            continue;
        }
        buttonTwistCheck(ev_.value, linear_state_, 1, -1);
      }
      else if(ev_.code == KEYBOARD_DOWN) {
        if(ev_.value == 1)
          downPressEvent = true;
        else if(ev_.value == 0)
        {
          if(downPressEvent)
            downPressEvent = false;
          else
            continue;
        }
        buttonTwistCheck(ev_.value, linear_state_, -1, 1);
      }
      else if(ev_.code == KEYBOARD_LEFT) {
        if(ev_.value == 1)
          leftPressEvent = true;
        else if(ev_.value == 0)
        {
          if(leftPressEvent)
            leftPressEvent = false;
          else
            continue;
        }
        buttonTwistCheck(ev_.value, angular_state_, 1, -1);
      }
      else if(ev_.code == KEYBOARD_RIGHT) {
        if(ev_.value == 1)
          rightPressEvent = true;
        else if(ev_.value == 0)
        {
          if(rightPressEvent)
            rightPressEvent = false;
          else
            continue;
        }
        buttonTwistCheck(ev_.value, angular_state_, -1, 1);
      }
      else if(ev_.code == LINEAR_INC)
        buttonScaleCheck(ev_.value, linear_scale_, linear_step_, linear_max_);
      else if(ev_.code == LINEAR_DEC)
        buttonScaleCheck(ev_.value, linear_scale_, -linear_step_, linear_min_);
      else if(ev_.code == ANGULAR_INC)
        buttonScaleCheck(ev_.value, angular_scale_, angular_step_, angular_max_);
      else if(ev_.code == ANGULAR_DEC)
        buttonScaleCheck(ev_.value, angular_scale_, -angular_step_, angular_min_);
      else if(ev_.code == STOP) {
        if (ev_.value == 1){
          send_flag_ = false;
        }
      }
      else if(ev_.code == RUN) {
        if (ev_.value == 1){
          send_flag_ = true;
        }
      }
      else if(ev_.code == REACH) {
        if (ev_.value == 1 && send_flag_ == true){
            agent::commond srv;
            srv.request.type = 1;
            srv.request.value = 0;
            agent_client.call(srv);
        }
      }
      else if(ev_.code == START) {
        if (ev_.value == 1 && send_flag_ == true){
            agent::commond srv;
            srv.request.type = 1;
            srv.request.value = 1;
            agent_client.call(srv);
        }
      }
      else if(ev_.code == BACK) {
        if (ev_.value == 1 && send_flag_ == true){
            agent::commond srv;
            srv.request.type = 1;
            srv.request.value = 2;
            agent_client.call(srv);
        }
      }
      else if(ev_.code == CAN_MOVEBASE) {
        if (ev_.value == 1 && send_flag_ == true){
            static bool can_move_base = true;
            can_move_base = !can_move_base;
            motor_control::motor_commond srv;
            srv.request.commond = can_move_base;
            motor_control_client.call(srv);
        }
      }
      else if(ev_.code == CLEAR_ERROR) {
        if (ev_.value == 1 && send_flag_ == true){
          motor_control::motor_commond srv;
          srv.request.commond = 2;
          motor_control_client.call(srv);
        }
      }
      else
        continue;
      publishCmdVel();
    }
  }
}
void KeyboardControl::publishCmdVel()
{
    static geometry_msgs::Twist twist_prev;
    geometry_msgs::Twist twist;
    twist.linear.x = 0;
    twist.angular.z = 0;
    if (send_flag_){
      
      twist.linear.x = linear_state_ * linear_scale_;
      twist.angular.z = angular_state_ * angular_scale_;
      if(twist.linear.x < 0)
        twist.angular.z = -1 * twist.angular.z;
      if(twist_prev.linear.x == twist.linear.x && twist_prev.angular.z == twist.angular.z)
        return;
      twist_pub_.publish(twist);
      twist_prev = twist;
      ROS_INFO_STREAM("linear: " << twist.linear.x << " angular: " << twist.angular.z);
    }
    else{
      twist_pub_.publish(twist);
      twist_prev = twist;
      ROS_INFO_STREAM("linear: " << 0 << " angular: " << 0);
    }
}


bool KeyboardControl::init(){
  const char path[] = "/dev/input/by-path";
  DIR *dev_dir = opendir(path);
  struct dirent *entry;
  if (dev_dir == NULL){
    return false;
  }

  std::string device_name;
  ros::param::get("~devicename", device_name);
  ros::param::get("~numberKey", numberKey);
  
  while ((entry = readdir(dev_dir)) != NULL){
    std::string dir_str = entry->d_name;
    if (dir_str.find(device_name) < dir_str.length()){
      port_name_ = std::string(path) + "/" + dir_str;
      ROS_INFO_STREAM("INFO: The keyboard port is :" << port_name_);
      break;
    }
  }
  closedir(dev_dir);

  if (port_name_ != ""){
    fd_ = open(port_name_.c_str(), O_RDONLY);
    if (fd_ < 0){
      ROS_ERROR_STREAM("ERROR: Can't Open The Port :" << port_name_);
      return false;
    }else{
      ROS_INFO_STREAM("INFO: Open The Port :" << port_name_);
      return true;
    }
  }else{
    return false;
  }
}

void KeyboardControl::run(){
  if (init()){

    twist_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_joy", 10);
    agent_client = nh_.serviceClient<agent::commond>("agent/commond");
    motor_control_client = nh_.serviceClient<motor_control::motor_commond>("motor_control/commond");
    boost::thread parse_thread(boost::bind(&KeyboardControl::parseKeyboard, this));
    ros::spin();
  }
}
}

int main(int argc, char *argv[]){
  ros::init(argc, argv, "keyboard_control_node");
  autolabor_tool::KeyboardControl keyboard_control;
  keyboard_control.run();
  return 0;
}
