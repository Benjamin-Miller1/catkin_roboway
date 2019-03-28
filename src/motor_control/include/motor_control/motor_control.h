#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <chrono>
#include <std_msgs/UInt8.h>
#include <mutex>
#include <boost/asio.hpp>
#include <motor_control/motor_commond.h>
#include <memory>
using namespace ros;
using namespace std;
using namespace tf2_ros;
using namespace boost;

class CanDevice
{
public:
    using protocol = boost::asio::generic::raw_protocol;

    CanDevice(const char* devname);
    CanDevice(const CanDevice&) = delete;
    CanDevice& operator=(const CanDevice&) = delete;
    ~CanDevice();
    
    void write(const char *data, size_t size);
    bool read(char *data, size_t size);
private:
    enum ReadResult
    {
        resultInProgress,
        resultSuccess,
        resultError,
        resultTimeoutExpired
    };

    int local_socket_;
    std::string interface_name_;
    asio::io_service io_service;
    std::shared_ptr<asio::generic::raw_protocol::socket> raw_socket;
    asio::deadline_timer timer; ///< Timer for timeout
    enum ReadResult result;  ///< Used by read with timeout
    size_t bytesTransferred;
    protocol::endpoint endpoint() const;
    void readCompleted(const boost::system::error_code& error, const size_t bytesTransferred);
    void timeoutExpired(const boost::system::error_code& error);
};


class MotorControl
{
public:
	MotorControl();
	void exec();
	void twist_callback(const geometry_msgs::Twist::ConstPtr& msg);
  void twist_joy_callback(const geometry_msgs::Twist::ConstPtr& msg);
	virtual ~MotorControl();

protected:
private:
  CanDevice canDevice;
  bool motor_in_control{false};
	ros::NodeHandle node;
	ros::Publisher odom_pub_;
  bool is_motor_on{false};
  double velocity_left_latest, velocity_right_latest; //最近一次所更新的数据
  int position_left_latest, position_right_latest;

  int position_left_handle, position_right_handle;//倒数第二次记录的数据实际处理的数据

	int control_rate_;
  int RPM_MAX;
	double model_param_;//理论上是左右两轮的间距
  double round_per_meter;
  const double pulse_per_round = 10000;//pulse per round
  std::string odom_frame_{"odom"}, base_frame_{"base_link"};
  double accumulation_x_{0}, accumulation_y_{0}, accumulation_th_{0};
  geometry_msgs::TransformStamped transformStamped_;
  tf2_ros::TransformBroadcaster br_;

  nav_msgs::Odometry odom_;
  mutex g_lock;
  
	geometry_msgs::Twist current_twist_;
	void send_speed_callback();
  void get_odometry_callback(const ros::TimerEvent&);
  void check_motor_callback(const ros::TimerEvent&);
  bool commondCallback(motor_control::motor_commond::Request & request, motor_control::motor_commond::Response & response);

  void publish_odom();
  bool canMove_base{true};
  bool handle_feedback(const char *data);
  bool can_send_velocity(unsigned char address, int velocity);
  bool can_send_getvelocity(unsigned char address);
  bool can_send_getposition(unsigned char address);
  bool can_send_enable(unsigned char address);
  bool can_send_clear_error(unsigned char address);
};