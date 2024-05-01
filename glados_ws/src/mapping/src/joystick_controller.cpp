#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include </usr/include/libserial/SerialStream.h>
#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <functional>
#include <sstream>
#include <memory>

#include <math.h>

using std::placeholders::_1;
using namespace std::chrono_literals;

class SerialCommunicator {

private:

struct data_packet
{
    int state;
    int direction_r;
    int motor_r;
    int direction_l;
    int motor_l;
    int servo;

    std::string toString()
    {
    return std::to_string(state)
           + " " + std::to_string(direction_r) + " " + std::to_string(motor_r) 
           + " " + std::to_string(direction_l) + " " + std::to_string(motor_l)
           + " " + std::to_string(servo);
    }
}; 

LibSerial::SerialStream serial;

public:
        SerialCommunicator() {}
        data_packet dp = {};

        void init_comms(const std::string& port) {
            serial.Open(port);
            serial.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
            serial.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
            serial.SetParity(LibSerial::Parity::PARITY_NONE);
            serial.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
            serial.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
        }

        void init_motors() {
            
            dp.state = 0;
            dp.direction_r = 0;
            dp.motor_r = 0;
            dp.direction_l = 0;
            dp.motor_l = 0;
            dp.servo = 0;

            serial << dp.toString() << std::endl;
        }

        void deinit_motors(){
            dp.state = 1;
            dp.direction_r = 0;
            dp.motor_r = 0;
            dp.direction_l = 0;
            dp.motor_l = 0;
            dp.servo = 0;

            serial << dp.toString() << std::endl;
        }

        void stop_motors(){
            dp.state = 2;
            dp.direction_r = 0;
            dp.motor_r = 0;
            dp.direction_l = 0;
            dp.motor_l = 0;
            dp.servo = 0;

            serial << dp.toString() << std::endl;
        }

        void write_motors(int direction_r, int speed_r, int direction_l, int speed_l) {
            dp.state = 3;
            dp.direction_r = direction_r;
            dp.motor_r = speed_r;
            dp.direction_l = direction_l;
            dp.motor_l = speed_l;
            dp.servo = 0;
            serial << dp.toString() << std::endl;
        }

        void servo(int servo_pwm) {
            dp.state = 4;
            dp.direction_r = 0;
            dp.motor_r = 0;
            dp.direction_l = 0;
            dp.motor_l = 0;
            dp.servo = servo_pwm;
            serial << dp.toString() << std::endl;
        }

        void read(int BUFFER_SIZE, char* input_buffer) {
            serial.read( input_buffer, BUFFER_SIZE);
        }

        void close() {
            serial.Close();
        }
};

class JoystickDriver : public rclcpp::Node
{
  public:
    
    JoystickDriver()
    : Node("joystick_driver")
    {
      joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&JoystickDriver::command_actuators, this, _1));
      
      // Initialize the serial communicator
      communicator.init_comms("/dev/ttyACM0");  
      
      lidar_frame_pub_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);    
    }

  private:
    void pub_transforms(float lidar_angle){
      geometry_msgs::msg::TransformStamped t;

      t.header.stamp = this->get_clock()->now();
      t.header.frame_id = "chassis_link";
      t.child_frame_id = "rotation_point";

      t.transform.translation.x = 1.0;
      t.transform.translation.y = -0.109;
      t.transform.translation.z = 0.1;

      tf2::Quaternion q;
      q.setRPY(0, -lidar_angle, 0);
      t.transform.rotation.x = q.x();
      t.transform.rotation.y = q.y();
      t.transform.rotation.z = q.z();
      t.transform.rotation.w = q.w();

      lidar_frame_pub_->sendTransform(t);
    }

    void command_motors(int forward_backward, int left, int right) {
      if(forward_backward > 500) {
        communicator.write_motors(1, forward_backward, 1, forward_backward);
      }
      else if(forward_backward < -500) {
        communicator.write_motors(0, forward_backward, 0, forward_backward); 
      } else if (right > 500) {
        communicator.write_motors(1, right, 0, right);
      } else if (left > 500) {
        communicator.write_motors(0, left, 1, left);
      } else {
        communicator.write_motors(0, 0, 0, 0);
      }
    }

    void rotate_servo(){
      
      for (int i = -15; i <= 20; i++) {
        int pwm_value = (int)((218.45 * i + 32389 + 13630) / 9.4486) - 46;
        pub_transforms(i*3.14159/180);
        communicator.servo(pwm_value);
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        //RCLCPP_INFO(get_logger(), "Servo: %d", pwm_value);        
      }

      communicator.servo(4824);
      pub_transforms(0);
    }


    void command_actuators(const sensor_msgs::msg::Joy & msg) 
    {

      int add_odom = msg.buttons[0]; // A button, Green
      int stop = msg.buttons[1];          // B button, Red
      int reset_odom = msg.buttons[2];   // X button, Blue
      int rotate = msg.buttons[3];      // Y button, Yellow
      
      int servo_scan = msg.buttons[4]; // LB button, Left Bumper
      int enable_motors = msg.buttons[5]; // RB button, Right Bumper

      int forward_backward = (int)(msg.axes[1] * 65535);
      int left = (int)  ( ((-msg.axes[4] + 1) / 2) * 65535);
      int right = (int) ( ((-msg.axes[5] + 1) / 2) * 65535);
      
      if(stop) {
        communicator.stop_motors();
      } else if(enable_motors) {
        command_motors(forward_backward, left, right);
      } else if(servo_scan) {
        rotate_servo();
      } else if(rotate) {
        communicator.write_motors(1, 65535, 0, 65535);
      } else {
        communicator.write_motors(0, 0, 0, 0);
      }

      pub_transforms(0);

    }
    
    SerialCommunicator communicator;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> lidar_frame_pub_;
};

//      rclcpp::sleep_for(std::chrono::milliseconds(100));


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoystickDriver>());
  rclcpp::shutdown();
  return 0;
}