#ifndef _RONIN_S_DRIVER_H_
#define _RONIN_S_DRIVER_H_

#include <rclcpp/rclcpp.hpp>

#include <stdio.h>
#include <cstring>
#include <string>
#include <vector>
#include <chrono>
#include <functional>
#include <memory>
#include <std_msgs/msg/u_int8_multi_array.h>
#include <ros_kvaser_can_driver/msg/can_frame.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <ros_ronin_s_driver/msg/dji_ronin_s_status.hpp>
#include <ronin_s_crc.h>

using namespace std::chrono_literals;

#define BUFFER_SIZE 255
#define CAN_ID_NUM 10
#define RONIN_S_MAX_VEL 15000
#define RONIN_S_MIN_VEL -15000
#define RONIN_S_COMMAND_TIMEOUT 10

typedef enum can_ids
{
  ID_0x012 = 0,
  ID_0x021 = 1,
  ID_0x416 = 2,
  ID_0x530 = 3
} can_ids;

class RoninSDriver : public rclcpp::Node {
public:
  RoninSDriver();
  ~RoninSDriver();

private:
  // ROS
  // Publisher
  rclcpp::Publisher<ros_ronin_s_driver::msg::DJIRoninSStatus>::SharedPtr ronin_s_driver_info_pub_;
  rclcpp::Publisher<ros_kvaser_can_driver::msg::CANFrame>::SharedPtr ronin_s_driver_tx_can_pub_;

  // Subscriber
  rclcpp::Subscription<ros_kvaser_can_driver::msg::CANFrame>::SharedPtr ronin_s_driver_rx_can_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr ronin_s_driver_twist_sub_;

  // Timer
  rclcpp::TimerBase::SharedPtr command_timer_;

  bool debug_print_;
  uint8_t can_buffer_[CAN_ID_NUM][BUFFER_SIZE];
  int can_buffer_rp_[CAN_ID_NUM];
  int can_buffer_wp_[CAN_ID_NUM];

  // Ronin-S Data
  uint16_t ronin_s_remote_cmd_counter_;
  geometry_msgs::msg::Twist in_twist_;
  geometry_msgs::msg::Twist out_twist_;
  int twist_update_flag_;
  int twist_not_update_count_;

  int16_t limitRoninSVel(int16_t in);
  int parseCanData(uint16_t id, uint8_t *in_data, uint8_t in_data_size, uint8_t out_data[], uint8_t *out_data_size);

  // callback functions
  void canmsgCallback(const ros_kvaser_can_driver::msg::CANFrame::SharedPtr msg);
  void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void publisherTimerCallback(void);

};

#endif  // _RONIN_S_DRIVER_H_