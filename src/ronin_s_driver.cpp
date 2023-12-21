#include "ronin_s_driver_node.h"

// RoninSDriver()
// Constructor
RoninSDriver::RoninSDriver() : Node("ronin_s_driver_node")
{
    // ROS
    // パラメータの取得
    this->declare_parameter("debug_print", false);
    this->get_parameter("debug_print", debug_print_);
    RCLCPP_INFO(this->get_logger(), "debug_print : %d", debug_print_);

    // Existing subscription logic
    ronin_s_driver_rx_can_sub_ = this->create_subscription<ros_kvaser_can_driver::msg::CANFrame>(
        "/can_rx", 10, std::bind(&RoninSDriver::canmsgCallback, this, std::placeholders::_1));
    ronin_s_driver_twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/ronin_s/cmd_twist", 10, std::bind(&RoninSDriver::twistCallback, this, std::placeholders::_1));

    // Publisher creation logic for custom message
    ronin_s_driver_info_pub_ = this->create_publisher<ros_ronin_s_driver::msg::DJIRoninSStatus>("/ronin_s/info", 10);
    ronin_s_driver_tx_can_pub_ = this->create_publisher<ros_kvaser_can_driver::msg::CANFrame>("/can_tx", 10);

    // Timer
    command_timer_ = this->create_wall_timer(20ms, std::bind(&RoninSDriver::publisherTimerCallback, this));

    for (int i = 0; i < CAN_ID_NUM; i++)
    {
        can_buffer_rp_[i] = 0;
        can_buffer_wp_[i] = 0;
    }
    ronin_s_remote_cmd_counter_ = 0;

    in_twist_.angular.x = 0;
    in_twist_.angular.y = 0;
    in_twist_.angular.z = 0;
    out_twist_.angular.x = 0;
    out_twist_.angular.y = 0;
    out_twist_.angular.z = 0;
    twist_update_flag_ = 0;
    twist_not_update_count_ = 0;

} // RoninSDriver()

// ~RoninSDriver()
// Destructor
RoninSDriver::~RoninSDriver()
{
} //~RoninSDriver()

int RoninSDriver::parseCanData(uint16_t id, uint8_t *in_data, uint8_t in_data_size, uint8_t out_data[], uint8_t *out_data_size)
{
    int i;

    for (int i = 0; i < in_data_size; i++)
    {
        can_buffer_[id][can_buffer_wp_[id]] = in_data[i];
        can_buffer_wp_[id]++;
        can_buffer_wp_[id] %= BUFFER_SIZE;
    }

    int can_in_buffer_size = (can_buffer_wp_[id] - can_buffer_rp_[id] + BUFFER_SIZE) % BUFFER_SIZE;

    // Data size check
    if (can_in_buffer_size < 7)
    {
        return 0;
    }

    // Search Header
    int find_flag = 0;
    for (i = 0; i < can_in_buffer_size - 7; i++)
    {
        if (can_buffer_[id][(can_buffer_rp_[id] + i) % BUFFER_SIZE] == 0x55 && can_buffer_[id][(can_buffer_rp_[id] + i + 2) % BUFFER_SIZE] == 0x04)
        {
            find_flag = 1;
            can_in_buffer_size -= i;
            can_buffer_rp_[id] += i;
            can_buffer_rp_[id] %= BUFFER_SIZE;
            break;
        }
    }

    if (find_flag == 0)
    {
        return 0;
    }

    // Check data length
    int send_data_size = can_buffer_[id][(can_buffer_rp_[id] + 1) % BUFFER_SIZE];
    if (send_data_size > can_in_buffer_size)
    {
        // Not enough data
        return 0;
    }

    // Prepare send data
    uint8_t *send_data;
    send_data = (uint8_t *)malloc(sizeof(uint8_t) * send_data_size);

    for (i = 0; i < send_data_size; i++)
    {
        int buffer_p = (can_buffer_rp_[id] + i) % BUFFER_SIZE;
        send_data[i] = can_buffer_[id][buffer_p];
    }

    // Check header crc8
    if (!verifyCRC8CheckSum(send_data, 4))
    {
        // checksum error
        // skip header
        can_buffer_rp_[id]++;
        can_buffer_rp_[id] %= BUFFER_SIZE;
        free(send_data);
        return 0;
    }

    // Check crc16
    if (!verifyCRC16CheckSum(send_data, send_data_size))
    {
        // checksum error
        // skip header
        can_buffer_rp_[id]++;
        can_buffer_rp_[id] %= BUFFER_SIZE;
        free(send_data);
        return 0;
    }

    memcpy(out_data, send_data, send_data_size);
    *out_data_size = send_data_size;

    free(send_data);
    can_buffer_rp_[id] += send_data_size;
    can_buffer_rp_[id] %= BUFFER_SIZE;
    return 1;
}

int16_t RoninSDriver::limitRoninSVel(int16_t in)
{
    int16_t out = in;
    if (out > RONIN_S_MAX_VEL)
    {
        out = RONIN_S_MAX_VEL;
    }
    if (out < RONIN_S_MIN_VEL)
    {
        out = RONIN_S_MIN_VEL;
    }
    return out;
}

void RoninSDriver::publisherTimerCallback(void)
{
    // this callback is called at 10Hz.
    // twist
    if (!twist_update_flag_)
    {
        // twist is not updated
        twist_not_update_count_++;
    }
    else
    {
        twist_update_flag_ = 0;
        twist_not_update_count_ = 0;
    }

    double limit_time_cycle = RONIN_S_COMMAND_TIMEOUT;
    if (twist_not_update_count_ > limit_time_cycle)
    {
        in_twist_.angular.x = 0.0;
        in_twist_.angular.y = 0.0;
        in_twist_.angular.z = 0.0;
    }

    double scale = 10000;
    int16_t roll = (int16_t)(out_twist_.angular.x * scale);
    int16_t pitch = (int16_t)(out_twist_.angular.y * scale);
    int16_t yaw = (int16_t)(out_twist_.angular.z * scale);

    roll = limitRoninSVel(roll);
    pitch = limitRoninSVel(pitch);
    yaw = limitRoninSVel(yaw);

    // Make Command
    uint8_t ronin_s_remote_cmd[22];
    ronin_s_remote_cmd[0] = 0x55;
    ronin_s_remote_cmd[1] = 0x16;
    ronin_s_remote_cmd[2] = 0x04;
    ronin_s_remote_cmd[3] = 0x00;
    appendCRC8CheckSum(ronin_s_remote_cmd, 4);
    ronin_s_remote_cmd[4] = 0xE5;
    ronin_s_remote_cmd[5] = 0x04;
    ronin_s_remote_cmd[6] = (uint8_t)(ronin_s_remote_cmd_counter_ & 0xFF);
    ronin_s_remote_cmd[7] = (uint8_t)(ronin_s_remote_cmd_counter_ >> 8);
    ronin_s_remote_cmd[8] = 0x00;
    ronin_s_remote_cmd[9] = 0x04;
    ronin_s_remote_cmd[10] = 0x69;
    ronin_s_remote_cmd[11] = 0x02;
    ronin_s_remote_cmd[12] = 0x07;
    ronin_s_remote_cmd[13] = 0x00;
    ronin_s_remote_cmd[14] = (uint8_t)(pitch);
    ronin_s_remote_cmd[15] = (uint8_t)(pitch >> 8);
    ronin_s_remote_cmd[16] = (uint8_t)(roll);
    ronin_s_remote_cmd[17] = (uint8_t)(roll >> 8);
    ronin_s_remote_cmd[18] = (uint8_t)(yaw);
    ronin_s_remote_cmd[19] = (uint8_t)(yaw >> 8);
    ronin_s_remote_cmd[20] = 0x00;
    ronin_s_remote_cmd[21] = 0x00;
    appendCRC16CheckSum(ronin_s_remote_cmd, 22);

    ros_kvaser_can_driver::msg::CANFrame out_msg;
    out_msg.device_channel = 0;
    out_msg.can_id = 0x012;

    for (int i = 0; i < 22; i++)
    {
        out_msg.can_data.push_back(ronin_s_remote_cmd[i]);
        if (i == 7 || i == 15)
        {
            out_msg.can_dlc = 8;
            ronin_s_driver_tx_can_pub_->publish(out_msg);
            out_msg.can_data.clear();
        }
        if (i == 21)
        {
            out_msg.can_data[6] = 0;
            out_msg.can_data[7] = 0;
            out_msg.can_dlc = 6;
            ronin_s_driver_tx_can_pub_->publish(out_msg);
            out_msg.can_data.clear();
        }
    }
    ronin_s_remote_cmd_counter_++;
}

void RoninSDriver::twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    in_twist_ = *msg;
    twist_update_flag_ = 1;
}

void RoninSDriver::canmsgCallback(const ros_kvaser_can_driver::msg::CANFrame::SharedPtr msg)
{
    uint8_t can_id_num = 0;
    uint8_t out_data_size;
    uint8_t out_data[255];

    switch (msg->can_id)
    {
    case 0x012:
        can_id_num = ID_0x012;
        break;
    case 0x021:
        can_id_num = ID_0x021;
        break;
    case 0x416:
        can_id_num = ID_0x416;
        break;
    case 0x530:
        can_id_num = ID_0x530;
        break;
    default:
        break;
    }

    uint8_t can_data[8];
    for (int i = 0; i < msg->can_dlc; i++)
    {
        can_data[i] = msg->can_data[i];
    }
    if (parseCanData(can_id_num, can_data, msg->can_dlc, out_data, &out_data_size))
    {
        switch (can_id_num)
        {
        case ID_0x012: // command
            ronin_s_remote_cmd_counter_ = ((uint16_t)out_data[7] << 8) | (uint16_t)out_data[6];
            ronin_s_remote_cmd_counter_++;
            for (int i = 0; i < out_data_size; i++)
            {
                if (debug_print_)
                {
                    printf("0x%02X,", out_data[i]);
                }
            }
            if (debug_print_)
            {
                printf("\r");
                printf("\n");
            }
            break;
        case ID_0x530:
        case ID_0x416:
            break;
        case ID_0x021: // ronin s state
            if (out_data[1] == 0xBE)
            { // Angle
                rclcpp::Clock system_clock(RCL_SYSTEM_TIME);
                auto time_now = system_clock.now();
                // Extract seconds and nanoseconds from the time_now
                auto nanoseconds = time_now.nanoseconds();
                uint16_t seq = ((uint16_t)out_data[7] << 8) | (uint16_t)out_data[6];
                int counter = 0;
                int8_t roll_torque_percent = 0;
                int8_t pitch_torque_percent = 0;
                int8_t yaw_torque_percent = 0;
                float relative_roll = 0;
                float relative_pitch = 0;
                float relative_yaw = 0;
                int16_t roll_velocity = 0;
                int16_t pitch_velocity = 0;
                int16_t yaw_velocity = 0;
                float target_roll = 0;
                float target_pitch = 0;
                float target_yaw = 0;
                float actual_roll = 0;
                float actual_pitch = 0;
                float actual_yaw = 0;

                for (int i = 12; i < out_data_size; i++)
                {
                    if (out_data[i] == counter)
                    {
                        i++;
                        int data_size = out_data[i];
                        if (data_size == 1)
                        {
                            i++;
                            if (debug_print_)
                            {
                                printf("%4d,", (int8_t)out_data[i]);
                            }

                            switch (counter)
                            {
                            case 6:
                                pitch_torque_percent = (int8_t)out_data[i];
                                break;
                            case 7:
                                roll_torque_percent = (int8_t)out_data[i];
                                break;
                            case 8:
                                yaw_torque_percent = (int8_t)out_data[i];
                                break;
                            default:
                                break;
                            }
                        }
                        if (data_size == 2)
                        {
                            i++;
                            i++;
                            int16_t data = (int16_t)((((uint16_t)out_data[i]) << 8) | (uint16_t)out_data[i - 1]);
                            if (debug_print_)
                            {
                                if (counter >= 31 && counter <= 36)
                                {
                                    printf("%6.1f,", data / 10.0);
                                }
                                else // if(counter >= 31 && counter <= 36 )
                                {
                                    printf("%4d,", data);
                                }
                            }
                            switch (counter)
                            {
                            case 16:
                                relative_pitch = data / 10.0;
                                break;
                            case 17:
                                relative_roll = data / 10.0;
                                break;
                            case 18:
                                relative_yaw = data / 10.0;
                                break;
                            case 19:
                                pitch_velocity = data;
                                break;
                            case 20:
                                roll_velocity = data;
                                break;
                            case 21:
                                yaw_velocity = data;
                                break;
                            case 31:
                                target_pitch = data / 10.0;
                                break;
                            case 32:
                                target_roll = data / 10.0;
                                break;
                            case 33:
                                target_yaw = data / 10.0;
                                break;
                            case 34:
                                actual_pitch = data / 10.0;
                                break;
                            case 35:
                                actual_roll = data / 10.0;
                                break;
                            case 36:
                                actual_yaw = data / 10.0;
                                break;
                            default:
                                break;
                            }
                        }
                    }
                    counter++;
                }
                if (debug_print_)
                {
                    printf("\r");
                    printf("\n");
                }
                ros_ronin_s_driver::msg::DJIRoninSStatus pub_data;
                pub_data.header.frame_id = std::string("ronin-s");
                pub_data.header.stamp.sec = nanoseconds / 1000000000;     // converting nanoseconds to seconds
                pub_data.header.stamp.nanosec = nanoseconds % 1000000000; // the remaining nanoseconds
                pub_data.roll_torque_percent = roll_torque_percent;
                pub_data.pitch_torque_percent = pitch_torque_percent;
                pub_data.yaw_torque_percent = yaw_torque_percent;
                pub_data.relative_roll = relative_roll;
                pub_data.relative_pitch = relative_pitch;
                pub_data.relative_yaw = relative_yaw;
                pub_data.roll_velocity = roll_velocity;
                pub_data.pitch_velocity = pitch_velocity;
                pub_data.yaw_velocity = yaw_velocity;
                pub_data.target_roll = target_roll;
                pub_data.target_pitch = target_pitch;
                pub_data.target_yaw = target_yaw;
                pub_data.actual_roll = actual_roll;
                pub_data.actual_pitch = actual_pitch;
                pub_data.actual_yaw = actual_yaw;
                pub_data.seq = seq;

                ronin_s_driver_info_pub_->publish(pub_data);
            }
            break;
        default:
            break;
        }
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RoninSDriver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}