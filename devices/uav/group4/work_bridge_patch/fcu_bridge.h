// fcu_bridge.h
#include <stdio.h>
#include <string.h>
#include <chrono>
#include <thread>
#include <eigen3/Eigen/Dense>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <serial/serial.h>
#include "../mavlink/common/mavlink.h"
#include "./ringbuffer.h"

#define BUF_SIZE 32768
#define DRONE_PORT 333

class fcu_bridge
{
public:
    fcu_bridge(int id) {
        drone_id = id;
    };
    ~fcu_bridge() = default;

    rclcpp::Node::SharedPtr node;

	// Original private members
    int drone_id;

    std::string drone_ip_;
    std::string usb_port_;
    int baudrate_;
    serial::Serial ser_;

    // Converted member variables from globals
    int channel_;
    mavlink_channel_t mav_chan_;
    bool offboard_;
    bool use_uwb_;
    bool set_goal_;
    bool simple_target_;

	int get_drone_;
    struct sockaddr_in drone_addr_;
    int socket_cli_;
    mavlink_system_t mavlink_system_;
    mavlink_message_t msg_received_;
    mavlink_status_t status_;
    mavlink_scaled_imu_t imu_;
    mavlink_global_vision_position_estimate_t pose_;
    mavlink_global_position_int_t position_;
    mavlink_battery_status_t batt_;
    mavlink_heartbeat_t heartbeat_;
    mavlink_command_long_t cmd_long_;
    mavlink_attitude_quaternion_t attitude_quaternion_;
    mavlink_set_position_target_local_ned_t set_position_target_;
    mavlink_set_gps_global_origin_t gps_global_origin_, set_gps_global_origin_;
    
    uint8_t buffer_[BUF_SIZE];
    uint8_t TxBuffer_buf_[BUF_SIZE];
    uint8_t TxBuffer_[BUF_SIZE];
    uint8_t RxBuffer_[BUF_SIZE];
    
    double time_start_;
    double time_odom_;
    bool armed_;
    bool get_gnss_origin_;

    RingBuffer mav_buf_send_;
    RingBuffer mav_buf_receive_;

    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr cmd;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr mission_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr motion_sub;

    nav_msgs::msg::Odometry odom_pub;
    sensor_msgs::msg::Imu imu_pub;
    sensor_msgs::msg::NavSatFix gnss_pub;
    nav_msgs::msg::Path path_pub;
    nav_msgs::msg::Path path_target;
    geometry_msgs::msg::PoseStamped odomPose;
    geometry_msgs::msg::PoseStamped odom_target;
    std_msgs::msg::Int16 cmd_pub;
    geometry_msgs::msg::PoseStamped goal_pub;
    std_msgs::msg::Float32 batt_pub;
    std_msgs::msg::Float32 batt_remaining_pub;
    std_msgs::msg::Bool armed_pub;
    

    
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_global;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_global;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_global;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_global;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_target_pub;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr command;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr batt;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr batt_remaining;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr armed_state;
    

	inline void init() {
		mav_chan_ = static_cast<mavlink_channel_t>(channel_);
		mavlink_system_.sysid = 254;
		mavlink_system_.compid=MAV_COMP_ID_MISSIONPLANNER;
		rbInit(&mav_buf_send_, TxBuffer_, BUF_SIZE);
		rbInit(&mav_buf_receive_, RxBuffer_, BUF_SIZE);
        RCLCPP_INFO(node->get_logger(),"fcu_bridge %03d init done",drone_id);
	}

    int serial_connect();
    int wifi_connect();
    void mav_send_heartbeat(void);
    void mav_send_arm();
    void mav_send_disarm();
    void mav_send_takeoff();
    void mav_send_land();
    void mav_send_follow(float mode);
    void mav_send_buffer(mavlink_channel_t chan, char *buf, uint16_t len);
    void mavlink_send_msg(mavlink_channel_t chan, mavlink_message_t *msg);

    //设置目标。注意：这里输入的目标都是全局坐标系下的目标值
    mavlink_set_position_target_local_ned_t set_position_target_local_ned;
    mavlink_message_t msg_position_target_local_ned;

    void mav_send_target(float target_pos_x, float target_pos_y, float target_pos_z, //单位：m
                    float target_vel_x, float target_vel_y, float target_vel_z,  //单位：m/s
                    float target_acc_x, float target_acc_y, float target_acc_z,  //单位：m/ss
                    float target_yaw, float target_yaw_rate);
    
    // Method declarations for functionality
    virtual void flush_data() = 0;
    virtual void parse_data() = 0;
};

int fcu_bridge::serial_connect() {
    try{
    //设置串口属性，并打开串口
        ser_.setPort(usb_port_.c_str());
        ser_.setBaudrate(baudrate_);
        serial::Timeout to = serial::Timeout::simpleTimeout(5000);
        ser_.setTimeout(to);
        ser_.open();
    }catch (serial::IOException& e)
    {
        printf("Unable to open port \n");
        return -1;
    }

    //检测串口是否已经打开，并给出提示信息
    if(ser_.isOpen())
    {
        printf("Serial Port initialized\n");
    }
    else
    {
        return -1;
    }
}

int fcu_bridge::wifi_connect() {
    socket_cli_ = socket(AF_INET, SOCK_STREAM, 0);
    if(socket_cli_ < 0) {
        RCLCPP_INFO(node->get_logger(),"socket error!");
        return -1;
    }
    memset(&drone_addr_, 0, sizeof(drone_addr_));
    drone_addr_.sin_family      = AF_INET;
    drone_addr_.sin_port        = htons(DRONE_PORT);
    drone_addr_.sin_addr.s_addr = inet_addr(drone_ip_.c_str());
    RCLCPP_INFO(node->get_logger(),"fcu_bridge %03d connecting...",drone_id);

    get_drone_ = connect(socket_cli_, (struct sockaddr*)&drone_addr_, sizeof(drone_addr_));
    if(get_drone_<0) {
        RCLCPP_INFO(node->get_logger(),"fcu_bridge %03d connect error!",drone_id);
        return -1;
    }
    else {
        RCLCPP_INFO(node->get_logger(),"fcu_bridge %03d connect succeed!",drone_id);
    }

    int tcp_delay=1;
    setsockopt(socket_cli_, IPPROTO_TCP, TCP_NODELAY, (void*)&tcp_delay, sizeof(tcp_delay));
    int flag = fcntl(socket_cli_,F_GETFL,0);//获取socket_cli当前的状态
    if(flag<0){
        RCLCPP_INFO(node->get_logger(),"fcntl F_GETFL fail");
        close(socket_cli_);
        return -1;
    }
    fcntl(socket_cli_, F_SETFL, flag | O_NONBLOCK);//设置为非阻塞态
}

void fcu_bridge::mav_send_heartbeat(void){
    mavlink_message_t msg_heartbeat;
    mavlink_heartbeat_t heartbeat;
        if(offboard_){//串口
            heartbeat.type=MAV_TYPE_ONBOARD_CONTROLLER;
        }else{
            heartbeat.type=MAV_TYPE_GCS;
        }
    heartbeat.autopilot=MAV_AUTOPILOT_INVALID;
    heartbeat.base_mode=MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    mavlink_msg_heartbeat_encode(mavlink_system_.sysid, mavlink_system_.compid, &msg_heartbeat, &heartbeat);
    mavlink_send_msg(mav_chan_, &msg_heartbeat);
}

void fcu_bridge::mav_send_arm(void){
    mavlink_set_mode_t setmode;
    mavlink_message_t msg_setmode;
    setmode.base_mode=MAV_MODE_AUTO_ARMED;
    mavlink_msg_set_mode_encode(mavlink_system_.sysid, mavlink_system_.compid, &msg_setmode, &setmode);
    mavlink_send_msg(mav_chan_, &msg_setmode);
}

void fcu_bridge::mav_send_disarm(void){
    mavlink_set_mode_t setmode;
    mavlink_message_t msg_setmode;
    setmode.base_mode=MAV_MODE_AUTO_DISARMED;
    mavlink_msg_set_mode_encode(mavlink_system_.sysid, mavlink_system_.compid, &msg_setmode, &setmode);
    mavlink_send_msg(mav_chan_, &msg_setmode);
}

void fcu_bridge::mav_send_takeoff(void){
    mavlink_command_long_t command_long;
    mavlink_message_t msg_command_long;
    command_long.command=MAV_CMD_NAV_TAKEOFF;
    mavlink_msg_command_long_encode(mavlink_system_.sysid, mavlink_system_.compid, &msg_command_long, &command_long);
    mavlink_send_msg(mav_chan_, &msg_command_long);
}

void fcu_bridge::mav_send_land(void){
    mavlink_command_long_t command_long = {};
    mavlink_message_t msg_command_long;
    command_long.command=MAV_CMD_NAV_LAND;
    // param1-7 显式清零：空速默认0、经纬度0=当前位置、高度0=当前高度，避免未初始化垃圾值
    mavlink_msg_command_long_encode(mavlink_system_.sysid, mavlink_system_.compid, &msg_command_long, &command_long);
    mavlink_send_msg(mav_chan_, &msg_command_long);
    RCLCPP_INFO(node->get_logger(),"[SAFE-LAND] cmd=%d p1=%.1f p2=%.1f p3=%.1f p4=%.1f p5=%.1f p6=%.1f p7=%.1f", command_long.command, command_long.param1, command_long.param2, command_long.param3, command_long.param4, command_long.param5, command_long.param6, command_long.param7);
}

void fcu_bridge::mav_send_follow(float mode){
	mavlink_message_t msg_command_long;
	mavlink_command_long_t command_long;
	command_long.command=MAV_CMD_DO_FOLLOW;
	command_long.param1=mode;
	mavlink_msg_command_long_encode(mavlink_system_.sysid, mavlink_system_.compid, &msg_command_long, &command_long);
	mavlink_send_msg(mav_chan_, &msg_command_long);
}

void fcu_bridge::mav_send_buffer(mavlink_channel_t chan, char *buf, uint16_t len) {
    for(uint16_t i=0; i<len; i++){
    rbPush(&mav_buf_send_, buf[i]);
    }
}

void fcu_bridge::mavlink_send_msg(mavlink_channel_t chan, mavlink_message_t *msg) {
    uint8_t ck[2];

    ck[0] = (uint8_t)(msg->checksum & 0xFF);
    ck[1] = (uint8_t)(msg->checksum >> 8);
    // XXX use the right sequence here

    mav_send_buffer(chan, (char *)&msg->magic, MAVLINK_NUM_HEADER_BYTES);
    mav_send_buffer(chan, (char *)&msg->payload64, msg->len);
    mav_send_buffer(chan, (char *)ck, 2);
}

void fcu_bridge::mav_send_target(float target_pos_x, float target_pos_y, float target_pos_z, //单位：m
                    float target_vel_x, float target_vel_y, float target_vel_z,  //单位：m/s
                    float target_acc_x, float target_acc_y, float target_acc_z,  //单位：m/ss
                    float target_yaw, float target_yaw_rate){                    //单位：rad, rad/s
    if(rclcpp::Clock().now().seconds()<time_start_+5.0f){
		return;
	}
    if(set_goal_){
        RCLCPP_INFO(node->get_logger(),"set_goal");
        set_position_target_local_ned.coordinate_frame=MAV_FRAME_MISSION;
    }else{
    if(simple_target_){
        set_position_target_local_ned.coordinate_frame=MAV_FRAME_GLOBAL;
    }else{
        set_position_target_local_ned.coordinate_frame=MAV_FRAME_VISION_NED;
    }
    path_target.header.frame_id = "map";
    path_target.header.stamp = rclcpp::Clock().now();
    odom_target.pose.position.x=target_pos_x;
    odom_target.pose.position.y=-target_pos_y;//FRU->FLU
    odom_target.pose.position.z=target_pos_z;
    // RCLCPP_INFO(node->get_logger(),"odom_target.pose.position.x=%f",odom_target.pose.position.x);
    path_target.poses.push_back(odom_target);
    path_target_pub->publish(path_target);
    }
    set_position_target_local_ned.x=target_pos_x;
    set_position_target_local_ned.y=target_pos_y;
    set_position_target_local_ned.z=target_pos_z;
    set_position_target_local_ned.vx=target_vel_x;
    set_position_target_local_ned.vy=target_vel_y;
    set_position_target_local_ned.vz=target_vel_z;
    set_position_target_local_ned.afx=target_acc_x;
    set_position_target_local_ned.afy=target_acc_y;
    set_position_target_local_ned.afz=target_acc_z;
    set_position_target_local_ned.yaw=target_yaw;
    // RCLCPP_INFO(node->get_logger(),"set_position_target_local_ned.yaw=%f",set_position_target_local_ned.yaw);
    set_position_target_local_ned.yaw_rate=0.0f;
    mavlink_msg_set_position_target_local_ned_encode(mavlink_system_.sysid, mavlink_system_.compid, &msg_position_target_local_ned, &set_position_target_local_ned);
    mavlink_send_msg(mav_chan_, &msg_position_target_local_ned);
}
