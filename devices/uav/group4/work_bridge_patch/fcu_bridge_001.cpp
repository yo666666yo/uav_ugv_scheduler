#include <rclcpp/rclcpp.hpp>
#include "fcu_bridge.h"

class fcu_bridge_001 : public fcu_bridge
{
public:
    fcu_bridge_001(int id) : fcu_bridge(id) {}
    void flush_data() override {
      uint16_t length=rbGetCount(&mav_buf_send_);
      if(length>0){
          for(uint16_t i=0; i<length; i++){
            TxBuffer_buf_[i]=rbPop(&mav_buf_send_);
          }
          if (mav_chan_ == MAVLINK_COMM_0){
            ser_.write(TxBuffer_buf_,length);
          }else{
            send(socket_cli_, TxBuffer_buf_, length, 0);
          }
      }
    }
    void parse_data() override {
      int chan = 0;
      uint16_t n=rbGetCount(&mav_buf_receive_);
      if(n) {
        for(int i=0; i<n; i++) {
          if (mavlink_parse_char(chan,rbPop(&mav_buf_receive_), &msg_received_, &status_)){
              switch (msg_received_.msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT:
                  mavlink_msg_heartbeat_decode(&msg_received_, &heartbeat_);
                  RCLCPP_INFO(node->get_logger(),"%03d Received heartbeat time: %fs, voltage:%fV, current:%fA, sat:%d, gnss:%d",drone_id, rclcpp::Clock().now().seconds()-time_start_, (double)batt_.voltages[1]/1000, (double)batt_.current_battery/100, position_.hdg&0xFF, position_.hdg>>12);
                  if((heartbeat_.base_mode&MAV_MODE_FLAG_SAFETY_ARMED)==0){
                    armed_=false;
                  }else{
                    if(!armed_){
                      cmd_pub.data=1101;
                      command->publish(cmd_pub);
                    }
                    armed_=true;
                  }
                  armed_pub.data = armed_;
                  armed_state->publish(armed_pub);
                  mav_send_heartbeat();
                  break;
                case MAVLINK_MSG_ID_BATTERY_STATUS:
                  mavlink_msg_battery_status_decode(&msg_received_, &batt_);
                  if (offboard_ == false) {
                    // Publish pack voltage in volts; cell[1] is the vendor's
                    // configured total-voltage field for this aircraft.
                    batt_pub.data = static_cast<float>(batt_.voltages[1]) / 1000.0f;
                    batt->publish(batt_pub);
                    // MAVLink uses -1/0 when remaining percentage is unknown.
                    batt_remaining_pub.data = static_cast<float>(batt_.battery_remaining);
                    batt_remaining->publish(batt_remaining_pub);
                  }
                  break;
                case MAVLINK_MSG_ID_COMMAND_LONG:
                  mavlink_msg_command_long_decode(&msg_received_, &cmd_long_);
                  switch(cmd_long_.command){
                    case MAV_CMD_DO_FOLLOW:
                      if(cmd_long_.param1==1.0f){//前视追踪
                        if(offboard_){
                          cmd_pub.data=1001;
                          command->publish(cmd_pub);
                        }
                      }else if(cmd_long_.param1==2.0f){//下视追踪
                        if(offboard_){
                          cmd_pub.data=1002;
                          command->publish(cmd_pub);
                        }
                      }else if(cmd_long_.param1==3.0f){//停止追踪
                        if(offboard_){
                          cmd_pub.data=1003;
                          command->publish(cmd_pub);
                        }
                      }
                      break;
                    default:
                      break;
                  }
                  break;
                case MAVLINK_MSG_ID_SCALED_IMU:
                  // RCLCPP_INFO(node->get_logger(),"MAVLINK_MSG_ID_SCALED_IMU");
                  mavlink_msg_scaled_imu_decode(&msg_received_, &imu_);
                  imu_pub.header.frame_id = "scaled_imu";
                  imu_pub.header.stamp = rclcpp::Clock().now();
                  imu_pub.linear_acceleration.x =(double)imu_.xacc/1000;
                  imu_pub.linear_acceleration.y = -(double)imu_.yacc/1000;
                  imu_pub.linear_acceleration.z = -(double)imu_.zacc/1000;
                  imu_pub.angular_velocity.x = (double)imu_.xgyro/1000;
                  imu_pub.angular_velocity.y = -(double)imu_.ygyro/1000;
                  imu_pub.angular_velocity.z = -(double)imu_.zgyro/1000;
                  imu_global->publish(imu_pub);
                  // RCLCPP_INFO(node->get_logger(),"acc0,acc1,acc2,gyr0,gyr1,gyr2: %f %f %f %f %f %f\n",
                  //     (double)imu_.xacc/1000,-(double)imu_.yacc/1000,-(double)imu_.zacc/1000,
                  //     (double)imu_.xgyro/1000,-(double)imu_.ygyro/1000,-(double)imu_.zgyro/1000);
                  break;
                case	MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                  // RCLCPP_INFO(node->get_logger(),"MAVLINK_MSG_ID_GLOBAL_POSITION_INT");
                  mavlink_msg_global_position_int_decode(&msg_received_, &position_);
                  gnss_pub.header.frame_id = "gnss_global";
                  gnss_pub.header.stamp = rclcpp::Clock().now();
                  gnss_pub.latitude = (double)position_.lat*1e-7;
                  gnss_pub.longitude = (double)position_.lon*1e-7;
                  gnss_pub.altitude = (double)position_.alt*1e-3;
                  gnss_global->publish(gnss_pub);
                  break;
                case MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN:
                  // RCLCPP_INFO(node->get_logger(),"MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN");
                  mavlink_msg_set_gps_global_origin_decode(&msg_received_, &set_gps_global_origin_);
                  if(set_gps_global_origin_.latitude==gps_global_origin_.latitude&&
                    set_gps_global_origin_.longitude==gps_global_origin_.longitude&&
                    set_gps_global_origin_.altitude==gps_global_origin_.altitude ){
                    get_gnss_origin_=true;
                  }
                  break;
                case  MAVLINK_MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE:
                  // RCLCPP_INFO(node->get_logger(),"MAVLINK_MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE");
                  mavlink_msg_global_vision_position_estimate_decode(&msg_received_, &pose_);
                  odom_pub.header.frame_id = "map";
                  odom_pub.header.stamp = rclcpp::Clock().now();
                  float quaternion_odom[4];
                  mavlink_euler_to_quaternion(pose_.roll, -pose_.pitch, -pose_.yaw, quaternion_odom);
                  odom_pub.pose.pose.orientation.w=quaternion_odom[0];
                  odom_pub.pose.pose.orientation.x=quaternion_odom[1];
                  odom_pub.pose.pose.orientation.y=quaternion_odom[2];
                  odom_pub.pose.pose.orientation.z=quaternion_odom[3];
                  odom_pub.pose.pose.position.x=pose_.x*0.01;
                  odom_pub.pose.pose.position.y=-pose_.y*0.01;
                  odom_pub.pose.pose.position.z=pose_.z*0.01;
                  odom_pub.twist.twist.linear.x=position_.vx*0.01;
                  odom_pub.twist.twist.linear.y=-position_.vy*0.01;
                  odom_pub.twist.twist.linear.z=position_.vz*0.01;
                  odom_global->publish(odom_pub);
                  // RCLCPP_INFO(node->get_logger(),"pose_x,pose_y,pose_z,pose_roll,pose_pitch,pose_yaw: %f %f %f %f %f %f",
                  //   pose_.x*0.01,pose_.y*0.01,pose_.z*0.01,pose_.roll,pose_.pitch,pose_.yaw);
                  if(use_uwb_&&(position_.lat==0||position_.lon==0)){
                    break;
                  }
                  odomPose.header = odom_pub.header;
                  odomPose.pose = odom_pub.pose.pose;
                  path_pub.header.stamp = odom_pub.header.stamp;
                  // RCLCPP_INFO(node->get_logger(),"odomPose.pose.position.x=%f",odomPose.pose.position.x);
                  path_pub.poses.push_back(odomPose);
                  path_pub.header.frame_id = "map";
                  path_global->publish(path_pub);
                  break;
                case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED://仅机载电脑接收该数据包
                  mavlink_msg_set_position_target_local_ned_decode(&msg_received_, &set_position_target_);
                  if(set_position_target_.coordinate_frame==MAV_FRAME_MISSION){//设置自主飞行的goal point,把前右上坐标转换为前左上
                    goal_pub.header.frame_id = "map";
                    goal_pub.header.stamp = rclcpp::Clock().now();
                    goal_pub.pose.position.x=set_position_target_.x;
                    goal_pub.pose.position.y=-set_position_target_.y;
                    goal_pub.pose.position.z=set_position_target_.z;
                    goal->publish(goal_pub);
                  }
              break;
                case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
                  //RCLCPP_INFO(node->get_logger(),"MAVLINK_MSG_ID_ATTITUDE_QUATERNION");
                  mavlink_msg_attitude_quaternion_decode(&msg_received_, &attitude_quaternion_);
                  imu_pub.orientation.w=attitude_quaternion_.q1;
                  imu_pub.orientation.x=attitude_quaternion_.q2;
                  imu_pub.orientation.y=attitude_quaternion_.q3;
                  imu_pub.orientation.z=attitude_quaternion_.q4;
                  // printf("q1,q2,q3,q4,g1,g2,g3: %f %f %f %f %f %f %f\n",attitude_quaternion.q1, attitude_quaternion.q2, attitude_quaternion.q3, attitude_quaternion.q4,
                  break;
                default:
                  break;
              }
          }
        }
      }
    }
    void cmdHandler(const std_msgs::msg::Int16::SharedPtr cmd){
      switch(cmd->data){
        case 1:
            mav_send_arm();
            break;
        case 2:
            mav_send_disarm();
            break;
        case 3:
            mav_send_takeoff();
            break;
        case 4:
            // 安全降落：连发3次确保飞控收到（应对TCP丢包/状态竞争）
            for(int _i=0;_i<3;_i++){
                mav_send_land();
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }
            break;
        case 1011:
            mav_send_follow(1.0f);
            break;
        case 1012:
            mav_send_follow(2.0f);
            break;
        case 1013:
            mav_send_follow(3.0f);
            break;
        default:
            break;
      }
    }

    void odomHandler(const nav_msgs::msg::Odometry::SharedPtr odom)
    {
      if(time_odom_<=time_start_||mav_chan_ == MAVLINK_COMM_0){//USB传输无需降频
        time_odom_=rclcpp::Clock().now().seconds();
      }else{
        if(rclcpp::Clock().now().seconds()-time_odom_<0.1){//用网络通信，如果odom频率过高进行降频，降至10hz以下
          return;
        }
        time_odom_=rclcpp::Clock().now().seconds();
      }

      Eigen::Vector3f position_map ((float)odom->pose.pose.position.x, -(float)odom->pose.pose.position.y, -(float)odom->pose.pose.position.z) ;
      float quaternion_odom[4]={(float)odom->pose.pose.orientation.w,
                                (float)odom->pose.pose.orientation.x,
                                (float)odom->pose.pose.orientation.y,
                                (float)odom->pose.pose.orientation.z};

      float roll, pitch, yaw;
      mavlink_quaternion_to_euler(quaternion_odom, &roll, &pitch, &yaw);
      RCLCPP_INFO(node->get_logger(),"x:%f,y:%f,z:%f,yaw:%f\n",position_map.x(),position_map.y(),position_map.z(),yaw);

      mavlink_message_t msg_local_position_ned, msg_attitude;
      mavlink_attitude_t attitude;
      mavlink_local_position_ned_t local_position_ned;

      attitude.yaw = -yaw;
      mavlink_msg_attitude_encode(mavlink_system_.sysid, mavlink_system_.compid, &msg_attitude, &attitude);
      mavlink_send_msg(mav_chan_, &msg_attitude);

      local_position_ned.x=position_map.x();
      local_position_ned.y=position_map.y();
      local_position_ned.z=position_map.z();
      mavlink_msg_local_position_ned_encode(mavlink_system_.sysid, mavlink_system_.compid, &msg_local_position_ned, &local_position_ned);
      mavlink_send_msg(mav_chan_, &msg_local_position_ned);
    }
    void missionHandler(const std_msgs::msg::Float32MultiArray::SharedPtr mission){
        float yaw=0.0f, yaw_rate=0.0f;
        float px=0.0f, py=0.0f, pz=0.0f;
        float vx=0.0f, vy=0.0f, vz=0.0f;
        float ax=0.0f, ay=0.0f, az=0.0f;
        // RCLCPP_INFO(node->get_logger(),"missionHandler");
        if (mission->data.size() == 11){
          yaw=mission->data[0];
          yaw_rate=mission->data[1];
          px=mission->data[2];
          py=mission->data[3];
          pz=mission->data[4];
          vx=mission->data[5];
          vy=mission->data[6];
          vz=mission->data[7];
          ax=mission->data[8];
          ay=mission->data[9];
          az=mission->data[10];
          // RCLCPP_INFO(node->get_logger(),"px:%f,py:%f,pz:%f,yaw:%f",px,py,pz,yaw);
          mav_send_target(
            px, py, pz,
            vx, vy, vz,
            ax, ay, az,
            yaw, yaw_rate
          );
        }
    }
    void gnssHandler(const sensor_msgs::msg::NavSatFix::SharedPtr gnss){
      if(gnss->latitude==0.0f||gnss->longitude==0.0f||get_gnss_origin_){
        return;
      }
      mavlink_message_t msg_gnss_origin;
      gps_global_origin_.latitude=gnss->latitude*1e7;
      gps_global_origin_.longitude=gnss->longitude*1e7;
      gps_global_origin_.altitude=gnss->altitude*100;//m->cm
      mavlink_msg_set_gps_global_origin_encode(mavlink_system_.sysid, mavlink_system_.compid, &msg_gnss_origin, &gps_global_origin_);
      mavlink_send_msg(mav_chan_, &msg_gnss_origin);
    }
    void motionHandler(const geometry_msgs::msg::PoseStamped::SharedPtr odom)
    {
      if(time_odom_<=time_start_||mav_chan_ == MAVLINK_COMM_0){//USB传输无需降频
        time_odom_=rclcpp::Clock().now().seconds();
      }else{
        if(rclcpp::Clock().now().seconds()-time_odom_<0.1){//用网络通信，如果odom频率过高进行降频，降至10hz以下
          return;
        }
        time_odom_=rclcpp::Clock().now().seconds();
      }

      Eigen::Vector3f position_map ((float)odom->pose.position.x, -(float)odom->pose.position.y, -(float)odom->pose.position.z) ;
      float quaternion_odom[4]={(float)odom->pose.orientation.w,
                                (float)odom->pose.orientation.x,
                                (float)odom->pose.orientation.y,
                                (float)odom->pose.orientation.z};

      float roll, pitch, yaw;
      mavlink_quaternion_to_euler(quaternion_odom, &roll, &pitch, &yaw);
      // printf("x:%f,y:%f,z:%f,yaw:%f\n",position_map.x(),position_map.y(),position_map.z(),yaw);
      //动捕一般为前左上坐标系，需要改为前右下坐标系发给飞控
      mavlink_message_t msg_local_position_ned, msg_attitude;
      mavlink_attitude_t attitude;
      mavlink_local_position_ned_t local_position_ned;

      attitude.yaw = -yaw;
      mavlink_msg_attitude_encode(mavlink_system_.sysid, mavlink_system_.compid, &msg_attitude, &attitude);
      mavlink_send_msg(mav_chan_, &msg_attitude);

      local_position_ned.x=position_map.x();
      local_position_ned.y=position_map.y();
      local_position_ned.z=position_map.z();
      mavlink_msg_local_position_ned_encode(mavlink_system_.sysid, mavlink_system_.compid, &msg_local_position_ned, &local_position_ned);
      mavlink_send_msg(mav_chan_, &msg_local_position_ned);
    }
};

int main (int argc,char ** argv)
{
    rclcpp::init(argc,argv);

    auto fcu_bridge_instance = std::make_shared<fcu_bridge_001>(1);

    fcu_bridge_instance->node = rclcpp::Node::make_shared("fcu_bridge_001");

    fcu_bridge_instance->node->declare_parameter<std::string>("DRONE_IP", "192.168.0.201");
    fcu_bridge_instance->node->declare_parameter<std::string>("USB_PORT", "/dev/ttyACM0");
    fcu_bridge_instance->node->declare_parameter<int>("BANDRATE", 460800);
    fcu_bridge_instance->node->declare_parameter<int>("channel", 1);
    fcu_bridge_instance->node->declare_parameter<bool>("offboard", false);
    fcu_bridge_instance->node->declare_parameter<bool>("use_uwb", false);
    fcu_bridge_instance->node->declare_parameter<bool>("set_goal", false);
    fcu_bridge_instance->node->declare_parameter<bool>("simple_target", true);

    fcu_bridge_instance->node->get_parameter("DRONE_IP", fcu_bridge_instance->drone_ip_);
    fcu_bridge_instance->node->get_parameter("USB_PORT", fcu_bridge_instance->usb_port_);
    fcu_bridge_instance->node->get_parameter("BANDRATE", fcu_bridge_instance->baudrate_);
    fcu_bridge_instance->node->get_parameter("channel", fcu_bridge_instance->channel_);
    fcu_bridge_instance->node->get_parameter("offboard", fcu_bridge_instance->offboard_);
    fcu_bridge_instance->node->get_parameter("use_uwb", fcu_bridge_instance->use_uwb_);
    fcu_bridge_instance->node->get_parameter("set_goal", fcu_bridge_instance->set_goal_);
    fcu_bridge_instance->node->get_parameter("simple_target", fcu_bridge_instance->simple_target_);
    
    fcu_bridge_instance->init();

    auto loop_rate = rclcpp::Rate(200); // 200Hz

    fcu_bridge_instance->cmd = fcu_bridge_instance->node->create_subscription<std_msgs::msg::Int16>(
      "command", 100, std::bind(&fcu_bridge_001::cmdHandler, fcu_bridge_instance, std::placeholders::_1));
    fcu_bridge_instance->gnss_sub = fcu_bridge_instance->node->create_subscription<sensor_msgs::msg::NavSatFix>(
      "gnss_global_001", 100, std::bind(&fcu_bridge_001::gnssHandler, fcu_bridge_instance, std::placeholders::_1));
    fcu_bridge_instance->odom_sub = fcu_bridge_instance->node->create_subscription<nav_msgs::msg::Odometry>(
      "odometry_001", 100, std::bind(&fcu_bridge_001::odomHandler, fcu_bridge_instance, std::placeholders::_1));
    fcu_bridge_instance->mission_sub = fcu_bridge_instance->node->create_subscription<std_msgs::msg::Float32MultiArray>(
      "mission_001", 100, std::bind(&fcu_bridge_001::missionHandler, fcu_bridge_instance, std::placeholders::_1));
    fcu_bridge_instance->motion_sub = fcu_bridge_instance->node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "motion_001", 100, std::bind(&fcu_bridge_001::motionHandler, fcu_bridge_instance, std::placeholders::_1));
    fcu_bridge_instance->odom_global = fcu_bridge_instance->node->create_publisher<nav_msgs::msg::Odometry>("odom_global_001", 100);
    fcu_bridge_instance->imu_global = fcu_bridge_instance->node->create_publisher<sensor_msgs::msg::Imu>("imu_global_001", 100);
    fcu_bridge_instance->gnss_global = fcu_bridge_instance->node->create_publisher<sensor_msgs::msg::NavSatFix>("gnss_global_001", 100);
    fcu_bridge_instance->path_global = fcu_bridge_instance->node->create_publisher<nav_msgs::msg::Path>("path_global_001", 100);
    fcu_bridge_instance->path_target_pub = fcu_bridge_instance->node->create_publisher<nav_msgs::msg::Path>("path_target_001", 100);
    fcu_bridge_instance->command = fcu_bridge_instance->node->create_publisher<std_msgs::msg::Int16>("command", 100);
    fcu_bridge_instance->goal = fcu_bridge_instance->node->create_publisher<geometry_msgs::msg::PoseStamped>("goal_001", 100);
    fcu_bridge_instance->batt = fcu_bridge_instance->node->create_publisher<std_msgs::msg::Float32>("batt_now_001", 100);
    fcu_bridge_instance->batt_remaining = fcu_bridge_instance->node->create_publisher<std_msgs::msg::Float32>("battery_remaining_001", 100);
    fcu_bridge_instance->armed_state = fcu_bridge_instance->node->create_publisher<std_msgs::msg::Bool>("armed_001", 100);
    
    std::this_thread::sleep_for(std::chrono::seconds(1));

    if(fcu_bridge_instance->set_goal_){
      fcu_bridge_instance->cmd_pub.data=101;
      fcu_bridge_instance->command->publish(fcu_bridge_instance->cmd_pub);
    }
    if(fcu_bridge_instance->offboard_){
      fcu_bridge_instance->cmd_pub.data=102;
      fcu_bridge_instance->command->publish(fcu_bridge_instance->cmd_pub);
    }

    if(fcu_bridge_instance->channel_==0){
      fcu_bridge_instance->serial_connect();
    }
    else if (fcu_bridge_instance->channel_==1){
      fcu_bridge_instance->wifi_connect();
    }

    fcu_bridge_instance->time_start_ = rclcpp::Clock().now().seconds();
    
    int n = 0;

    while (rclcpp::ok()) {
      rclcpp::spin_some(fcu_bridge_instance->node);
      if (fcu_bridge_instance->channel_==0){
        n=fcu_bridge_instance->ser_.available();
        if(n){
          fcu_bridge_instance->ser_.read(fcu_bridge_instance->buffer_, n);
        }
      }
      else if (fcu_bridge_instance->channel_==1){
        n = recv(fcu_bridge_instance->socket_cli_, fcu_bridge_instance->buffer_, sizeof(fcu_bridge_instance->buffer_), 0);
      }
      if(n > 0) {
          for(uint16_t i = 0; i < n; i++) {
              rbPush(&fcu_bridge_instance->mav_buf_receive_, fcu_bridge_instance->buffer_[i]);
          }
      }
      fcu_bridge_instance->parse_data();
      fcu_bridge_instance->flush_data();
      loop_rate.sleep();
    }
    fcu_bridge_instance->ser_.close();
    close(fcu_bridge_instance->socket_cli_);

    RCLCPP_INFO(fcu_bridge_instance->node->get_logger(),"hello world!");

    rclcpp::shutdown();

    return 0;
}
