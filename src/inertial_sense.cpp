#include "inertial_sense.h"
#include <chrono>
#include <stddef.h>
#include <unistd.h>
#include <tf2/LinearMath/Transform.h>
// #include <ros/console.h>

InertialSenseROS::InertialSenseROS() :
  Node("inertial_sense_node"), 
  initialized_(false)
{
  connect();
  set_navigation_dt_ms();

  /// Start Up ROS service servers
  refLLA_set_current_srv_ = create_service<std_srvs::srv::Trigger>("set_refLLA_current",
                std::bind(&InertialSenseROS::set_current_position_as_refLLA, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  refLLA_set_value_srv_ = create_service<inertial_sense::srv::RefLLAUpdate>("set_refLLA_value",
                std::bind(&InertialSenseROS::set_refLLA_to_value, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  mag_cal_srv_ = create_service<std_srvs::srv::Trigger>("single_axis_mag_cal",
                std::bind(&InertialSenseROS::perform_mag_cal_srv_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  multi_mag_cal_srv_ = create_service<std_srvs::srv::Trigger>("multi_axis_mag_cal",
                std::bind(&InertialSenseROS::perform_multi_mag_cal_srv_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  firmware_update_srv_ = create_service<inertial_sense::srv::FirmwareUpdate>("firmware_update",
                std::bind(&InertialSenseROS::update_firmware_srv_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  br = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  configure_parameters();
  configure_rtk();
  configure_data_streams();

  log_enabled_ = this->declare_parameter("enable_log", false);
  if (log_enabled_)
  {
    start_log();//start log should always happen last, does not all stop all message streams.
  }


//  configure_ascii_output(); //does not work right now

  initialized_ = true;
}


void InertialSenseROS::configure_data_streams()
{
  SET_CALLBACK(DID_GPS1_POS, gps_pos_t, GPS_pos_callback,1); // we always need GPS for Fix status
  SET_CALLBACK(DID_GPS1_VEL, gps_vel_t, GPS_vel_callback,1); // we always need GPS for Fix status
  SET_CALLBACK(DID_STROBE_IN_TIME, strobe_in_time_t, strobe_in_time_callback,1); // we always want the strobe
  

  INS_enabled_ = this->declare_parameter("stream_INS", true);
  if (INS_enabled_)
  {
    INS_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("ins", rclcpp::SensorDataQoS());
    SET_CALLBACK(DID_INS_1, ins_1_t, INS1_callback, 5);
    SET_CALLBACK(DID_INS_2, ins_2_t, INS2_callback, 5);
    SET_CALLBACK(DID_DUAL_IMU, dual_imu_t, IMU_callback, 1);
//    SET_CALLBACK(DID_INL2_VARIANCE, nav_dt_ms, inl2_variance_t, INS_variance_callback);
  }
  publishTf = this->declare_parameter("publishTf", true);
  LTCF = this->declare_parameter("LTCF", (int)NED);
  // Set up the IMU ROS stream

  IMU_enabled_ = this->declare_parameter("stream_IMU", true);
  if (IMU_enabled_)
  {
    IMU_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", rclcpp::SensorDataQoS());
    SET_CALLBACK(DID_INS_1, ins_1_t, INS1_callback, 1);
    SET_CALLBACK(DID_INS_2, ins_2_t, INS2_callback, 1);
    SET_CALLBACK(DID_DUAL_IMU, dual_imu_t, IMU_callback, 1);
  }

  // Set up the GPS ROS stream - we always need GPS information for time sync, just don't always need to publish it
  GPS_enabled_ = this->declare_parameter("stream_GPS", true);
  if (GPS_enabled_)
      GPS_pub_ = this->create_publisher<inertial_sense::msg::GPS>("gps", rclcpp::SensorDataQoS());

  GPS_obs_enabled_ = this->declare_parameter("stream_GPS_raw", false);
  // GPS_eph_enabled_ = this->declare_parameter("stream_GPS_raw", false);
  if (GPS_obs_enabled_)
  {
    GPS_obs_pub_ = this->create_publisher<inertial_sense::msg::GNSSObsVec>("gps/obs", rclcpp::SensorDataQoS());
    GPS_eph_pub_ = this->create_publisher<inertial_sense::msg::GNSSEphemeris>("gps/eph", rclcpp::SensorDataQoS());
    GPS_eph_pub2_ = this->create_publisher<inertial_sense::msg::GlonassEphemeris>("gps/geph", rclcpp::SensorDataQoS());
    SET_CALLBACK(DID_GPS1_RAW, gps_raw_t, GPS_raw_callback,1);
    SET_CALLBACK(DID_GPS_BASE_RAW, gps_raw_t, GPS_raw_callback,1);
    SET_CALLBACK(DID_GPS2_RAW, gps_raw_t, GPS_raw_callback,1);
    obs_bundle_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&InertialSenseROS::GPS_obs_bundle_timer_callback, this));
  }

  // Set up the GPS info ROS stream
  GPS_info_enabled_ = this->declare_parameter("stream_GPS_info", false);
  if (GPS_info_enabled_)
  {
    GPS_info_pub_ = this->create_publisher<inertial_sense::msg::GPSInfo>("gps/info", rclcpp::SensorDataQoS());
    SET_CALLBACK(DID_GPS1_SAT, gps_sat_t, GPS_info_callback,1);
  }

  // Set up the magnetometer ROS stream
  mag_enabled_ = this->declare_parameter("stream_mag", false);
  if (mag_enabled_)
  {
    mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("mag", rclcpp::SensorDataQoS());
    SET_CALLBACK(DID_MAGNETOMETER, magnetometer_t, mag_callback,1);
  }

  // Set up the barometer ROS stream
  baro_enabled_ = this->declare_parameter("stream_baro", false);
  if (baro_enabled_)
  {
    baro_pub_ = this->create_publisher<sensor_msgs::msg::FluidPressure>("baro", rclcpp::SensorDataQoS());
    SET_CALLBACK(DID_BAROMETER, barometer_t, baro_callback,1);
  }

  // Set up the preintegrated IMU (coning and sculling integral) ROS stream
  dt_vel_enabled_ = this->declare_parameter("stream_preint_IMU", false);
  if (dt_vel_enabled_)
  {
    dt_vel_pub_ = this->create_publisher<inertial_sense::msg::PreIntIMU>("preint_imu", rclcpp::SensorDataQoS());
    SET_CALLBACK(DID_PREINTEGRATED_IMU, preintegrated_imu_t, preint_IMU_callback,1);
  }

  // Set up ROS dianostics for rqt_robot_monitor
  diagnostics_enabled_ = this->declare_parameter("stream_diagnostics", true);
  if (diagnostics_enabled_)
  {
    diagnostics_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", rclcpp::SensorDataQoS());
    diagnostics_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&InertialSenseROS::diagnostics_callback, this)); // 2 Hz
  }
}

void InertialSenseROS::start_log()
{
  std::string filename = cISLogger::CreateCurrentTimestamp();
  RCLCPP_INFO(this->get_logger(), "Creating log in " + filename + " folder");
  IS_.SetLoggerEnabled(true, filename, cISLogger::LOGTYPE_DAT, RMC_PRESET_PPD_ROBOT);
}

void InertialSenseROS::configure_ascii_output()
{
  //  int NMEA_rate = nh_private_.param<int>("NMEA_rate", 0);
  //  int NMEA_message_configuration = nh_private_.param<int>("NMEA_configuration", 0x00);
  //  int NMEA_message_ports = nh_private_.param<int>("NMEA_ports", 0x00);
  //  ascii_msgs_t msgs = {};
  //  msgs.options = (NMEA_message_ports & NMEA_SER0) ? RMC_OPTIONS_PORT_SER0 : 0; // output on serial 0
  //  msgs.options |= (NMEA_message_ports & NMEA_SER1) ? RMC_OPTIONS_PORT_SER1 : 0; // output on serial 1
  //  msgs.gpgga = (NMEA_message_configuration & NMEA_GPGGA) ? NMEA_rate : 0;
  //  msgs.gpgll = (NMEA_message_configuration & NMEA_GPGLL) ? NMEA_rate : 0;
  //  msgs.gpgsa = (NMEA_message_configuration & NMEA_GPGSA) ? NMEA_rate : 0;
  //  msgs.gprmc = (NMEA_message_configuration & NMEA_GPRMC) ? NMEA_rate : 0;
  //  IS_.SendData(DID_ASCII_BCAST_PERIOD, (uint8_t*)(&msgs), sizeof(ascii_msgs_t), 0);

}

void InertialSenseROS::connect()
{

  port_ = this->declare_parameter("port", "/dev/ttyUSB0");
  baudrate_ = this->declare_parameter("baudrate", 921600);
  frame_id_ = this->declare_parameter("frame_id", "body");

  /// Connect to the uINS
  RCLCPP_INFO(this->get_logger(), "Connecting to serial port \"%s\", at %d baud", port_.c_str(), baudrate_);
  if (! IS_.Open(port_.c_str(), baudrate_))
  {
    RCLCPP_FATAL(this->get_logger(), "inertialsense: Unable to open serial port \"%s\", at %d baud", port_.c_str(), baudrate_);
    exit(0);
  }
  else
  {
    // Print if Successful
    RCLCPP_INFO(this->get_logger(), "Connected to uINS %d on \"%s\", at %d baud", IS_.GetDeviceInfo().serialNumber, port_.c_str(), baudrate_);
  }
}

void InertialSenseROS::set_navigation_dt_ms()
{
  // Make sure the navigation rate is right, if it's not, then we need to change and reset it.
  int nav_dt_ms = IS_.GetFlashConfig().startupNavDtMs;
  if (this->get_parameter("navigation_dt_ms", nav_dt_ms))
  {
    if (nav_dt_ms != IS_.GetFlashConfig().startupNavDtMs)
    {
      uint32_t data = nav_dt_ms;
      IS_.SendData(DID_FLASH_CONFIG, (uint8_t*)(&data), sizeof(uint32_t), offsetof(nvm_flash_cfg_t, startupNavDtMs));
      RCLCPP_INFO(this->get_logger(), "navigation rate change from %dms to %dms, resetting uINS to make change", IS_.GetFlashConfig().startupNavDtMs, nav_dt_ms);
      sleep(3);
      reset_device();
    }
  }
}

void InertialSenseROS::configure_parameters()
{
  set_vector_flash_config<float>("INS_rpy_radians", 3, offsetof(nvm_flash_cfg_t, insRotation));
  set_vector_flash_config<float>("INS_xyz", 3, offsetof(nvm_flash_cfg_t, insOffset));
  set_vector_flash_config<float>("GPS_ant1_xyz", 3, offsetof(nvm_flash_cfg_t, gps1AntOffset));
  set_vector_flash_config<float>("GPS_ant2_xyz", 3, offsetof(nvm_flash_cfg_t, gps2AntOffset));
  set_vector_flash_config<double>("GPS_ref_lla", 3, offsetof(nvm_flash_cfg_t, refLla));

  set_flash_config<float>("inclination", offsetof(nvm_flash_cfg_t, magInclination), 0.0f);
  set_flash_config<float>("declination", offsetof(nvm_flash_cfg_t, magDeclination), 0.0f);
  set_flash_config<int>("dynamic_model", offsetof(nvm_flash_cfg_t, insDynModel), 8);
  set_flash_config<int>("ser1_baud_rate", offsetof(nvm_flash_cfg_t, ser1BaudRate), 921600);
}

void InertialSenseROS::configure_rtk()
{
  bool RTK_rover, RTK_base, dual_GNSS;
  RTK_rover = this->declare_parameter("RTK_rover", false);
  RTK_base = this->declare_parameter("RTK_base", false);
  dual_GNSS = this->declare_parameter("dual_GNSS", false);
  std::string RTK_server_IP, RTK_correction_type;
  int RTK_server_port;
  RTK_server_IP = this->declare_parameter("RTK_server_IP", "127.0.0.1");
  RTK_server_port = this->declare_parameter("RTK_server_port", 7777);
  RTK_correction_type = this->declare_parameter("RTK_correction_type", "UBLOX");
  if (RTK_rover && RTK_base)
    RCLCPP_ERROR(this->get_logger(), "unable to configure uINS to be both RTK rover and base - default to rover");
  if (RTK_rover && dual_GNSS)
    RCLCPP_ERROR(this->get_logger(), "unable to configure uINS to be both RTK rover as dual GNSS - default to dual GNSS");

  uint32_t RTKCfgBits = 0;
  if (dual_GNSS)
  {
    RTK_rover = false;
    RCLCPP_INFO(this->get_logger(), "InertialSense: Configured as dual GNSS (compassing)");
    RTK_state_ = DUAL_GNSS;
    RTKCfgBits |= RTK_CFG_BITS_COMPASSING;
    SET_CALLBACK(DID_GPS1_RTK_MISC, gps_rtk_misc_t, RTK_Misc_callback,1);
    SET_CALLBACK(DID_GPS1_RTK_REL, gps_rtk_rel_t, RTK_Rel_callback,1);
    RTK_enabled_ = true;
    RTK_pub_ = this->create_publisher<inertial_sense::msg::RTKInfo>("RTK/info", rclcpp::SensorDataQoS());
    RTK_pub2_ = this->create_publisher<inertial_sense::msg::RTKRel>("RTK/rel", rclcpp::SensorDataQoS());
  }

  if (RTK_rover)
  {
    RTK_base = false;
    std::string RTK_connection =  RTK_correction_type + ":" + RTK_server_IP + ":" + std::to_string(RTK_server_port);
    RCLCPP_INFO(this->get_logger(), "InertialSense: Configured as RTK Rover");
    RTK_state_ = RTK_ROVER;
    RTKCfgBits |= RTK_CFG_BITS_GPS1_RTK_ROVER;

    if (IS_.OpenServerConnection(RTK_connection))
      RCLCPP_INFO(this->get_logger(), "Successfully connected to " + RTK_connection + " RTK server");
    else
      RCLCPP_ERROR(this->get_logger(), "Failed to connect to base server at " + RTK_connection);

    SET_CALLBACK(DID_GPS1_RTK_MISC, gps_rtk_misc_t, RTK_Misc_callback,1);
    SET_CALLBACK(DID_GPS1_RTK_REL, gps_rtk_rel_t, RTK_Rel_callback,1);
    RTK_enabled_ = true;
    RTK_pub_ = this->create_publisher<inertial_sense::msg::RTKInfo>("RTK/info", rclcpp::SensorDataQoS());
    RTK_pub2_ = this->create_publisher<inertial_sense::msg::RTKRel>("RTK/rel", rclcpp::SensorDataQoS());
  }

  else if (RTK_base)
  {
    std::string RTK_connection =  RTK_server_IP + ":" + std::to_string(RTK_server_port);
    RTK_enabled_ = true;
    RCLCPP_INFO(this->get_logger(), "InertialSense: Configured as RTK Base");
    RTK_state_ = RTK_BASE;
    RTKCfgBits |= RTK_CFG_BITS_BASE_OUTPUT_GPS1_UBLOX_SER0;

    if (IS_.CreateHost(RTK_connection))
    {
      RCLCPP_INFO(this->get_logger(), "Successfully created " + RTK_connection + " as RTK server");
      initialized_ = true;
      return;
    }
    else
      RCLCPP_ERROR(this->get_logger(), "Failed to create base server at " + RTK_connection);
  }
  IS_.SendData(DID_FLASH_CONFIG, reinterpret_cast<uint8_t*>(&RTKCfgBits), sizeof(RTKCfgBits), offsetof(nvm_flash_cfg_t, RTKCfgBits));
}

template <typename T>
void InertialSenseROS::set_vector_flash_config(std::string param_name, uint32_t size, uint32_t offset){
  std::vector<double> tmp(size,0);
  T v[size];

  rclcpp::Parameter param_list(param_name, tmp);

  this->declare_parameter(param_name);
  this->get_parameter(param_name, param_list);
  tmp = param_list.as_double_array();

  std::cout << param_name << ": ";
  for (int i = 0; i < size; i++)
  {
    v[i] = tmp[i];
    std::cout << v[i] << " ";
  }
  std::cout << std::endl;
  
  IS_.SendData(DID_FLASH_CONFIG, reinterpret_cast<uint8_t*>(&v), sizeof(v), offset);
  IS_.GetFlashConfig() = IS_.GetFlashConfig();
}

template <typename T>
void InertialSenseROS::set_flash_config(std::string param_name, uint32_t offset, T def)
{
  T tmp;
  // nh_private_.param<T>(param_name, tmp, def);
  this->get_parameter_or(param_name, tmp, def);
  IS_.SendData(DID_FLASH_CONFIG, reinterpret_cast<uint8_t*>(&tmp), sizeof(T), offset);
}

void InertialSenseROS::INS1_callback(const ins_1_t * const msg)
{
  odom_msg.header.frame_id = frame_id_;
  if (LTCF == NED)
  {
    odom_msg.pose.pose.position.x = msg->ned[0];
    odom_msg.pose.pose.position.y = msg->ned[1];
    odom_msg.pose.pose.position.z = msg->ned[2];
  }
  else if (LTCF == ENU)
  {
    odom_msg.pose.pose.position.x = msg->ned[1];
    odom_msg.pose.pose.position.y = msg->ned[0];
    odom_msg.pose.pose.position.z = -msg->ned[2];
  }

}

//void InertialSenseROS::INS_variance_callback(const inl2_variance_t * const msg)
//{
//  // We have to convert NED velocity covariance into body-fixed
//  tf::Matrix3x3 cov_vel_NED;
//  cov_vel_NED.setValue(msg->PvelNED[0], 0, 0, 0, msg->PvelNED[1], 0, 0, 0, msg->PvelNED[2]);
//  tf::Quaternion att;
//  tf::quaternionMsgToTF(odom_msg.pose.pose.orientation, att);
//  tf::Matrix3x3 R_NED_B(att);
//  tf::Matrix3x3 cov_vel_B = R_NED_B.transposeTimes(cov_vel_NED * R_NED_B);

//  // Populate Covariance Matrix
//  for (int i = 0; i < 3; i++)
//  {
//    // Position and velocity covariance is only valid if in NAV mode (with GPS)
//    if (insStatus_ & INS_STATUS_NAV_MODE)
//    {
//      odom_msg.pose.covariance[7*i] = msg->PxyzNED[i];
//      for (int j = 0; j < 3; j++)
//        odom_msg.twist.covariance[6*i+j] = cov_vel_B[i][j];
//    }
//    else
//    {
//      odom_msg.pose.covariance[7*i] = 0;
//      odom_msg.twist.covariance[7*i] = 0;
//    }
//    odom_msg.pose.covariance[7*(i+3)] = msg->PattNED[i];
//    odom_msg.twist.covariance[7*(i+3)] = msg->PWBias[i];
//  }
//}


void InertialSenseROS::INS2_callback(const ins_2_t * const msg)
{
  odom_msg.header.stamp = ros_time_from_week_and_tow(msg->week, msg->timeOfWeek);
  odom_msg.header.frame_id = frame_id_;

  odom_msg.pose.pose.orientation.w = msg->qn2b[0];
  if (LTCF == NED)
  {
    odom_msg.pose.pose.orientation.x = msg->qn2b[1];
    odom_msg.pose.pose.orientation.y = msg->qn2b[2];
    odom_msg.pose.pose.orientation.z = msg->qn2b[3];
  }
  else if (LTCF == ENU)
  {
    odom_msg.pose.pose.orientation.x = msg->qn2b[2];
    odom_msg.pose.pose.orientation.y = msg->qn2b[1];
    odom_msg.pose.pose.orientation.z = -msg->qn2b[3];
  }

  odom_msg.twist.twist.linear.x = msg->uvw[0];
  odom_msg.twist.twist.linear.y = msg->uvw[1];
  odom_msg.twist.twist.linear.z = msg->uvw[2];

  lla_[0] = msg->lla[0];
  lla_[1] = msg->lla[1];
  lla_[2] = msg->lla[2];

  odom_msg.pose.covariance[0] = lla_[0];
  odom_msg.pose.covariance[1] = lla_[1];
  odom_msg.pose.covariance[2] = lla_[2];
  odom_msg.pose.covariance[3] = ecef_[0];
  odom_msg.pose.covariance[4] = ecef_[1];
  odom_msg.pose.covariance[5] = ecef_[2];
  odom_msg.pose.covariance[6] = LTCF;         //Defined in inertial_sense.h: enum InertialSenseROS::ltcf

  odom_msg.twist.twist.angular.x = imu1_msg.angular_velocity.x;
  odom_msg.twist.twist.angular.y = imu1_msg.angular_velocity.y;
  odom_msg.twist.twist.angular.z = imu1_msg.angular_velocity.z;

  if (publishTf)
  {    
    transform.header.stamp = this->now();// + rclcpp::Duration(0, 10000000);
    transform.header.frame_id = "ins";
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = odom_msg.pose.pose.position.x;
    transform.transform.translation.y = odom_msg.pose.pose.position.y;
    transform.transform.translation.z = odom_msg.pose.pose.position.z;
    transform.transform.rotation.x = odom_msg.pose.pose.orientation.x;
    transform.transform.rotation.y = odom_msg.pose.pose.orientation.y;
    transform.transform.rotation.z = odom_msg.pose.pose.orientation.z;
    transform.transform.rotation.w = odom_msg.pose.pose.orientation.w;

    br->sendTransform(transform);

    // Calculate the TF from the pose...
		// transform.setOrigin(tf::Vector3(odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z));
    // tf::Quaternion q;
    // tf::quaternionMsgToTF(odom_msg.pose.pose.orientation, q);
    // transform.setRotation(q);

    // br->sendTransform(tf::StampedTransform(transform, this->now(), "ins", "base_link"));
  }

  if (INS_enabled_)
    INS_pub_->publish(odom_msg);
}


void InertialSenseROS::IMU_callback(const dual_imu_t* const msg)
{
  imu1_msg.header.stamp = imu2_msg.header.stamp = ros_time_from_start_time(msg->time);
  imu1_msg.header.frame_id = imu2_msg.header.frame_id = frame_id_;

  imu1_msg.angular_velocity.x = msg->I[0].pqr[0];
  imu1_msg.angular_velocity.y = msg->I[0].pqr[1];
  imu1_msg.angular_velocity.z = msg->I[0].pqr[2];
  imu1_msg.linear_acceleration.x = msg->I[0].acc[0];
  imu1_msg.linear_acceleration.y = msg->I[0].acc[1];
  imu1_msg.linear_acceleration.z = msg->I[0].acc[2];

  //  imu2_msg.angular_velocity.x = msg->I[1].pqr[0];
  //  imu2_msg.angular_velocity.y = msg->I[1].pqr[1];
  //  imu2_msg.angular_velocity.z = msg->I[1].pqr[2];
  //  imu2_msg.linear_acceleration.x = msg->I[1].acc[0];
  //  imu2_msg.linear_acceleration.y = msg->I[1].acc[1];
  //  imu2_msg.linear_acceleration.z = msg->I[1].acc[2];

  if (IMU_enabled_)
  {
    IMU_pub_->publish(imu1_msg);
    //    IMU_pub2_->publish(imu2_msg);
  }
}


void InertialSenseROS::GPS_pos_callback(const gps_pos_t * const msg)
{
  GPS_week_ = msg->week;
  GPS_towOffset_ = msg->towOffset;
  if (GPS_enabled_)
  {
    gps_msg.header.stamp = ros_time_from_week_and_tow(msg->week, msg->timeOfWeekMs/1e3);
    gps_msg.fix_type = msg->status & GPS_STATUS_FIX_MASK;
    gps_msg.header.frame_id =frame_id_;
    gps_msg.num_sat = (uint8_t)(msg->status & GPS_STATUS_NUM_SATS_USED_MASK);
    gps_msg.cno = msg->cnoMean;
    gps_msg.latitude = msg->lla[0];
    gps_msg.longitude = msg->lla[1];
    gps_msg.altitude = msg->lla[2];
    gps_msg.pos_ecef.x = ecef_[0] = msg->ecef[0];
    gps_msg.pos_ecef.y = ecef_[1] = msg->ecef[1];
    gps_msg.pos_ecef.z = ecef_[2] = msg->ecef[2];
    gps_msg.hmsl = msg->hMSL;
    gps_msg.hacc = msg->hAcc;
    gps_msg.vacc = msg->vAcc;
    gps_msg.pdop = msg->pDop;
    publishGPS();
  }
}

void InertialSenseROS::GPS_vel_callback(const gps_vel_t * const msg)
{
	if (GPS_enabled_)
	{
		gps_velEcef.header.stamp = ros_time_from_week_and_tow(GPS_week_, msg->timeOfWeekMs/1e3);
		gps_velEcef.vector.x = msg->velEcef[0];
		gps_velEcef.vector.y = msg->velEcef[1];
		gps_velEcef.vector.z = msg->velEcef[2];
		publishGPS();
	}
}

void InertialSenseROS::publishGPS()
{
  auto time_differance = (double)gps_velEcef.header.stamp.sec + gps_velEcef.header.stamp.nanosec*1e-9  
                       - ( (double)gps_msg.header.stamp.sec + gps_msg.header.stamp.nanosec*1e-9  );
  if (time_differance < 2e-3)
	{
		gps_msg.vel_ecef = gps_velEcef.vector;
		GPS_pub_->publish(gps_msg);
	}
}

void InertialSenseROS::update()
{
	IS_.Update();
}

void InertialSenseROS::strobe_in_time_callback(const strobe_in_time_t * const msg)
{
  // create the subscriber if it doesn't exist
  if ( std::strlen( strobe_pub_->get_topic_name() ) == 0 )
    strobe_pub_ = this->create_publisher<std_msgs::msg::Header>("strobe_time", rclcpp::SensorDataQoS());
  
  std_msgs::msg::Header strobe_msg;
  strobe_msg.stamp = ros_time_from_week_and_tow(msg->week, msg->timeOfWeekMs * 1e-3);
  strobe_pub_->publish(strobe_msg);
}


void InertialSenseROS::GPS_info_callback(const gps_sat_t* const msg)
{
  gps_info_msg.header.stamp = ros_time_from_tow(msg->timeOfWeekMs/1e3);
  gps_info_msg.header.frame_id = frame_id_;
  gps_info_msg.num_sats = msg->numSats;
  for (int i = 0; i < 50; i++)
  {
    gps_info_msg.sattelite_info[i].sat_id = msg->sat[i].svId;
    gps_info_msg.sattelite_info[i].cno = msg->sat[i].cno;
  }
  GPS_info_pub_->publish(gps_info_msg);
}


void InertialSenseROS::mag_callback(const magnetometer_t* const msg)
{
  sensor_msgs::msg::MagneticField mag_msg;
  mag_msg.header.stamp = ros_time_from_start_time(msg->time);
  mag_msg.header.frame_id = frame_id_;
  mag_msg.magnetic_field.x = msg->mag[0];
  mag_msg.magnetic_field.y = msg->mag[1];
  mag_msg.magnetic_field.z = msg->mag[2];

  mag_pub_->publish(mag_msg);
}

void InertialSenseROS::baro_callback(const barometer_t * const msg)
{
  sensor_msgs::msg::FluidPressure baro_msg;
  baro_msg.header.stamp = ros_time_from_start_time(msg->time);
  baro_msg.header.frame_id = frame_id_;
  baro_msg.fluid_pressure = msg->bar;
  baro_msg.variance = msg-> barTemp;

  baro_pub_->publish(baro_msg);
}

void InertialSenseROS::preint_IMU_callback(const preintegrated_imu_t * const msg)
{
  inertial_sense::msg::PreIntIMU preintIMU_msg;
  preintIMU_msg.header.stamp = ros_time_from_start_time(msg->time);
  preintIMU_msg.header.frame_id = frame_id_;
  preintIMU_msg.dtheta.x = msg->theta1[0];
  preintIMU_msg.dtheta.y = msg->theta1[1];
  preintIMU_msg.dtheta.z = msg->theta1[2];

  preintIMU_msg.dvel.x = msg->vel1[0];
  preintIMU_msg.dvel.y = msg->vel1[1];
  preintIMU_msg.dvel.z = msg->vel1[2];

  preintIMU_msg.dt = msg->dt;

  dt_vel_pub_->publish(preintIMU_msg);
}

void InertialSenseROS::RTK_Misc_callback(const gps_rtk_misc_t* const msg)
{
  if (RTK_enabled_)
  {
    inertial_sense::msg::RTKInfo rtk_info;
    rtk_info.header.stamp = ros_time_from_week_and_tow(GPS_week_, msg->timeOfWeekMs/1000.0);
    rtk_info.base_antcount = msg->baseAntennaCount;
    rtk_info.base_eph = msg->baseBeidouEphemerisCount + msg->baseGalileoEphemerisCount + msg->baseGlonassEphemerisCount
                       + msg->baseGpsEphemerisCount;
    rtk_info.base_obs = msg->baseBeidouObservationCount + msg->baseGalileoObservationCount + msg->baseGlonassObservationCount
                       + msg->baseGpsObservationCount;
    rtk_info.base_lla[0] = msg->baseLla[0];
    rtk_info.base_lla[1] = msg->baseLla[1];
    rtk_info.base_lla[2] = msg->baseLla[2];

    rtk_info.rover_eph = msg->roverBeidouEphemerisCount + msg->roverGalileoEphemerisCount + msg->roverGlonassEphemerisCount
                        + msg->roverGpsEphemerisCount;
    rtk_info.rover_obs = msg->roverBeidouObservationCount + msg->roverGalileoObservationCount + msg->roverGlonassObservationCount
                        + msg->roverGpsObservationCount;
    rtk_info.cycle_slip_count = msg->cycleSlipCount;
    RTK_pub_->publish(rtk_info);
  }
}


void InertialSenseROS::RTK_Rel_callback(const gps_rtk_rel_t* const msg)
{
  if (RTK_enabled_)
  {
    inertial_sense::msg::RTKRel rtk_rel;
    rtk_rel.header.stamp = ros_time_from_week_and_tow(GPS_week_, msg->timeOfWeekMs/1000.0);
    rtk_rel.differential_age = msg->differentialAge;
    rtk_rel.ar_ratio = msg->arRatio;
    rtk_rel.vector_base_to_rover.x = msg->baseToRoverVector[0];
    rtk_rel.vector_base_to_rover.y = msg->baseToRoverVector[1];
    rtk_rel.vector_base_to_rover.z = msg->baseToRoverVector[2];
    rtk_rel.distance_base_to_rover = msg->baseToRoverDistance;
    rtk_rel.heading_base_to_rover = msg->baseToRoverHeading;
    RTK_pub2_->publish(rtk_rel);

    // save for diagnostics
    diagnostic_ar_ratio_ = rtk_rel.ar_ratio;
    diagnostic_differential_age_ = rtk_rel.differential_age;
    diagnostic_heading_base_to_rover_ = rtk_rel.heading_base_to_rover;
  }
}

void InertialSenseROS::GPS_raw_callback(const gps_raw_t * const msg)
{
  switch(msg->dataType)
  {
  case raw_data_type_observation:
    GPS_obs_callback((obsd_t*)&msg->data.obs, msg->obsCount);
    break;

  case raw_data_type_ephemeris:
    GPS_eph_callback((eph_t*)&msg->data.eph);
    break;

  case raw_data_type_glonass_ephemeris:
    GPS_geph_callback((geph_t*)&msg->data.gloEph);
    break;

  default:
    break;
  }
}

void InertialSenseROS::GPS_obs_callback(const obsd_t * const msg, int nObs)
{
  if (obs_Vec_.obs.size() > 0 &&
       (msg[0].time.time != obs_Vec_.obs[0].time.time ||
        msg[0].time.sec != obs_Vec_.obs[0].time.sec))
      GPS_obs_bundle_timer_callback();

  for (int i = 0; i < nObs; i++)
  {
      inertial_sense::msg::GNSSObservation obs;
      obs.header.stamp = ros_time_from_gtime(msg[i].time.time, msg[i].time.sec);
      obs.time.time = msg[i].time.time;
      obs.time.sec = msg[i].time.sec;
      obs.sat = msg[i].sat;
      obs.rcv = msg[i].rcv;
      obs.snr = msg[i].SNR[0];
      obs.lli = msg[i].LLI[0];
      obs.code = msg[i].code[0];
      obs.qual_l = msg[i].qualL[0];
      obs.qual_p = msg[i].qualP[0];
      obs.l = msg[i].L[0];
      obs.p = msg[i].P[0];
      obs.d = msg[i].D[0];
      obs_Vec_.obs.push_back(obs);
      last_obs_time_ = this->now();
  }
}

void InertialSenseROS::GPS_obs_bundle_timer_callback()
{
    if (obs_Vec_.obs.size() == 0)
        return;

    if ((this->now() - last_obs_time_).seconds() > 1e-2)
    {
        obs_Vec_.header.stamp = ros_time_from_gtime(obs_Vec_.obs[0].time.time, obs_Vec_.obs[0].time.sec);
        obs_Vec_.time = obs_Vec_.obs[0].time;
        GPS_obs_pub_->publish(obs_Vec_);
        obs_Vec_.obs.clear();
//        cout << "dt" << (obs_Vec_.header.stamp - this->now()) << endl;
    }
}


void InertialSenseROS::GPS_eph_callback(const eph_t * const msg)
{
  inertial_sense::msg::GNSSEphemeris eph;
  eph.sat = msg->sat;
  eph.iode = msg->iode;
  eph.iodc = msg->iodc;
  eph.sva = msg->sva;
  eph.svh = msg->svh;
  eph.week = msg->week;
  eph.code = msg->code;
  eph.flag = msg->flag;
  eph.toe.time = msg->toe.time;
  eph.toc.time = msg->toc.time;
  eph.ttr.time = msg->ttr.time;
  eph.toe.sec = msg->toe.sec;
  eph.toc.sec = msg->toc.sec;
  eph.ttr.sec = msg->ttr.sec;
  eph.a = msg->A;
  eph.e = msg->e;
  eph.i0 = msg->i0;
  eph.omg0 = msg->OMG0;
  eph.omg = msg->omg;
  eph.m0 = msg->M0;
  eph.deln = msg->deln;
  eph.omgd = msg->OMGd;
  eph.idot = msg->idot;
  eph.crc = msg->crc;
  eph.crs = msg->crs;
  eph.cuc = msg->cuc;
  eph.cus = msg->cus;
  eph.cic = msg->cic;
  eph.cis = msg->cis;
  eph.toes = msg->toes;
  eph.fit = msg->fit;
  eph.f0 = msg->f0;
  eph.f1 = msg->f1;
  eph.f2 = msg->f2;
  eph.tgd[0] = msg->tgd[0];
  eph.tgd[1] = msg->tgd[1];
  eph.tgd[2] = msg->tgd[2];
  eph.tgd[3] = msg->tgd[3];
  eph.adot = msg->Adot;
  eph.ndot = msg->ndot;
  GPS_eph_pub_->publish(eph);
}

void InertialSenseROS::GPS_geph_callback(const geph_t * const msg)
{
  inertial_sense::msg::GlonassEphemeris geph;
  geph.sat = msg->sat;
  geph.iode = msg->iode;
  geph.frq = msg->frq;
  geph.svh = msg->svh;
  geph.sva = msg->sva;
  geph.age = msg->age;
  geph.toe.time = msg->toe.time;
  geph.tof.time = msg->tof.time;
  geph.toe.sec = msg->toe.sec;
  geph.tof.sec = msg->tof.sec;
  geph.pos[0] = msg->pos[0];
  geph.pos[1] = msg->pos[1];
  geph.pos[2] = msg->pos[2];
  geph.vel[0] = msg->vel[0];
  geph.vel[1] = msg->vel[1];
  geph.vel[2] = msg->vel[2];
  geph.acc[0] = msg->acc[0];
  geph.acc[1] = msg->acc[1];
  geph.acc[2] = msg->acc[2];
  geph.taun = msg->taun;
  geph.gamn = msg->gamn;
  geph.dtaun = msg->dtaun;
  GPS_eph_pub2_->publish(geph);
}

void InertialSenseROS::diagnostics_callback()
{
  // Create diagnostic objects
  diagnostic_msgs::msg::DiagnosticArray diag_array;
  diag_array.header.stamp = this->now();

  // CNO mean
  diagnostic_msgs::msg::DiagnosticStatus cno_mean;
  cno_mean.name = "CNO Mean";
  cno_mean.level =  diagnostic_msgs::msg::DiagnosticStatus::OK;
  cno_mean.message = std::to_string(gps_msg.cno);
  diag_array.status.push_back(cno_mean);

  if (RTK_enabled_){
    diagnostic_msgs::msg::DiagnosticStatus rtk_status;
    rtk_status.name = "RTK";
    rtk_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string rtk_message;

    // AR ratio
    diagnostic_msgs::msg::KeyValue ar_ratio;
    ar_ratio.key = "AR Ratio";
    ar_ratio.value = std::to_string(diagnostic_ar_ratio_);
    rtk_status.values.push_back(ar_ratio);
    if (diagnostic_ar_ratio_ < 3.0){
      rtk_status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      rtk_message = "Float: " + std::to_string(diagnostic_ar_ratio_);
    } else if (diagnostic_ar_ratio_ < 6.0){
      rtk_message = "Fix: " + std::to_string(diagnostic_ar_ratio_);
    } else {
      rtk_message = "Fix and Hold: " + std::to_string(diagnostic_ar_ratio_);
    }

    // Differential age
    diagnostic_msgs::msg::KeyValue differential_age;
    differential_age.key = "Differential Age";
    differential_age.value = std::to_string(diagnostic_differential_age_);
    rtk_status.values.push_back(differential_age);
    if (diagnostic_differential_age_ > 1.5){
      rtk_status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      rtk_message += " Differential Age Large";
    }

    // Heading base to rover
    diagnostic_msgs::msg::KeyValue heading_base_to_rover;
    heading_base_to_rover.key = "Heading Base to Rover (rad)";
    heading_base_to_rover.value = std::to_string(diagnostic_heading_base_to_rover_);
    rtk_status.values.push_back(heading_base_to_rover);
    
    rtk_status.message = rtk_message;
    diag_array.status.push_back(rtk_status);
  }

  diagnostics_pub_->publish(diag_array);
}

void InertialSenseROS::set_current_position_as_refLLA(const std::shared_ptr<rmw_request_id_t> request_header,
              const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
              std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  (void)req;
  double current_lla_[3];
  current_lla_[0] = lla_[0];
  current_lla_[1] = lla_[1];
  current_lla_[2] = lla_[2];

  IS_.SendData(DID_FLASH_CONFIG, reinterpret_cast<uint8_t*>(&current_lla_), sizeof(current_lla_), offsetof(nvm_flash_cfg_t, refLla));

  comManagerGetData(0, DID_FLASH_CONFIG, 0, 0, 1);

  int i = 0;
  nvm_flash_cfg_t current_flash = IS_.GetFlashConfig();
  while (current_flash.refLla[0] == IS_.GetFlashConfig().refLla[0] && current_flash.refLla[1] == IS_.GetFlashConfig().refLla[1] && current_flash.refLla[2] == IS_.GetFlashConfig().refLla[2])
  {
      comManagerStep();
      i++;
      if(i>100){break;}
  }

  if(current_lla_[0] == IS_.GetFlashConfig().refLla[0] && current_lla_[1] == IS_.GetFlashConfig().refLla[1] && current_lla_[2] == IS_.GetFlashConfig().refLla[2])
  {
      comManagerGetData(0, DID_FLASH_CONFIG, 0, 0, 0);
      res->success = true;
      res->message = ("Update was succesful.  refLla: Lat: " + to_string(current_lla_[0]) + "  Lon: " +to_string( current_lla_[1]) + "  Alt: " + to_string(current_lla_[2]));
  }
  else
  {
      comManagerGetData(0, DID_FLASH_CONFIG, 0, 0, 0);
      res->success = false;
      res->message = "Unable to update refLLA. Please try again.";

  }
    
}

void InertialSenseROS::set_refLLA_to_value(const std::shared_ptr<rmw_request_id_t> request_header,
              const std::shared_ptr<inertial_sense::srv::RefLLAUpdate::Request> req,
              std::shared_ptr<inertial_sense::srv::RefLLAUpdate::Response> res)
{
  IS_.SendData(DID_FLASH_CONFIG, reinterpret_cast<uint8_t*>(&req->lla), sizeof(req->lla), offsetof(nvm_flash_cfg_t, refLla));

  comManagerGetData(0, DID_FLASH_CONFIG, 0, 0, 1);

  int i = 0;
  nvm_flash_cfg_t current_flash = IS_.GetFlashConfig();
  while (current_flash.refLla[0] == IS_.GetFlashConfig().refLla[0] && current_flash.refLla[1] == IS_.GetFlashConfig().refLla[1] && current_flash.refLla[2] == IS_.GetFlashConfig().refLla[2])
  {
      comManagerStep();
      i++;
      if(i>100){break;}
  }

  if(req->lla[0] == IS_.GetFlashConfig().refLla[0] && req->lla[1] == IS_.GetFlashConfig().refLla[1] && req->lla[2] == IS_.GetFlashConfig().refLla[2])
  {
      comManagerGetData(0, DID_FLASH_CONFIG, 0, 0, 0);
      res->success = true;
      res->message = ("Update was succesful.  refLla: Lat: " + to_string(req->lla[0]) + "  Lon: " +to_string( req->lla[1]) + "  Alt: " + to_string(req->lla[2]));
  }
  else
  {
      comManagerGetData(0, DID_FLASH_CONFIG, 0, 0, 0);
      res->success = false;
      res->message = "Unable to update refLLA. Please try again.";
  }
}

void InertialSenseROS::perform_mag_cal_srv_callback(const std::shared_ptr<rmw_request_id_t> request_header,
              const std::shared_ptr<std_srvs::srv::Trigger::Request> req, 
              std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  (void)req;
    uint32_t single_axis_command = 2;
    IS_.SendData(DID_MAG_CAL, reinterpret_cast<uint8_t*>(&single_axis_command), sizeof(uint32_t), offsetof(mag_cal_t, recalCmd));


    is_comm_instance_t comm;
    uint8_t buffer[2048];
    comm.buffer = buffer;
    comm.bufferSize = sizeof(buffer);

    is_comm_init(&comm);
    uint8_t inByte;
    serial_port_t* serialPort = IS_.GetSerialPort();
    int count;
    while ((count = serialPortReadCharTimeout(serialPort, &inByte, 20)) > 0)
    {
        switch (is_comm_parse(&comm, inByte))
        {
        case DID_INS_1:
            ins_1_t* msg = (ins_1_t*)buffer;
            if (msg->insStatus & 0x00400000)
            {
                res->success = true;
                res->message = "Successfully initiated mag recalibration.";
                return;
            }

            break;
        }
    }
}

void InertialSenseROS::perform_multi_mag_cal_srv_callback(const std::shared_ptr<rmw_request_id_t> request_header,
              const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
              std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  (void)req;
  uint32_t multi_axis_command = 1;
  IS_.SendData(DID_MAG_CAL, reinterpret_cast<uint8_t*>(&multi_axis_command), sizeof(uint32_t), offsetof(mag_cal_t, recalCmd));

  is_comm_instance_t comm;
  uint8_t buffer[2048];
  comm.buffer = buffer;
  comm.bufferSize = sizeof(buffer);

  is_comm_init(&comm);
  uint8_t inByte;
  serial_port_t* serialPort = IS_.GetSerialPort();
  int count;
  while ((count = serialPortReadCharTimeout(serialPort, &inByte, 20)) > 0)
  {
      switch (is_comm_parse(&comm, inByte))
      {
      case DID_INS_1:
          ins_1_t* msg = (ins_1_t*)buffer;
          if (msg->insStatus & 0x00400000)
          {
              res->success = true;
              res->message = "Successfully initiated mag recalibration";
              return;
          }

          break;
      }
  }
}

void InertialSenseROS::reset_device()
{
  // send reset command
  system_command_t reset_command;
  reset_command.command = 99;
  reset_command.invCommand = ~reset_command.command;
  IS_.SendData(DID_SYS_CMD, reinterpret_cast<uint8_t*>(&reset_command), sizeof(system_command_t), 0);
  sleep(1);
}

void InertialSenseROS::update_firmware_srv_callback(const std::shared_ptr<rmw_request_id_t> request_header,
              const std::shared_ptr<inertial_sense::srv::FirmwareUpdate::Request> req,  
              std::shared_ptr<inertial_sense::srv::FirmwareUpdate::Response> res)
{
  IS_.Close();
  vector<InertialSense::bootloader_result_t> results = IS_.BootloadFile("*", req->filename, 921600);
  if (!results[0].error.empty())
  {
    res->success = false;
    res->message = results[0].error;
    return;
  }
  IS_.Open(port_.c_str(), baudrate_);
  return;
}


rclcpp::Time InertialSenseROS::ros_time_from_week_and_tow(const uint32_t week, const double timeOfWeek)
{
  rclcpp::Time rostime(0, 0);
  //  If we have a GPS fix, then use it to set timestamp
  if (GPS_towOffset_)
  {
    uint64_t sec = UNIX_TO_GPS_OFFSET + floor(timeOfWeek) + week*7*24*3600;
    uint64_t nsec = (timeOfWeek - floor(timeOfWeek))*1e9;
    rostime = rclcpp::Time(sec, nsec);
  }
  else
  {
    // Otherwise, estimate the uINS boot time and offset the messages
    if (!got_first_message_)
    {
      got_first_message_ = true;
      INS_local_offset_ = this->now().seconds() - timeOfWeek;
    }
    else // low-pass filter offset to account for drift
    {
      double y_offset = this->now().seconds() - timeOfWeek;
      INS_local_offset_ = 0.005 * y_offset + 0.995 * INS_local_offset_;
    }
    // Publish with ROS time
    rostime = rclcpp::Time(INS_local_offset_ + timeOfWeek);
  }
  return rostime;
}

rclcpp::Time InertialSenseROS::ros_time_from_start_time(const double time)
{
  rclcpp::Time rostime(0, 0);
  
  //  If we have a GPS fix, then use it to set timestamp
  if (GPS_towOffset_ > 0.001)
  {
    uint64_t sec = UNIX_TO_GPS_OFFSET + floor(time + GPS_towOffset_) + GPS_week_*7*24*3600;
    uint64_t nsec = (time + GPS_towOffset_ - floor(time + GPS_towOffset_))*1e9;
    rostime = rclcpp::Time(sec, nsec);
  }
  else
  {
    // Otherwise, estimate the uINS boot time and offset the messages
    if (!got_first_message_)
    {
      got_first_message_ = true;
      INS_local_offset_ = this->now().seconds() - time;
    }
    else // low-pass filter offset to account for drift
    {
      double y_offset = this->now().seconds() - time;
      INS_local_offset_ = 0.005 * y_offset + 0.995 * INS_local_offset_;
    }
    // Publish with ROS time
    rostime = rclcpp::Time(INS_local_offset_ + time);
  }
  return rostime;
}

rclcpp::Time InertialSenseROS::ros_time_from_tow(const double tow)
{
  return ros_time_from_week_and_tow(GPS_week_, tow);
}

double InertialSenseROS::tow_from_ros_time(const rclcpp::Time &rt)
{
  // return (rt.sec - UNIX_TO_GPS_OFFSET - GPS_week_*604800) + rt.nsec*1.0e-9;
  return rt.seconds() - UNIX_TO_GPS_OFFSET - GPS_week_*604800;
}

rclcpp::Time InertialSenseROS::ros_time_from_gtime(const uint64_t sec, double subsec)
{
    rclcpp::Time out(sec - LEAP_SECONDS, subsec*1e9);
    return out;
}

