#pragma once

#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <string>
#include <cstdlib>

#include "InertialSense.h"

#include <rclcpp/rclcpp.hpp>
// #include "ros/timer.h"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "inertial_sense/msg/gps.hpp"
#include "inertial_sense/msg/gps_info.hpp"
#include "inertial_sense/msg/pre_int_imu.hpp"
#include "inertial_sense/srv/firmware_update.hpp"
#include "inertial_sense/srv/ref_lla_update.hpp"
#include "inertial_sense/msg/rtk_rel.hpp"
#include "inertial_sense/msg/rtk_info.hpp"
#include "inertial_sense/msg/gnss_ephemeris.hpp"
#include "inertial_sense/msg/glonass_ephemeris.hpp"
#include "inertial_sense/msg/gnss_observation.hpp"
#include "inertial_sense/msg/gnss_obs_vec.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include <tf2_ros/transform_broadcaster.h>
//#include "geometry/xform.h"

# define GPS_UNIX_OFFSET 315964800 // GPS time started on 6/1/1980 while UNIX time started 1/1/1970 this is the difference between those in seconds
# define LEAP_SECONDS 18 // GPS time does not have leap seconds, UNIX does (as of 1/1/2017 - next one is probably in 2020 sometime unless there is some crazy earthquake or nuclear blast)
# define UNIX_TO_GPS_OFFSET (GPS_UNIX_OFFSET - LEAP_SECONDS)

#define SET_CALLBACK(DID, __type, __cb_fun, __periodmultiple) \
    IS_.BroadcastBinaryData(DID, __periodmultiple, \
    [this](InertialSense*i, p_data_t* data, int pHandle)\
    { \
       /* ROS_INFO("Got message %d", DID);*/\
       this->__cb_fun(reinterpret_cast<__type*>(data->buf));\
    })

class InertialSenseROS : public rclcpp::Node
{
public:
  typedef enum
  {
    NMEA_GPGGA = 0x01,
    NMEA_GPGLL = 0x02,
    NMEA_GPGSA = 0x04,
    NMEA_GPRMC = 0x08,
    NMEA_SER0 = 0x01,
    NMEA_SER1 = 0x02
  } NMEA_message_config_t;
      
  InertialSenseROS();
  void callback(p_data_t* data);
  void update();

  void connect();
  void set_navigation_dt_ms();
  void configure_parameters();
  void configure_rtk();
  void configure_data_streams();
  void configure_ascii_output();
  void start_log();
  
  template<typename T> void set_vector_flash_config(std::string param_name, uint32_t size, uint32_t offset);
  template<typename T>  void set_flash_config(std::string param_name, uint32_t offset, T def) __attribute__ ((optimize(0)));
  void get_flash_config();
  void reset_device();
  void flash_config_callback(const nvm_flash_cfg_t* const msg);
  // Serial Port Configuration
  std::string port_;
  int baudrate_;
  bool initialized_;
  bool log_enabled_;

  std::string frame_id_;

  // ROS Stream handling
  // typedef struct
  // {
  //   bool enabled;
  //   ros::Publisher pub;
  //   ros::Publisher pub2;
  // } ros_stream_t;

  bool INS_enabled_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr INS_pub_;
  void INS1_callback(const ins_1_t* const msg);
  void INS2_callback(const ins_2_t* const msg);
//  void INS_variance_callback(const inl2_variance_t* const msg);

  // tf2::TransformBroadcaster br;
  std::shared_ptr<tf2_ros::TransformBroadcaster> br;
  bool publishTf;
  geometry_msgs::msg::TransformStamped transform;
  int LTCF;
  enum
  {
    NED,
    ENU
  }ltcf;

  bool IMU_enabled_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr IMU_pub_;
  void IMU_callback(const dual_imu_t* const msg);

  bool GPS_enabled_;
  rclcpp::Publisher<inertial_sense::msg::GPS>::SharedPtr GPS_pub_;

  bool GPS_obs_enabled_;
  rclcpp::Publisher<inertial_sense::msg::GNSSObsVec>::SharedPtr GPS_obs_pub_;
  rclcpp::Publisher<inertial_sense::msg::GNSSEphemeris>::SharedPtr GPS_eph_pub_;
  rclcpp::Publisher<inertial_sense::msg::GlonassEphemeris>::SharedPtr GPS_eph_pub2_;
  void GPS_pos_callback(const gps_pos_t* const msg);
  void GPS_vel_callback(const gps_vel_t* const msg);
  void GPS_raw_callback(const gps_raw_t* const msg);
  void GPS_obs_callback(const obsd_t * const msg, int nObs);
  void GPS_eph_callback(const eph_t* const msg);
  void GPS_geph_callback(const geph_t* const msg);
  void GPS_obs_bundle_timer_callback();
  inertial_sense::msg::GNSSObsVec obs_Vec_;
  rclcpp::TimerBase::SharedPtr obs_bundle_timer_;
  rclcpp::Time last_obs_time_;

  bool GPS_info_enabled_;
  rclcpp::Publisher<inertial_sense::msg::GPSInfo>::SharedPtr GPS_info_pub_;
  void GPS_info_callback(const gps_sat_t* const msg);

  bool mag_enabled_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  void mag_callback(const magnetometer_t* const msg);

  bool baro_enabled_;
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr baro_pub_;
  void baro_callback(const barometer_t* const msg);

  bool dt_vel_enabled_;
  rclcpp::Publisher<inertial_sense::msg::PreIntIMU>::SharedPtr dt_vel_pub_;
  void preint_IMU_callback(const preintegrated_imu_t * const msg);
  
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr strobe_pub_;
  void strobe_in_time_callback(const strobe_in_time_t * const msg);

  bool diagnostics_enabled_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  void diagnostics_callback();
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  float diagnostic_ar_ratio_, diagnostic_differential_age_, diagnostic_heading_base_to_rover_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr mag_cal_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr multi_mag_cal_srv_;
  rclcpp::Service<inertial_sense::srv::FirmwareUpdate>::SharedPtr firmware_update_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr refLLA_set_current_srv_;
  rclcpp::Service<inertial_sense::srv::RefLLAUpdate>::SharedPtr refLLA_set_value_srv_;
  void set_current_position_as_refLLA(const std::shared_ptr<rmw_request_id_t> request_header,
              const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
              std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void set_refLLA_to_value(const std::shared_ptr<rmw_request_id_t> request_header,
              const std::shared_ptr<inertial_sense::srv::RefLLAUpdate::Request> req,
              std::shared_ptr<inertial_sense::srv::RefLLAUpdate::Response> res);
  void perform_mag_cal_srv_callback(const std::shared_ptr<rmw_request_id_t> request_header,
              const std::shared_ptr<std_srvs::srv::Trigger::Request> req, 
              std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void perform_multi_mag_cal_srv_callback(const std::shared_ptr<rmw_request_id_t> request_header,
              const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
              std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void update_firmware_srv_callback(const std::shared_ptr<rmw_request_id_t> request_header,
              const std::shared_ptr<inertial_sense::srv::FirmwareUpdate::Request> req,  
              std::shared_ptr<inertial_sense::srv::FirmwareUpdate::Response> res);

  void publishGPS();

  typedef enum
  {
    RTK_NONE,
    RTK_ROVER,
    RTK_BASE,
    DUAL_GNSS
  } rtk_state_t;
  rtk_state_t RTK_state_ = RTK_NONE;

  bool RTK_enabled_;
  rclcpp::Publisher<inertial_sense::msg::RTKInfo>::SharedPtr RTK_pub_;
  rclcpp::Publisher<inertial_sense::msg::RTKRel>::SharedPtr RTK_pub2_;
  void RTK_Misc_callback(const gps_rtk_misc_t* const msg);
  void RTK_Rel_callback(const gps_rtk_rel_t* const msg);

  
  /**
   * @brief ros_time_from_week_and_tow
   * Get current ROS time from week and tow
   * @param week Weeks since January 6th, 1980
   * @param timeOfWeek Time of week (since Sunday morning) in seconds, GMT
   * @return equivalent ros::Time
   */
  rclcpp::Time ros_time_from_week_and_tow(const uint32_t week, const double timeOfWeek);
  
  /**
   * @brief ros_time_from_start_time
   * @param time - Time since boot up in seconds - Convert to GPS time of week by adding gps.towOffset
   * @return equivalent ros::Time
   */
  rclcpp::Time ros_time_from_start_time(const double time);
  
  /**
   * @brief ros_time_from_tow
   * Get equivalent ros time from tow and internal week counter
   * @param tow Time of Week (seconds)
   * @return equivalent ros::Time
   */
  rclcpp::Time ros_time_from_tow(const double tow);

  double tow_from_ros_time(const rclcpp::Time& rt);
  rclcpp::Time ros_time_from_gtime(const uint64_t sec, double subsec);

  double GPS_towOffset_ = 0; // The offset between GPS time-of-week and local time on the uINS
                             //  If this number is 0, then we have not yet got a fix
  uint64_t GPS_week_ = 0; // Week number to start of GPS_towOffset_ in GPS time
  // Time sync variables
  double INS_local_offset_ = 0.0; // Current estimate of the uINS start time in ROS time seconds
  bool got_first_message_ = false; // Flag to capture first uINS start time guess

  // Data to hold on to in between callbacks
  double lla_[3];
  double ecef_[3];
  sensor_msgs::msg::Imu imu1_msg, imu2_msg;
  nav_msgs::msg::Odometry odom_msg;
  inertial_sense::msg::GPS gps_msg; 
  geometry_msgs::msg::Vector3Stamped gps_velEcef;
  inertial_sense::msg::GPSInfo gps_info_msg;

  // Connection to the uINS
  InertialSense IS_;
};