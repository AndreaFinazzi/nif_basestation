#include <chrono>
#include <functional>
#include <memory>
#include <string>

// ROS2 messages
#include <deep_orange_msgs/msg/ct_report.hpp>
#include <deep_orange_msgs/msg/joystick_command.hpp>
#include <deep_orange_msgs/msg/misc_report.hpp>
#include <deep_orange_msgs/msg/pt_report.hpp>
#include <deep_orange_msgs/msg/rc_to_ct.hpp>
#include <nif_msgs/msg/dynamic_trajectory.hpp>
#include <nif_msgs/msg/system_status.hpp>
#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nif_msgs/msg/localization_status.hpp"
#include "nif_msgs/msg/system_status.hpp"
#include "nif_msgs/msg/telemetry.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include <visualization_msgs/msg/marker_array.hpp>

// UDP stuff
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
using namespace boost::asio;

using namespace std::chrono_literals;

const unsigned short int OUT_DATA_FRAME_SIZE = 25;
const unsigned short int IN_DATA_FRAME_SIZE = 103;

class Telemetry : public rclcpp::Node {
public:
  Telemetry()
    : Node("telemetry"),
      send_basestation_socket(io_service_main),
      recv_telemetry_socket(io_service_main) {
    // setup QOS to be best effort
    auto qos = rclcpp::QoS(
        rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));
    qos.best_effort();

    // ROS2 topics which should be republished

    // setup QOS to be best effort
    pub_system_status = this->create_publisher<nif_msgs::msg::SystemStatus>(
        "/nif_telemetry/system_status", 10);
    pub_telemetry = this->create_publisher<nif_msgs::msg::Telemetry>(
        "/nif_telemetry/telemetry", 10);
    pub_reference_path = this->create_publisher<nav_msgs::msg::Path>(
        "/nif_telemetry/path_global", 10);

    //   Add traj pub
    pub_reference_traj =
        this->create_publisher<nif_msgs::msg::DynamicTrajectory>(
            "/nif_telemetry/path_traj", 10);

    pub_oppo_prediction_path = this->create_publisher<nav_msgs::msg::Path>(
        "/nif_telemetry/oppo_prediction", 10);
    pub_perception_result =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/nif_telemetry/perception_result", 10);

    sub_rc_to_ct = this->create_subscription<deep_orange_msgs::msg::RcToCt>(
        "/raptor_dbw_interface/rc_to_ct",
        qos,
        std::bind(&Telemetry::rc_to_ct_callback, this, std::placeholders::_1));

    sub_joystick_command =
        this->create_subscription<deep_orange_msgs::msg::JoystickCommand>(
            "/joystick/command",
            qos,
            std::bind(&Telemetry::joystick_command_callback,
                      this,
                      std::placeholders::_1));

    // timer which handles republishing
    timer_ = this->create_wall_timer(
        25ms, std::bind(&Telemetry::timer_callback, this));

    // setup UDP interfaces
    send_basestation_ip = "10.42.4.200";
    send_basestation_port = 23531;
    send_basestation_socket.open(ip::udp::v4());
    send_basestation_endpoint = ip::udp::endpoint(
        ip::address::from_string(send_basestation_ip), send_basestation_port);
    RCLCPP_INFO(
        this->get_logger(),
        "Established connection to send basestation commands to : %s:%u",
        send_basestation_ip.c_str(),
        send_basestation_port);

    recv_telemetry_ip = "10.42.4.79";
    recv_telemetry_port = 23431;
    recv_telemetry_socket.open(ip::udp::v4());
    recv_telemetry_endpoint = ip::udp::endpoint(
        ip::address::from_string(recv_telemetry_ip), recv_telemetry_port);
    recv_telemetry_socket.bind(recv_telemetry_endpoint);
    recv_telemetry_socket.non_blocking(true);
    RCLCPP_INFO(this->get_logger(),
                "Established connection to receive telemetry from on : %s:%u",
                recv_telemetry_ip.c_str(),
                recv_telemetry_port);

    msg_reference_path.poses = std::vector<geometry_msgs::msg::PoseStamped>(10);
    msg_reference_traj.trajectory_path.poses =
        std::vector<geometry_msgs::msg::PoseStamped>(10);
    msg_oppo_prediction_path.poses =
        std::vector<geometry_msgs::msg::PoseStamped>(4);
    msg_perception_result.markers =
        std::vector<visualization_msgs::msg::Marker>(10);
  }

private:
  void timer_callback() {
    // data frame structure for basestation to telemetry message
    double data_frame[OUT_DATA_FRAME_SIZE] = {
        msg_joystick_command.counter,
        msg_joystick_command.emergency_stop,
        msg_joystick_command.joy_enable,
        msg_joystick_command.steering_cmd,
        msg_joystick_command.brake_cmd,
        msg_joystick_command.accelerator_cmd,
        msg_joystick_command.gear_cmd,
        msg_joystick_command.stamp.sec,
        msg_joystick_command.stamp.nanosec,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0};
    boost::system::error_code err;
    send_basestation_socket.send_to(buffer(data_frame, sizeof(data_frame)),
                                    send_basestation_endpoint,
                                    0,
                                    err);

    // receive data from telemetry
    boost::array<double, IN_DATA_FRAME_SIZE> recv_telemetry_buffer;
    // read all messages until none is there anymore
    int i = 0;
    while (recv_telemetry_socket.receive_from(
               boost::asio::buffer(recv_telemetry_buffer),
               recv_telemetry_endpoint,
               0,
               err) != 0) {
      i++;
    }
    // only publish when a new topic was received
    // this is required such that the timeouts still work
    if (i > 0) {
      msg_telemetry.misc_report.stamp.nanosec = recv_telemetry_buffer[0];
      msg_telemetry.misc_report.sys_state = recv_telemetry_buffer[1];
      msg_telemetry.misc_report.ct_state = recv_telemetry_buffer[2];
      msg_telemetry.misc_report.battery_voltage = recv_telemetry_buffer[3];
      msg_telemetry.pt_report.stamp.nanosec = recv_telemetry_buffer[4];
      msg_telemetry.pt_report.fuel_pressure = recv_telemetry_buffer[5];
      msg_telemetry.pt_report.engine_oil_temperature = recv_telemetry_buffer[6];
      msg_telemetry.pt_report.engine_coolant_temperature =
          recv_telemetry_buffer[7];
      msg_telemetry.pt_report.engine_on_status = recv_telemetry_buffer[8];
      msg_telemetry.pt_report.current_gear = recv_telemetry_buffer[9];
      msg_telemetry.pt_report.vehicle_speed_kmph = recv_telemetry_buffer[10];
      msg_telemetry.localization.stamp.nanosec = recv_telemetry_buffer[11];
      msg_telemetry.localization.odometry.pose.position.x =
          recv_telemetry_buffer[12];
      msg_telemetry.localization.odometry.pose.position.y =
          recv_telemetry_buffer[13];
      msg_telemetry.localization.uncertainty = recv_telemetry_buffer[14];
      msg_telemetry.localization.localization_status_code =
          recv_telemetry_buffer[15];
      msg_telemetry.localization.detected_inner_distance =
          recv_telemetry_buffer[16];
      msg_telemetry.localization.detected_outer_distance =
          recv_telemetry_buffer[17];
      msg_telemetry.localization.pos_type_0 = recv_telemetry_buffer[18];
      msg_telemetry.localization.pos_type_1 = recv_telemetry_buffer[19];
      msg_telemetry.localization.heading = recv_telemetry_buffer[20];
      msg_telemetry.control.stamp.nanosec = recv_telemetry_buffer[21];
      msg_telemetry.control.steering_cmd = recv_telemetry_buffer[22];
      msg_telemetry.control.brake_cmd = recv_telemetry_buffer[23];
      msg_telemetry.control.accelerator_cmd = recv_telemetry_buffer[24];
      msg_telemetry.control.gear_cmd = recv_telemetry_buffer[25];
      msg_telemetry.control.desired_velocity_mps = recv_telemetry_buffer[26];
      msg_telemetry.control.crosstrack_error = recv_telemetry_buffer[27];
      msg_telemetry.kinematic.stamp.nanosec = recv_telemetry_buffer[28];
      msg_telemetry.kinematic.wheel_speed_mps = recv_telemetry_buffer[29];
      msg_telemetry.kinematic.steering_wheel_angle_deg =
          recv_telemetry_buffer[30];
      msg_telemetry.tires.stamp.nanosec = recv_telemetry_buffer[31];
      msg_telemetry.tires.temp_front_right = recv_telemetry_buffer[32];
      msg_telemetry.tires.temp_front_left = recv_telemetry_buffer[33];
      msg_telemetry.tires.temp_rear_right = recv_telemetry_buffer[34];
      msg_telemetry.tires.temp_rear_left = recv_telemetry_buffer[35];

      msg_system_status.header.stamp.nanosec = recv_telemetry_buffer[36];
      msg_system_status.autonomy_status.longitudinal_autonomy_enabled =
          recv_telemetry_buffer[37];
      msg_system_status.autonomy_status.lateral_autonomy_enabled =
          recv_telemetry_buffer[38];
      msg_system_status.autonomy_status.emergency_mode_enabled =
          recv_telemetry_buffer[39];
      msg_system_status.health_status.system_failure =
          recv_telemetry_buffer[40];
      msg_system_status.health_status.communication_failure =
          recv_telemetry_buffer[41];
      msg_system_status.health_status.localization_failure =
          recv_telemetry_buffer[42];
      msg_system_status.health_status.commanded_stop =
          recv_telemetry_buffer[43];
      msg_system_status.health_status.system_status_code =
          recv_telemetry_buffer[44];
      msg_system_status.mission_status.stamp_last_update.nanosec =
          recv_telemetry_buffer[45];
      msg_system_status.mission_status.track_flag = recv_telemetry_buffer[46];
      msg_system_status.mission_status.veh_flag = recv_telemetry_buffer[47];
      msg_system_status.mission_status.mission_status_code =
          recv_telemetry_buffer[48];
      msg_system_status.mission_status.max_velocity_mps =
          recv_telemetry_buffer[49];

      // msg_reference_path.header.stamp.nanosec = recv_telemetry_buffer[50];
      // msg_reference_path.header.frame_id = "odom";

      // if (msg_reference_path.poses.size() > 0) {
      //   msg_reference_path.poses[0].pose.position.x =
      //   recv_telemetry_buffer[51];
      //   msg_reference_path.poses[0].pose.position.y =
      //   recv_telemetry_buffer[52];
      // }
      // if (msg_reference_path.poses.size() > 1) {
      //   msg_reference_path.poses[1].pose.position.x =
      //   recv_telemetry_buffer[53];
      //   msg_reference_path.poses[1].pose.position.y =
      //   recv_telemetry_buffer[54];
      // }
      // if (msg_reference_path.poses.size() > 2) {
      //   msg_reference_path.poses[2].pose.position.x =
      //   recv_telemetry_buffer[55];
      //   msg_reference_path.poses[2].pose.position.y =
      //   recv_telemetry_buffer[56];
      // }
      // if (msg_reference_path.poses.size() > 3) {
      //   msg_reference_path.poses[3].pose.position.x =
      //   recv_telemetry_buffer[57];
      //   msg_reference_path.poses[3].pose.position.y =
      //   recv_telemetry_buffer[58];
      // }
      // if (msg_reference_path.poses.size() > 4) {
      //   msg_reference_path.poses[4].pose.position.x =
      //   recv_telemetry_buffer[59];
      //   msg_reference_path.poses[4].pose.position.y =
      //   recv_telemetry_buffer[60];
      // }
      // if (msg_reference_path.poses.size() > 5) {
      //   msg_reference_path.poses[5].pose.position.x =
      //   recv_telemetry_buffer[61];
      //   msg_reference_path.poses[5].pose.position.y =
      //   recv_telemetry_buffer[62];
      // }
      // if (msg_reference_path.poses.size() > 6) {
      //   msg_reference_path.poses[6].pose.position.x =
      //   recv_telemetry_buffer[63];
      //   msg_reference_path.poses[6].pose.position.y =
      //   recv_telemetry_buffer[64];
      // }
      // if (msg_reference_path.poses.size() > 7) {
      //   msg_reference_path.poses[7].pose.position.x =
      //   recv_telemetry_buffer[65];
      //   msg_reference_path.poses[7].pose.position.y =
      //   recv_telemetry_buffer[66];
      // }
      // if (msg_reference_path.poses.size() > 8) {
      //   msg_reference_path.poses[8].pose.position.x =
      //   recv_telemetry_buffer[67];
      //   msg_reference_path.poses[8].pose.position.y =
      //   recv_telemetry_buffer[68];
      // }

      msg_reference_traj.header.stamp.nanosec = recv_telemetry_buffer[50];
      msg_reference_traj.header.frame_id = "odom";

      if (msg_reference_traj.trajectory_path.poses.size() > 0) {
        msg_reference_traj.trajectory_path.poses[0].pose.position.x =
            recv_telemetry_buffer[51];
        msg_reference_traj.trajectory_path.poses[0].pose.position.y =
            recv_telemetry_buffer[52];
      }
      if (msg_reference_traj.trajectory_path.poses.size() > 1) {
        msg_reference_traj.trajectory_path.poses[1].pose.position.x =
            recv_telemetry_buffer[53];
        msg_reference_traj.trajectory_path.poses[1].pose.position.y =
            recv_telemetry_buffer[54];
      }
      if (msg_reference_traj.trajectory_path.poses.size() > 2) {
        msg_reference_traj.trajectory_path.poses[2].pose.position.x =
            recv_telemetry_buffer[55];
        msg_reference_traj.trajectory_path.poses[2].pose.position.y =
            recv_telemetry_buffer[56];
      }
      if (msg_reference_traj.trajectory_path.poses.size() > 3) {
        msg_reference_traj.trajectory_path.poses[3].pose.position.x =
            recv_telemetry_buffer[57];
        msg_reference_traj.trajectory_path.poses[3].pose.position.y =
            recv_telemetry_buffer[58];
      }
      if (msg_reference_traj.trajectory_path.poses.size() > 4) {
        msg_reference_traj.trajectory_path.poses[4].pose.position.x =
            recv_telemetry_buffer[59];
        msg_reference_traj.trajectory_path.poses[4].pose.position.y =
            recv_telemetry_buffer[60];
      }
      if (msg_reference_traj.trajectory_path.poses.size() > 5) {
        msg_reference_traj.trajectory_path.poses[5].pose.position.x =
            recv_telemetry_buffer[61];
        msg_reference_traj.trajectory_path.poses[5].pose.position.y =
            recv_telemetry_buffer[62];
      }
      if (msg_reference_traj.trajectory_path.poses.size() > 6) {
        msg_reference_traj.trajectory_path.poses[6].pose.position.x =
            recv_telemetry_buffer[63];
        msg_reference_traj.trajectory_path.poses[6].pose.position.y =
            recv_telemetry_buffer[64];
      }
      if (msg_reference_traj.trajectory_path.poses.size() > 7) {
        msg_reference_traj.trajectory_path.poses[7].pose.position.x =
            recv_telemetry_buffer[65];
        msg_reference_traj.trajectory_path.poses[7].pose.position.y =
            recv_telemetry_buffer[66];
      }
      if (msg_reference_traj.trajectory_path.poses.size() > 8) {
        msg_reference_traj.trajectory_path.poses[8].pose.position.x =
            recv_telemetry_buffer[67];
        msg_reference_traj.trajectory_path.poses[8].pose.position.y =
            recv_telemetry_buffer[68];
      }

      // Trajectory lat and long planning type
      msg_reference_traj.lat_planning_type = recv_telemetry_buffer[100];
      msg_reference_traj.longi_planning_type = recv_telemetry_buffer[101];
      msg_reference_traj.planning_target_path_type = recv_telemetry_buffer[102];

      if (msg_perception_result.markers.size() > 0) {
        msg_perception_result.markers[0].pose.position.x =
            recv_telemetry_buffer[69];
        msg_perception_result.markers[0].pose.position.y =
            recv_telemetry_buffer[70];
        msg_perception_result.markers[0].pose.orientation.w =
            recv_telemetry_buffer[71];
        msg_perception_result.markers[0].pose.orientation.z =
            recv_telemetry_buffer[72];
      }

      if (msg_perception_result.markers.size() > 1) {
        msg_perception_result.markers[1].pose.position.x =
            recv_telemetry_buffer[73];
        msg_perception_result.markers[1].pose.position.y =
            recv_telemetry_buffer[74];
        msg_perception_result.markers[1].pose.orientation.w =
            recv_telemetry_buffer[75];
        msg_perception_result.markers[1].pose.orientation.z =
            recv_telemetry_buffer[76];
      }

      if (msg_perception_result.markers.size() > 2) {
        msg_perception_result.markers[2].pose.position.x =
            recv_telemetry_buffer[77];
        msg_perception_result.markers[2].pose.position.y =
            recv_telemetry_buffer[78];
        msg_perception_result.markers[2].pose.orientation.w =
            recv_telemetry_buffer[79];
        msg_perception_result.markers[2].pose.orientation.z =
            recv_telemetry_buffer[80];
      }

      if (msg_perception_result.markers.size() > 3) {
        msg_perception_result.markers[3].pose.position.x =
            recv_telemetry_buffer[81];
        msg_perception_result.markers[3].pose.position.y =
            recv_telemetry_buffer[82];
        msg_perception_result.markers[3].pose.orientation.w =
            recv_telemetry_buffer[83];
        msg_perception_result.markers[3].pose.orientation.z =
            recv_telemetry_buffer[84];
      }

      if (msg_perception_result.markers.size() > 4) {
        msg_perception_result.markers[4].pose.position.x =
            recv_telemetry_buffer[85];
        msg_perception_result.markers[4].pose.position.y =
            recv_telemetry_buffer[86];
        msg_perception_result.markers[4].pose.orientation.w =
            recv_telemetry_buffer[87];
        msg_perception_result.markers[4].pose.orientation.z =
            recv_telemetry_buffer[88];
      }

      msg_telemetry.localization.odometry.pose.orientation.z =
          recv_telemetry_buffer[89];
      msg_telemetry.localization.odometry.pose.orientation.w =
          recv_telemetry_buffer[90];

      msg_oppo_prediction_path.header.stamp.nanosec = recv_telemetry_buffer[91];
      msg_oppo_prediction_path.header.frame_id = "odom";

      if (msg_oppo_prediction_path.poses.size() > 0) {
        msg_oppo_prediction_path.poses[0].pose.position.x =
            recv_telemetry_buffer[92];
        msg_oppo_prediction_path.poses[0].pose.position.y =
            recv_telemetry_buffer[93];
      }
      if (msg_oppo_prediction_path.poses.size() > 1) {
        msg_oppo_prediction_path.poses[1].pose.position.x =
            recv_telemetry_buffer[94];
        msg_oppo_prediction_path.poses[1].pose.position.y =
            recv_telemetry_buffer[95];
      }
      if (msg_oppo_prediction_path.poses.size() > 2) {
        msg_oppo_prediction_path.poses[2].pose.position.x =
            recv_telemetry_buffer[96];
        msg_oppo_prediction_path.poses[2].pose.position.y =
            recv_telemetry_buffer[97];
      }
      if (msg_oppo_prediction_path.poses.size() > 3) {
        msg_oppo_prediction_path.poses[3].pose.position.x =
            recv_telemetry_buffer[98];
        msg_oppo_prediction_path.poses[3].pose.position.y =
            recv_telemetry_buffer[99];
      }

      // publish everything on ros2
      pub_system_status->publish(msg_system_status);
      pub_telemetry->publish(msg_telemetry);
      // pub_reference_path->publish(msg_reference_path);
      pub_reference_traj->publish(msg_reference_traj);
      pub_oppo_prediction_path->publish(msg_oppo_prediction_path);
      pub_perception_result->publish(msg_perception_result);
    }
  }
  void rc_to_ct_callback(const deep_orange_msgs::msg::RcToCt::SharedPtr msg) {
    msg_rc_to_ct = std::move(*msg);
  }
  void joystick_command_callback(
      const deep_orange_msgs::msg::JoystickCommand::SharedPtr msg) {
    msg_joystick_command = *msg;
  }

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<nif_msgs::msg::SystemStatus>::SharedPtr pub_system_status;
  rclcpp::Publisher<nif_msgs::msg::Telemetry>::SharedPtr pub_telemetry;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_reference_path;
  rclcpp::Publisher<nif_msgs::msg::DynamicTrajectory>::SharedPtr
      pub_reference_traj;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_oppo_prediction_path;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      pub_perception_result;

  rclcpp::Subscription<deep_orange_msgs::msg::RcToCt>::SharedPtr sub_rc_to_ct;
  rclcpp::Subscription<deep_orange_msgs::msg::JoystickCommand>::SharedPtr
      sub_joystick_command;

  deep_orange_msgs::msg::JoystickCommand msg_joystick_command;
  nif_msgs::msg::Telemetry msg_telemetry;
  nif_msgs::msg::SystemStatus msg_system_status;
  nav_msgs::msg::Path msg_reference_path;
  nif_msgs::msg::DynamicTrajectory msg_reference_traj;
  nav_msgs::msg::Path msg_oppo_prediction_path;
  visualization_msgs::msg::MarkerArray msg_perception_result;

  deep_orange_msgs::msg::RcToCt msg_rc_to_ct;

  // udp stuff
  io_service io_service_main;
  ip::udp::socket send_basestation_socket;
  ip::udp::socket recv_telemetry_socket;
  ip::udp::endpoint send_basestation_endpoint;
  ip::udp::endpoint recv_telemetry_endpoint;
  std::string send_basestation_ip;
  unsigned int send_basestation_port;
  std::string recv_telemetry_ip;
  unsigned int recv_telemetry_port;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Telemetry>());
  rclcpp::shutdown();
  return 0;
}
