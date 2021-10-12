#include <chrono>
#include <functional>
#include <memory>
#include <string>

// ROS2 messages
#include <nif_msgs/msg/system_status.hpp>
#include <deep_orange_msgs/msg/ct_report.hpp>
#include <deep_orange_msgs/msg/joystick_command.hpp>
#include <deep_orange_msgs/msg/misc_report.hpp>
#include <deep_orange_msgs/msg/pt_report.hpp>
#include <deep_orange_msgs/msg/rc_to_ct.hpp>
#include <rclcpp/rclcpp.hpp>

// UDP stuff
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
using namespace boost::asio;

using namespace std::chrono_literals;

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
        pub_ct_report = this->create_publisher<deep_orange_msgs::msg::CtReport>(
            "/telemetry/ct_report", qos);
        pub_pt_report = this->create_publisher<deep_orange_msgs::msg::PtReport>(
            "/telemetry/pt_report", qos);
        pub_misc_report_do =
            this->create_publisher<deep_orange_msgs::msg::MiscReport>(
            "/telemetry/misc_report_do", qos);
        pub_safety_status = this->create_publisher<nif_msgs::msg::SystemStatus>(
            "/telemetry/system_status", qos);

        sub_rc_to_ct = this->create_subscription<deep_orange_msgs::msg::RcToCt>(
            "/raptor_dbw_interface/rc_to_ct", qos,
            std::bind(&Telemetry::rc_to_ct_callback, this,
                      std::placeholders::_1));
        sub_joystick_command =
            this->create_subscription<deep_orange_msgs::msg::JoystickCommand>(
                "/joystick/command", qos,
                std::bind(&Telemetry::joystick_command_callback, this,
                          std::placeholders::_1));

        // timer which handles republishing
        timer_ = this->create_wall_timer(
            25ms, std::bind(&Telemetry::timer_callback, this));

        // setup UDP interfaces
        send_basestation_ip = "10.42.4.200";
        send_basestation_port = 23531;
        send_basestation_socket.open(ip::udp::v4());
        send_basestation_endpoint =
            ip::udp::endpoint(ip::address::from_string(send_basestation_ip),
                              send_basestation_port);
        RCLCPP_INFO(
            this->get_logger(),
            "Established connection to send basestation commands to : %s:%u",
            send_basestation_ip.c_str(), send_basestation_port);

        recv_telemetry_ip = "10.42.4.1";
        recv_telemetry_port = 23431;
        recv_telemetry_socket.open(ip::udp::v4());
        recv_telemetry_endpoint = ip::udp::endpoint(
            ip::address::from_string(recv_telemetry_ip), recv_telemetry_port);
        recv_telemetry_socket.bind(recv_telemetry_endpoint);
        recv_telemetry_socket.non_blocking(true);
        RCLCPP_INFO(
            this->get_logger(),
            "Established connection to receive telemetry from on : %s:%u",
            recv_telemetry_ip.c_str(), recv_telemetry_port);
    }

   private:
    void timer_callback() {
        // data frame structure for basestation to telemetry message
        double data_frame[50] = {msg_joystick_command.counter,
                                 msg_joystick_command.emergency_stop,
                                 msg_joystick_command.joy_enable,
                                 msg_joystick_command.steering_cmd,
                                 msg_joystick_command.brake_cmd,
                                 msg_joystick_command.accelerator_cmd,
                                 msg_joystick_command.gear_cmd,
                                 msg_rc_to_ct.track_cond,
                                 msg_rc_to_ct.rolling_counter,
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
                                        send_basestation_endpoint, 0, err);

        // receive data from telemetry
        boost::array<double, 50> recv_telemetry_buffer;
        // read all messages until none is there anymore
        int i = 0;
        while (recv_telemetry_socket.receive_from(
                   boost::asio::buffer(recv_telemetry_buffer),
                   recv_telemetry_endpoint, 0, err) != 0) {
            i++;
        }
        // only publish when a new topic was received
        // this is required such that the timeouts still work
        if (i > 0) {
            msg_ct_report.track_cond_ack = recv_telemetry_buffer[0];
            msg_ct_report.veh_sig_ack = recv_telemetry_buffer[1];
            msg_ct_report.ct_state = recv_telemetry_buffer[2];
            msg_ct_report.rolling_counter = recv_telemetry_buffer[3];

            msg_pt_report.fuel_pressure = recv_telemetry_buffer[4];
            msg_pt_report.transmission_oil_temperature =
                recv_telemetry_buffer[5];
            msg_pt_report.engine_oil_temperature = recv_telemetry_buffer[6];
            msg_pt_report.engine_coolant_temperature = recv_telemetry_buffer[7];
            msg_pt_report.engine_rpm = recv_telemetry_buffer[8];
            msg_pt_report.current_gear = recv_telemetry_buffer[9];

            msg_misc_report_do.battery_voltage = recv_telemetry_buffer[10];
            msg_misc_report_do.sys_state = recv_telemetry_buffer[11];

            msg_safety_status.gps_healthy = recv_telemetry_buffer[12];
            msg_safety_status.comms_healthy = recv_telemetry_buffer[13];
            msg_safety_status.joy_emergency = recv_telemetry_buffer[14];
            msg_safety_status.lat_stdev = recv_telemetry_buffer[15];
            msg_safety_status.long_stdev = recv_telemetry_buffer[16];
            msg_safety_status.best_pos_lat_stdev = recv_telemetry_buffer[17];
            msg_safety_status.best_pos_long_stdev = recv_telemetry_buffer[18];
            msg_safety_status.time_since_last_update =
                recv_telemetry_buffer[19];

            // publish everything on ros2
            pub_ct_report->publish(msg_ct_report);
            pub_pt_report->publish(msg_pt_report);
            pub_misc_report_do->publish(msg_misc_report_do);
            pub_safety_status->publish(msg_safety_status);
        }
    }
    void rc_to_ct_callback(const deep_orange_msgs::msg::RcToCt::SharedPtr msg) {
        msg_rc_to_ct = *msg;
    }
    void joystick_command_callback(
        const deep_orange_msgs::msg::JoystickCommand::SharedPtr msg) {
        msg_joystick_command = *msg;
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<deep_orange_msgs::msg::CtReport>::SharedPtr pub_ct_report;
    rclcpp::Publisher<deep_orange_msgs::msg::PtReport>::SharedPtr pub_pt_report;
    rclcpp::Publisher<deep_orange_msgs::msg::MiscReport>::SharedPtr
        pub_misc_report_do;
    rclcpp::Publisher<nif_msgs::msg::SystemStatus>::SharedPtr pub_safety_status;
    rclcpp::Subscription<deep_orange_msgs::msg::RcToCt>::SharedPtr sub_rc_to_ct;
    rclcpp::Subscription<deep_orange_msgs::msg::JoystickCommand>::SharedPtr
        sub_joystick_command;

    deep_orange_msgs::msg::CtReport msg_ct_report;
    deep_orange_msgs::msg::PtReport msg_pt_report;
    deep_orange_msgs::msg::MiscReport msg_misc_report_do;
    nif_msgs::msg::SystemStatus msg_safety_status;
    deep_orange_msgs::msg::RcToCt msg_rc_to_ct;
    deep_orange_msgs::msg::JoystickCommand msg_joystick_command;

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
