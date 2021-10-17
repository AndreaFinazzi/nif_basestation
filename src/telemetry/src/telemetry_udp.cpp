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
#include <rclcpp/rclcpp.hpp>
#include <nif_msgs/msg/system_status.hpp>

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
          send_telemetry_socket(io_service_main),
          recv_basestation_socket(io_service_main) {
        // setup QOS to be best effort
        auto qos = rclcpp::QoS(
            rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));
        qos.best_effort();

        // ROS2 topics which should be republished
        sub_ct_report =
            this->create_subscription<deep_orange_msgs::msg::CtReport>(
                "/raptor_dbw_interface/ct_report", 1,
                std::bind(&Telemetry::ct_report_callback, this,
                          std::placeholders::_1));
        sub_pt_report =
            this->create_subscription<deep_orange_msgs::msg::PtReport>(
                "/raptor_dbw_interface/pt_report", 1,
                std::bind(&Telemetry::pt_report_callback, this,
                          std::placeholders::_1));
        sub_misc_report_do =
            this->create_subscription<deep_orange_msgs::msg::MiscReport>(
                "/raptor_dbw_interface/misc_report_do", 1,
                std::bind(&Telemetry::misc_report_do_callback, this,
                          std::placeholders::_1));
        sub_safety_status =
            this->create_subscription<nif_msgs::msg::SystemStatus>(
                "/telemetry/system_status", qos,
                std::bind(&Telemetry::safety_status_callback, this,
                          std::placeholders::_1));

        pub_rc_to_ct = this->create_publisher<deep_orange_msgs::msg::RcToCt>(
            "/raptor_dbw_interface/rc_to_ct", rclcpp::QoS{10});
        pub_joystick_command =
            this->create_publisher<deep_orange_msgs::msg::JoystickCommand>(
                "/joystick/command", qos);

        // timer which handles sending to base station
        send_bs_timer_ = this->create_wall_timer(
            25ms, std::bind(&Telemetry::send_bs_callback, this));

        // timer which handles receiving from base station
        rec_bs_timer_ = this->create_wall_timer(
            25ms, std::bind(&Telemetry::rec_bs_callback, this));

        // setup UDP interfaces
        send_telemetry_ip = "10.42.4.1";
        send_telemetry_port = 23431;
        send_telemetry_socket.open(ip::udp::v4());
        send_telemetry_endpoint = ip::udp::endpoint(
            ip::address::from_string(send_telemetry_ip), send_telemetry_port);
        RCLCPP_INFO(this->get_logger(),
                    "Established connection to send telemetry to : %s:%u",
                    send_telemetry_ip.c_str(), send_telemetry_port);

        recv_basestation_ip = "10.42.4.200";
        recv_basestation_port = 23531;
        recv_basestation_socket.open(ip::udp::v4());
        recv_basestation_endpoint =
            ip::udp::endpoint(ip::address::from_string(recv_basestation_ip),
                              recv_basestation_port);
        recv_basestation_socket.bind(recv_basestation_endpoint);
        recv_basestation_socket.non_blocking(true);
        send_telemetry_endpoint = ip::udp::endpoint(
            ip::address::from_string(send_telemetry_ip), send_telemetry_port);
        RCLCPP_INFO(
            this->get_logger(),
            "Established connection to receive basestation commands on : %s:%u",
            recv_basestation_ip.c_str(), recv_basestation_port);
    }

   private:
    void send_bs_callback() {
        // data frame structure for telemtry to basestation message
        // added 40 dummy variables for load testing
        double data_frame[50] = {msg_ct_report.track_cond_ack,
                                 msg_ct_report.veh_sig_ack,
                                 msg_ct_report.ct_state,
                                 msg_ct_report.rolling_counter,
                                 msg_pt_report.fuel_pressure,
                                 msg_pt_report.transmission_oil_temperature,
                                 msg_pt_report.engine_oil_temperature,
                                 msg_pt_report.engine_coolant_temperature,
                                 msg_pt_report.engine_rpm,
                                 msg_pt_report.current_gear,
                                 msg_misc_report_do.battery_voltage,
                                 msg_misc_report_do.sys_state,
                                 msg_system_status.,
                                 msg_system_status.,
                                 msg_system_status.,
                                 msg_system_status.,
                                 msg_system_status.,
                                 msg_system_status.,
                                 msg_system_status.,
                                 msg_system_status.,
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
        send_telemetry_socket.send_to(buffer(data_frame, sizeof(data_frame)),
                                      send_telemetry_endpoint, 0, err);
    }
    void rec_bs_callback() {
        boost::system::error_code err;
        // receive data from basestation
        // add 40 values which are unused
        boost::array<double, 50> recv_basestation_buffer;
        // read all messages until none is there anymore
        int i = 0;
        while (recv_basestation_socket.receive_from(
                   boost::asio::buffer(recv_basestation_buffer),
                   recv_basestation_endpoint, 0, err) != 0) {
            i++;
        }
        // only publish when a new topic was received
        // this is required such that the timeouts still work
        if (i > 0) {
            msg_joystick_command.counter = recv_basestation_buffer[0];
            msg_joystick_command.emergency_stop = recv_basestation_buffer[1];
            msg_joystick_command.joy_enable = recv_basestation_buffer[2];
            msg_joystick_command.steering_cmd = recv_basestation_buffer[3];
            msg_joystick_command.brake_cmd = recv_basestation_buffer[4];
            msg_joystick_command.accelerator_cmd = recv_basestation_buffer[5];
            msg_joystick_command.gear_cmd = recv_basestation_buffer[6];

            msg_rc_to_ct.track_cond = recv_basestation_buffer[7];
            msg_rc_to_ct.rolling_counter = recv_basestation_buffer[8];

            // publish everything on ros2
            pub_joystick_command->publish(msg_joystick_command);
            pub_rc_to_ct->publish(msg_rc_to_ct);
        }
    }
    void ct_report_callback(
        const deep_orange_msgs::msg::CtReport::SharedPtr msg) {
        msg_ct_report = *msg;
    }
    void pt_report_callback(
        const deep_orange_msgs::msg::PtReport::SharedPtr msg) {
        msg_pt_report = *msg;
    }
    void misc_report_do_callback(
        const deep_orange_msgs::msg::MiscReport::SharedPtr msg) {
        msg_misc_report_do = *msg;
    }
    void safety_status_callback(const nif_msgs::msg::SystemStatus::SharedPtr msg) {
        msg_system_status = *msg;
    }
    rclcpp::TimerBase::SharedPtr send_bs_timer_;
    rclcpp::TimerBase::SharedPtr rec_bs_timer_;

    // subcribers of data to be published out to base station
    rclcpp::Subscription<deep_orange_msgs::msg::CtReport>::SharedPtr
        sub_ct_report;
    rclcpp::Subscription<deep_orange_msgs::msg::PtReport>::SharedPtr
        sub_pt_report;
    rclcpp::Subscription<deep_orange_msgs::msg::MiscReport>::SharedPtr
        sub_misc_report_do;
    rclcpp::Subscription<nif_msgs::msg::SystemStatus>::SharedPtr sub_safety_status;

    // re-publish incoming data from base station
    rclcpp::Publisher<deep_orange_msgs::msg::RcToCt>::SharedPtr pub_rc_to_ct;
    rclcpp::Publisher<deep_orange_msgs::msg::JoystickCommand>::SharedPtr
        pub_joystick_command;

    deep_orange_msgs::msg::CtReport msg_ct_report;
    deep_orange_msgs::msg::PtReport msg_pt_report;
    deep_orange_msgs::msg::MiscReport msg_misc_report_do;
    nif_msgs::msg::SystemStatus msg_system_status;
    deep_orange_msgs::msg::RcToCt msg_rc_to_ct;
    deep_orange_msgs::msg::JoystickCommand msg_joystick_command;

    // udp stuff
    io_service io_service_main;
    ip::udp::socket send_telemetry_socket;
    ip::udp::socket recv_basestation_socket;
    ip::udp::endpoint send_telemetry_endpoint;
    ip::udp::endpoint recv_basestation_endpoint;
    std::string send_telemetry_ip;
    unsigned int send_telemetry_port;
    std::string recv_basestation_ip;
    unsigned int recv_basestation_port;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Telemetry>());
    rclcpp::shutdown();
    return 0;
}
