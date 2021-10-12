#include <chrono>
#include <deep_orange_msgs/msg/ct_report.hpp>
#include <deep_orange_msgs/msg/misc_report.hpp>
#include <deep_orange_msgs/msg/pt_report.hpp>
#include <deep_orange_msgs/msg/telemetry.hpp>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

using namespace std::chrono_literals;

class Telemetry : public rclcpp::Node {
   public:
    Telemetry() : Node("telemetry") {
        // setup QOS to be best effort
        auto qos = rclcpp::QoS(
            rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));
        qos.best_effort();

        telemetry_msg = deep_orange_msgs::msg::Telemetry();

        pub_telemetry =
            this->create_publisher<deep_orange_msgs::msg::Telemetry>(
                "/telemetry/vehicle_state", qos);

        pub_ct_report = this->create_publisher<deep_orange_msgs::msg::CtReport>(
            "/telemetry/ct_report", qos);
        pub_pt_report = this->create_publisher<deep_orange_msgs::msg::PtReport>(
            "/telemetry/pt_report", qos);
        pub_misc_report_do =
            this->create_publisher<deep_orange_msgs::msg::MiscReport>(
                "/telemetry/misc_report_do", qos);

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

        timer_ = this->create_wall_timer(
            200ms, std::bind(&Telemetry::timer_callback, this));
    }

   private:
    void timer_callback() {
        telemetry_msg.stamp = rclcpp::Clock().now();
        pub_telemetry->publish(telemetry_msg);
        pub_ct_report->publish(msg_ct_report);
        pub_pt_report->publish(msg_pt_report);
        pub_misc_report_do->publish(msg_misc_report_do);
    }

    void ct_report_callback(
        const deep_orange_msgs::msg::CtReport::SharedPtr msg) {
        msg_ct_report = *msg;
        telemetry_msg.track_cond_ack = msg->track_cond_ack;
        telemetry_msg.veh_sig_ack = msg->veh_sig_ack;
        telemetry_msg.ct_state = msg->ct_state;
    }

    void pt_report_callback(
        const deep_orange_msgs::msg::PtReport::SharedPtr msg) {
        msg_pt_report = *msg;
        telemetry_msg.map_sensor = msg->map_sensor;
        telemetry_msg.lambda_sensor = msg->lambda_sensor;
        telemetry_msg.fuel_level = msg->fuel_level;
        telemetry_msg.fuel_pressure = msg->fuel_pressure;
        telemetry_msg.engine_oil_pressure = msg->engine_oil_pressure;
        telemetry_msg.engine_oil_temperature = msg->engine_oil_temperature;
        telemetry_msg.engine_coolant_temperature =
            msg->engine_coolant_temperature;
        telemetry_msg.engine_coolant_pressure = msg->engine_coolant_pressure;
        telemetry_msg.engine_rpm = msg->engine_rpm;
        telemetry_msg.engine_on_status = msg->engine_on_status;
        telemetry_msg.engine_run_switch_status = msg->engine_run_switch_status;
        telemetry_msg.throttle_position = msg->throttle_position;
        telemetry_msg.current_gear = msg->current_gear;
        telemetry_msg.gear_shift_status = msg->gear_shift_status;
        telemetry_msg.transmission_oil_pressure =
            msg->transmission_oil_pressure;
        telemetry_msg.transmission_accumulator_pressure =
            msg->transmission_accumulator_pressure;
        telemetry_msg.transmission_oil_temperature =
            msg->transmission_oil_temperature;
        telemetry_msg.vehicle_speed_kmph = msg->vehicle_speed_kmph;
    }

    void misc_report_do_callback(
        const deep_orange_msgs::msg::MiscReport::SharedPtr msg) {
        msg_misc_report_do = *msg;
        telemetry_msg.battery_voltage = msg->battery_voltage;
        telemetry_msg.off_grid_power_connection =
            msg->off_grid_power_connection;
        telemetry_msg.safety_switch_state = msg->safety_switch_state;
        telemetry_msg.mode_switch_state = msg->mode_switch_state;
        telemetry_msg.sys_state = msg->sys_state;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<deep_orange_msgs::msg::Telemetry>::SharedPtr
        pub_telemetry;

    rclcpp::Publisher<deep_orange_msgs::msg::CtReport>::SharedPtr pub_ct_report;
    rclcpp::Publisher<deep_orange_msgs::msg::PtReport>::SharedPtr pub_pt_report;
    rclcpp::Publisher<deep_orange_msgs::msg::MiscReport>::SharedPtr
        pub_misc_report_do;
    
    rclcpp::Subscription<deep_orange_msgs::msg::CtReport>::SharedPtr
        sub_ct_report;
    rclcpp::Subscription<deep_orange_msgs::msg::PtReport>::SharedPtr
        sub_pt_report;
    rclcpp::Subscription<deep_orange_msgs::msg::MiscReport>::SharedPtr
        sub_misc_report_do;

    deep_orange_msgs::msg::Telemetry telemetry_msg;

    deep_orange_msgs::msg::CtReport msg_ct_report;
    deep_orange_msgs::msg::PtReport msg_pt_report;
    deep_orange_msgs::msg::MiscReport msg_misc_report_do;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Telemetry>());
    rclcpp::shutdown();
    return 0;
}
