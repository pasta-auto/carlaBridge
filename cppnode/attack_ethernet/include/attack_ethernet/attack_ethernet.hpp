#ifndef ATTACK_ETHERNET_HPP
#define ATTACK_ETHERNET_HPP

#include <rclcpp/callback_group.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/header.hpp>
#include <autoware_sensing_msgs/msg/gnss_ins_orientation_stamped.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/hand_brake_report.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <tier4_vehicle_msgs/msg/actuation_status_stamped.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <utility>

#include <attack_ethernet_msgs/msg/ethernet_attack_status.hpp>
#include <attack_ethernet_msgs/msg/ethernet_attack_config.hpp>
#include <attack_ethernet/config.hpp>
#include <attack_ethernet/config_node.hpp>


class EthernetAttackNode : public rclcpp::Node
{
public: 
    explicit EthernetAttackNode(std::shared_ptr<EthernetAttackConfig> config);

    void camera_callback(sensor_msgs::msg::Image::UniquePtr msg);
    void camera_info_callback(sensor_msgs::msg::CameraInfo::UniquePtr msg);
    void lidar_callback(sensor_msgs::msg::PointCloud2::UniquePtr msg);
    void nav_sat_callback(sensor_msgs::msg::NavSatFix::UniquePtr msg);
    void ecef_vel_callback(geometry_msgs::msg::TwistWithCovarianceStamped::UniquePtr msg);
    void imu_callback(sensor_msgs::msg::Imu::UniquePtr msg);
    void gnss_ins_callback(autoware_sensing_msgs::msg::GnssInsOrientationStamped::UniquePtr msg);
    void auto_steering_status_callback(autoware_auto_vehicle_msgs::msg::SteeringReport::UniquePtr msg);
    void auto_hazard_status_callback(autoware_auto_vehicle_msgs::msg::HazardLightsReport::UniquePtr msg);
    void auto_turning_status_callback(autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::UniquePtr msg);
    void auto_gear_report_callback(autoware_auto_vehicle_msgs::msg::GearReport::UniquePtr msg);
    void velocity_report_callback(autoware_auto_vehicle_msgs::msg::VelocityReport::UniquePtr msg);
    void actuation_status_callback(tier4_vehicle_msgs::msg::ActuationStatusStamped::UniquePtr msg);
    void handbrake_status_callback(autoware_auto_vehicle_msgs::msg::HandBrakeReport::UniquePtr msg);
    void headlights_status_callback(std_msgs::msg::Int16::UniquePtr msg);
    void twist_callback(geometry_msgs::msg::TwistWithCovarianceStamped::UniquePtr msg);

private:
    // NOTE: If more attack targets are needed, the EthernetAttackConfig struct
    // will need to be modified to store attack target as uint32
    enum AttackNum
    {
        CAMERA,
        CAMERA_INFO,
        LIDAR,
        NAV_SAT,
        ECEF_VEL,
        IMU,
        GNSS_INS,
        AUTO_STEERING_STATUS,
        AUTO_HAZARD_STATUS,
        AUTO_TURNING_STATUS,
        AUTO_GEAR_REPORT,
        VELOCITY_REPORT,
        ACTUATION_STATUS,
        HANDBRAKE_STATUS,
        HEADLIGHTS_STATUS,
        TWIST,

        NUM_TARGETS,
    };
    std::array<std::mutex, AttackNum::NUM_TARGETS> pub_mutex;

    std::shared_ptr<EthernetAttackConfig> attack_config;

    rclcpp::CallbackGroup::SharedPtr _callback_group;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_loc;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr nav_sat_sub;
    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr ecef_vel_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Subscription<autoware_sensing_msgs::msg::GnssInsOrientationStamped>::SharedPtr gnss_ins_sub;
    rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr auto_steering_status_sub;
    rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::HazardLightsReport>::SharedPtr auto_hazard_status_sub;
    rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>::SharedPtr auto_turning_status_sub;
    rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::GearReport>::SharedPtr auto_gear_report_sub;
    rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_report_sub;
    rclcpp::Subscription<tier4_vehicle_msgs::msg::ActuationStatusStamped>::SharedPtr actuation_status_sub;
    rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::HandBrakeReport>::SharedPtr handbrake_status_sub;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr headlights_status_sub;
    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_sub;

    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image, std::allocator<void>>> camera_pub;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo, std::allocator<void>>> camera_info_pub;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2, std::allocator<void>>> lidar_pub_loc;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::NavSatFix, std::allocator<void>>> nav_sat_pub;
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped, std::allocator<void>>> ecef_vel_pub;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu, std::allocator<void>>> imu_pub;
    std::shared_ptr<rclcpp::Publisher<autoware_sensing_msgs::msg::GnssInsOrientationStamped, std::allocator<void>>> gnss_ins_pub;
    std::shared_ptr<rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::SteeringReport, std::allocator<void>>> auto_steering_status_pub;
    std::shared_ptr<rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::HazardLightsReport, std::allocator<void>>> auto_hazard_status_pub;
    std::shared_ptr<rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport, std::allocator<void>>> auto_turning_status_pub;
    std::shared_ptr<rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::GearReport, std::allocator<void>>> auto_gear_report_pub;
    std::shared_ptr<rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::VelocityReport, std::allocator<void>>> velocity_report_pub;
    std::shared_ptr<rclcpp::Publisher<tier4_vehicle_msgs::msg::ActuationStatusStamped, std::allocator<void>>> actuation_status_pub;
    std::shared_ptr<rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::HandBrakeReport, std::allocator<void>>> handbrake_status_pub;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Int16, std::allocator<void>>> headlights_status_pub;
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped, std::allocator<void>>> twist_pub;

    std::shared_ptr<rclcpp::Publisher<attack_ethernet_msgs::msg::EthernetAttackStatus, std::allocator<void>>> status_pub;

    rclcpp::TimerBase::SharedPtr replay_timer;


    struct ReplayCmd
    {
        uint16_t target;
        unsigned int sec;
        unsigned int nanosec;
        std::function<void(void)> cb;
    };

    struct TimestampGreater
    {
        bool operator()(const ReplayCmd & l, const ReplayCmd & r) const
        {
            return l.sec > r.sec || (l.sec == r.sec && l.nanosec > r.nanosec);
        }
    };

    std::priority_queue<ReplayCmd, std::vector<ReplayCmd>, TimestampGreater> replay_queue;
    std::mutex replay_queue_lock;

    void handle_replay();

    template <typename M,
             std::enable_if_t<std::is_same_v<decltype(M::header), std_msgs::msg::Header>> * = nullptr>
    std::pair<unsigned int, unsigned int> get_timestamp(const M & msg)
    {
        return std::make_pair(msg.header.stamp.sec, msg.header.stamp.nanosec);
    }

    template <typename M,
             std::enable_if_t<std::is_same_v<decltype(M::header), std_msgs::msg::Header>> * = nullptr>
    void set_timestamp(M & msg, unsigned int sec, unsigned int nanosec)
    {
        msg.header.stamp.sec = sec;
        msg.header.stamp.nanosec = nanosec;
    }

    template <typename M,
             std::enable_if_t<std::is_same_v<decltype(M::stamp), builtin_interfaces::msg::Time>> * = nullptr>
    std::pair<unsigned int, unsigned int> get_timestamp(const M & msg)
    {
        return std::make_pair(msg.stamp.sec, msg.stamp.nanosec);
    }

    template <typename M,
             std::enable_if_t<std::is_same_v<decltype(M::stamp), builtin_interfaces::msg::Time>> * = nullptr>
    void set_timestamp(M & msg, unsigned int sec, unsigned int nanosec)
    {
        msg.stamp.sec = sec;
        msg.stamp.nanosec = nanosec;
    }

    template <typename M,
             std::enable_if_t<std::is_same_v<M, std_msgs::msg::Int16>> * = nullptr>
    std::pair<unsigned int, unsigned int> get_timestamp(const M &)
    {
        auto current_time = this->now();
        auto curr_sec = static_cast<unsigned int>(current_time.nanoseconds() / 1000000000l);
        auto curr_nanosec = static_cast<unsigned int>(current_time.nanoseconds() % 1000000000l);
        return std::make_pair(curr_sec, curr_nanosec);
    }

    template <typename M,
             std::enable_if_t<std::is_same_v<M, std_msgs::msg::Int16>> * = nullptr>
    void set_timestamp(M &, unsigned int, unsigned int)
    {
        // NOP
    }

    std::pair<unsigned int, unsigned int> add_timestamps(const std::pair<unsigned int, unsigned int> & a,
                                                         const std::pair<unsigned int, unsigned int> & b)
    {
        constexpr unsigned int ns_per_sec = 1000000000u;
        std::pair<unsigned int, unsigned int> sum;
        sum.first = a.first + b.first;
        sum.second = a.second + b.second;
        if (sum.second > ns_per_sec)
        {
            sum.first++;
            sum.second %= ns_per_sec;
        }
        return sum;
    }

    template <typename P, typename M>
    bool handle_switching(const uint16_t attack_id, const int attack_num, const P & pub, const M & msg)
    {
        std::lock_guard<std::mutex> config_guard(attack_config->config_access);

        auto timestamp = get_timestamp(msg);

        if ((attack_config->attack_target() & attack_id) == 0)
        {
            auto attack_status_msg = std::make_unique<attack_ethernet_msgs::msg::EthernetAttackStatus>();
            attack_status_msg->config = attack_config->get_data();
            attack_status_msg->header.stamp.sec = timestamp.first;
            attack_status_msg->header.stamp.nanosec = timestamp.second;

            std::lock_guard<std::mutex> pub_guard(pub_mutex[attack_num]);
            status_pub->publish(std::move(attack_status_msg));

            return true;
        }

        std::lock_guard<std::mutex> replay_guard(replay_queue_lock);

        using attack_ethernet_msgs::msg::EthernetAttackConfig;
        bool forward_message = true;

        const bool replay_en = (attack_config->attack_type() & EthernetAttackConfig::ATTACK_TYPE_REPLAY);
        const bool dos_en = (attack_config->attack_type() & EthernetAttackConfig::ATTACK_TYPE_DOS);

        if (replay_en && (attack_config->get_random_percent() < attack_config->replay_percent()))
        {
            auto count = attack_config->replay_count();
            auto delay_ms = attack_config->replay_delay();
            auto delay = std::make_pair<unsigned int, unsigned int>(delay_ms / 1000, (delay_ms % 1000) * 1000000);

            auto delayed_stamp = add_timestamps(timestamp, delay);
            while(count > 0)
            {
                auto replay = ReplayCmd{
                    attack_id,
                    delayed_stamp.first,
                    delayed_stamp.second,
                    [this, attack_num, msg, delayed_stamp, &pub]() {
                        auto new_msg = std::make_unique<M>(msg);
                        this->set_timestamp(*new_msg, delayed_stamp.first, delayed_stamp.second);
                        std::lock_guard<std::mutex> pub_guard(pub_mutex[attack_num]);
                        pub->publish(std::move(new_msg));
                    }
                };
                replay_queue.push(replay);
                delayed_stamp = add_timestamps(delayed_stamp, delay);

                --count;
            }
        }

        if (dos_en)
        {
            auto delay_ms = attack_config->dos_delay();
            auto drop_percent = attack_config->dos_drop_percent();
            auto count = attack_config->dos_count();

            if (attack_config->get_random_percent() < drop_percent)
            {
                forward_message = false;
            }
            else
            {
                if (delay_ms > 0)
                {
                    forward_message = false;
                    auto delay = std::make_pair<unsigned int, unsigned int>(delay_ms / 1000, (delay_ms % 1000) * 1000000);
                    auto delayed_stamp = add_timestamps(timestamp, delay);
                    auto replay = ReplayCmd{
                        attack_id,
                        delayed_stamp.first,
                        delayed_stamp.second,
                        [this, attack_num, msg, delayed_stamp, &pub]() {
                            auto new_msg = std::make_unique<M>(msg);
                            this->set_timestamp(*new_msg, delayed_stamp.first, delayed_stamp.second);
                            std::lock_guard<std::mutex> pub_guard(pub_mutex[attack_num]);
                            pub->publish(std::move(new_msg));
                        }
                    };
                    replay_queue.push(replay);
                }

                std::lock_guard<std::mutex> pub_guard(pub_mutex[attack_num]);
                while (count > 0)
                {
                    auto new_msg = std::make_unique<M>(msg);
                    pub->publish(std::move(new_msg));
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    --count;
                }
            }
        }

        auto attack_status_msg = std::make_unique<attack_ethernet_msgs::msg::EthernetAttackStatus>();
        attack_status_msg->config = attack_config->get_data();
        attack_status_msg->header.stamp.sec = timestamp.first;
        attack_status_msg->header.stamp.nanosec = timestamp.second;
        status_pub->publish(std::move(attack_status_msg));

        return forward_message;
    }
};

#endif
