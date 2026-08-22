#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>              
#include <tf2/LinearMath/Matrix3x3.h>               
#include <vector>
#include <cmath>
#include <algorithm>

class LaserScanMerger : public rclcpp::Node {
public:
    LaserScanMerger() : Node("laserscan_multi_merger") {
        // 🟢 إجبار العقدة على التزامن مع وقت جازيبو لمنع الـ Lag 🟢
        this->set_parameter(rclcpp::Parameter("use_sim_time", true));

        this->declare_parameter("destination_frame", "base_footprint");
        this->declare_parameter("scan_destination_topic", "/scan");
        this->declare_parameter("angle_min", -3.14159);
        this->declare_parameter("angle_max", 3.14159);
        this->declare_parameter("angle_increment", 0.01745);
        this->declare_parameter("scan_time", 0.1);
        this->declare_parameter("range_min", 0.3);
        this->declare_parameter("range_max", 12.0);

        destination_frame_ = this->get_parameter("destination_frame").as_string();
        std::string destination_topic = this->get_parameter("scan_destination_topic").as_string();

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(destination_topic, rclcpp::SensorDataQoS());
        
        subscription1_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "back_left_scan", rclcpp::SensorDataQoS(), std::bind(&LaserScanMerger::scan1_callback, this, std::placeholders::_1));
        
        subscription2_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "front_right_scan", rclcpp::SensorDataQoS(), std::bind(&LaserScanMerger::scan2_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Laser Merger initialized flawlessly with Exact Time Sync!");
    }

private:
    void scan1_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) { 
        last_scan1_ = msg; 
        try_merge();
    }
    void scan2_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) { 
        last_scan2_ = msg; 
        try_merge();
    }

    void try_merge() {
        if (!last_scan1_ || !last_scan2_) return;

        rclcpp::Time t1(last_scan1_->header.stamp);
        rclcpp::Time t2(last_scan2_->header.stamp);
        
        // 🟢 ندمج فقط إذا كانت القراءتين في نفس اللحظة الزمنية (منع تشوه الدوران) 🟢
        if (std::abs((t1 - t2).seconds()) > 0.02) {
            return;
        }

        auto merged_scan = std::make_unique<sensor_msgs::msg::LaserScan>();
        merged_scan->header.stamp = t1; // نستخدم وقت حقيقي موحد
        merged_scan->header.frame_id = destination_frame_;
        
        merged_scan->angle_min = this->get_parameter("angle_min").as_double();
        merged_scan->angle_max = this->get_parameter("angle_max").as_double();
        merged_scan->angle_increment = this->get_parameter("angle_increment").as_double();
        merged_scan->time_increment = 0.0;
        merged_scan->scan_time = this->get_parameter("scan_time").as_double();
        merged_scan->range_min = this->get_parameter("range_min").as_double();
        merged_scan->range_max = this->get_parameter("range_max").as_double();

        size_t bins = std::round((merged_scan->angle_max - merged_scan->angle_min) / merged_scan->angle_increment);
        merged_scan->ranges.assign(bins, std::numeric_limits<float>::infinity());

        project_into_vscan(last_scan1_, merged_scan);
        project_into_vscan(last_scan2_, merged_scan);

        publisher_->publish(std::move(merged_scan));
        
        // 🟢 تفريغ المتغيرات لضمان عدم استخدام قراءات قديمة مرتين 🟢
        last_scan1_ = nullptr;
        last_scan2_ = nullptr;
    }

    void project_into_vscan(const sensor_msgs::msg::LaserScan::SharedPtr& src, std::unique_ptr<sensor_msgs::msg::LaserScan>& dest) {
        geometry_msgs::msg::TransformStamped transformStamped;
        try {
            transformStamped = tf_buffer_->lookupTransform("base_link", src->header.frame_id, tf2::TimePointZero);
        } catch (tf2::TransformException &ex) {
            return;
        }

        double tx = transformStamped.transform.translation.x;
        double ty = transformStamped.transform.translation.y;
        
        tf2::Quaternion q(
            transformStamped.transform.rotation.x,
            transformStamped.transform.rotation.y,
            transformStamped.transform.rotation.z,
            transformStamped.transform.rotation.w
        );
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        double src_angle = src->angle_min;
        for (size_t i = 0; i < src->ranges.size(); ++i) {
            double r = src->ranges[i];
            
            if (std::isfinite(r) && r >= src->range_min && r <= src->range_max) {
                double lx = r * cos(src_angle);
                double ly = r * sin(src_angle);
                
                double dest_x = lx * cos(yaw) - ly * sin(yaw) + tx;
                double dest_y = lx * sin(yaw) + ly * cos(yaw) + ty;
                
                double global_angle = atan2(dest_y, dest_x);
                double global_range = sqrt(dest_x*dest_x + dest_y*dest_y);

                int dest_idx = std::round((global_angle - dest->angle_min) / dest->angle_increment);
                if (dest_idx >= 0 && dest_idx < (int)dest->ranges.size()) {
                    if (std::isinf(dest->ranges[dest_idx]) || global_range < dest->ranges[dest_idx]) {
                        dest->ranges[dest_idx] = global_range;
                    }
                }
            }
            src_angle += src->angle_increment;
        }
    }

    std::string destination_frame_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription1_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription2_;

    sensor_msgs::msg::LaserScan::SharedPtr last_scan1_;
    sensor_msgs::msg::LaserScan::SharedPtr last_scan2_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserScanMerger>());
    rclcpp::shutdown();
    return 0;
}