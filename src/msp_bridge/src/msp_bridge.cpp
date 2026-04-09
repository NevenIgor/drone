#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <cmath>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <vector>
#include <cstring>
#include <cstdint>
#include <chrono>

class GPSBridgeNode : public rclcpp::Node {
public:
    GPSBridgeNode() : Node("gps_bridge"), 
                      uart_fd_(-1),
                      kp_(0.5),
                      base_lat_(0), base_lon_(0), base_alt_(0),
                      true_x_(0), true_y_(0), true_z_(0),
                      target_x_(0), target_y_(0), target_z_(0),
                      mode_("MANUAL") {
        RCLCPP_INFO(get_logger(), "🛰 GPS Bridge (Drift Mode) starting...");

        declare_parameter<std::string>("uart_port", "/dev/ttyUSB0");
        declare_parameter<int>("baudrate", 115200);
        declare_parameter<double>("base_lat", 55.7558);
        declare_parameter<double>("base_lon", 37.6173);
        declare_parameter<double>("base_alt", 150.0);
        declare_parameter<double>("kp", 0.5);

        std::string port = get_parameter("uart_port").as_string();
        int baud = get_parameter("baudrate").as_int();
        base_lat_ = get_parameter("base_lat").as_double();
        base_lon_ = get_parameter("base_lon").as_double();
        base_alt_ = get_parameter("base_alt").as_double();
        kp_ = get_parameter("kp").as_double();

        if (!init_uart(port, baud)) {
            RCLCPP_FATAL(get_logger(), "❌ Failed to open UART");
            rclcpp::shutdown();
            return;
        }

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&GPSBridgeNode::odom_callback, this, std::placeholders::_1)
        );

        target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mission/target", 10,
            std::bind(&GPSBridgeNode::target_callback, this, std::placeholders::_1)
        );

        mode_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/mission/mode", 10,
            std::bind(&GPSBridgeNode::mode_callback, this, std::placeholders::_1)
        );

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&GPSBridgeNode::timer_callback, this)
        );

        RCLCPP_INFO(get_logger(), "✅ Drift Mode ready. Kp=%.2f", kp_);
        RCLCPP_INFO(get_logger(), "📍 Base: lat=%.6f, lon=%.6f, alt=%.1fm", base_lat_, base_lon_, base_alt_);
    }

    ~GPSBridgeNode() {
        if (uart_fd_ >= 0) close(uart_fd_);
    }

private:
    bool init_uart(const std::string& port, int baud) {
        uart_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (uart_fd_ < 0) {
            RCLCPP_ERROR(get_logger(), "open() failed: %s", strerror(errno));
            return false;
        }

        struct termios tty{};
        if (tcgetattr(uart_fd_, &tty) != 0) return false;

        speed_t speed;
        switch (baud) {
            case 9600: speed = B9600; break;
            case 19200: speed = B19200; break;
            case 57600: speed = B57600; break;
            case 115200: speed = B115200; break;
            default: speed = B115200; break;
        }
        cfsetospeed(&tty, speed);
        cfsetispeed(&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;
        tty.c_cflag |= CLOCAL | CREAD;
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_lflag = 0;
        tty.c_oflag = 0;
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 1;

        if (tcsetattr(uart_fd_, TCSANOW, &tty) != 0) return false;
        tcflush(uart_fd_, TCIOFLUSH);
        return true;
    }

    void send_ublox_nav_pvt(int32_t lat, int32_t lon, int32_t alt_mm, uint8_t fix_type = 3) {
        std::vector<uint8_t> payload;
        
        uint32_t itow = 1000;
        payload.push_back(itow & 0xFF);
        payload.push_back((itow >> 8) & 0xFF);
        payload.push_back((itow >> 16) & 0xFF);
        payload.push_back((itow >> 24) & 0xFF);
        
        uint16_t year = 2024;
        payload.push_back(year & 0xFF);
        payload.push_back((year >> 8) & 0xFF);
        
        payload.push_back(1);
        payload.push_back(1);
        payload.push_back(12);
        payload.push_back(0);
        payload.push_back(0);
        
        payload.push_back(0x04);
        payload.push_back(0); payload.push_back(0); payload.push_back(0); payload.push_back(0);
        payload.push_back(0); payload.push_back(0); payload.push_back(0); payload.push_back(0);
        
        payload.push_back(fix_type);
        payload.push_back(0x01);
        payload.push_back(0);
        payload.push_back(10);
        
        payload.push_back(lon & 0xFF);
        payload.push_back((lon >> 8) & 0xFF);
        payload.push_back((lon >> 16) & 0xFF);
        payload.push_back((lon >> 24) & 0xFF);
        
        payload.push_back(lat & 0xFF);
        payload.push_back((lat >> 8) & 0xFF);
        payload.push_back((lat >> 16) & 0xFF);
        payload.push_back((lat >> 24) & 0xFF);
        
        payload.push_back(alt_mm & 0xFF);
        payload.push_back((alt_mm >> 8) & 0xFF);
        payload.push_back((alt_mm >> 16) & 0xFF);
        payload.push_back((alt_mm >> 24) & 0xFF);
        
        payload.push_back(alt_mm & 0xFF);
        payload.push_back((alt_mm >> 8) & 0xFF);
        payload.push_back((alt_mm >> 16) & 0xFF);
        payload.push_back((alt_mm >> 24) & 0xFF);
        
        uint32_t hacc = 1000;
        payload.push_back(hacc & 0xFF);
        payload.push_back((hacc >> 8) & 0xFF);
        payload.push_back((hacc >> 16) & 0xFF);
        payload.push_back((hacc >> 24) & 0xFF);
        
        uint32_t vacc = 1000;
        payload.push_back(vacc & 0xFF);
        payload.push_back((vacc >> 8) & 0xFF);
        payload.push_back((vacc >> 16) & 0xFF);
        payload.push_back((vacc >> 24) & 0xFF);

        std::vector<uint8_t> ubx;
        ubx.push_back(0xB5);
        ubx.push_back(0x62);
        ubx.push_back(0x01);
        ubx.push_back(0x07);
        
        uint16_t len = static_cast<uint16_t>(payload.size());
        ubx.push_back(len & 0xFF);
        ubx.push_back((len >> 8) & 0xFF);
        
        ubx.insert(ubx.end(), payload.begin(), payload.end());
        
        uint8_t ck_a = 0, ck_b = 0;
        for (size_t i = 2; i < ubx.size(); ++i) {
            ck_a += ubx[i];
            ck_b += ck_a;
        }
        ubx.push_back(ck_a);
        ubx.push_back(ck_b);

        write(uart_fd_, ubx.data(), ubx.size());
    }

    void compute_and_send_fake_position() {
        if (mode_ == "MANUAL") {
            send_real_position(true_x_, true_y_, true_z_);
            return;
        }

        float error_x = target_x_ - true_x_;
        float error_y = target_y_ - true_y_;
        float error_z = target_z_ - true_z_;

        float fake_x = true_x_ - (kp_ * error_x);
        float fake_y = true_y_ - (kp_ * error_y);
        float fake_z = true_z_ - (kp_ * error_z);

        double lat_offset = (-fake_y) / 111320.0;
        double lon_offset = fake_x / (111320.0 * cos(base_lat_ * M_PI / 180.0));
        
        int32_t lat = static_cast<int32_t>((base_lat_ + lat_offset) * 10000000.0);
        int32_t lon = static_cast<int32_t>((base_lon_ + lon_offset) * 10000000.0);
        int32_t alt_mm = static_cast<int32_t>((base_alt_ - fake_z) * 1000.0);

        send_ublox_nav_pvt(lat, lon, alt_mm);

        if (error_x > 0.1 || error_y > 0.1) {
            RCLCPP_INFO(get_logger(), 
                "🎯 Target: (%.2f, %.2f, %.2f) | True: (%.2f, %.2f, %.2f) | Error: (%.2f, %.2f)",
                target_x_, target_y_, target_z_,
                true_x_, true_y_, true_z_,
                error_x, error_y);
        }
    }

    void send_real_position(float x, float y, float z) {
        double lat_offset = (-y) / 111320.0;
        double lon_offset = x / (111320.0 * cos(base_lat_ * M_PI / 180.0));
        
        int32_t lat = static_cast<int32_t>((base_lat_ + lat_offset) * 10000000.0);
        int32_t lon = static_cast<int32_t>((base_lon_ + lon_offset) * 10000000.0);
        int32_t alt_mm = static_cast<int32_t>((base_alt_ - z) * 1000.0);

        send_ublox_nav_pvt(lat, lon, alt_mm);
    }

    void timer_callback() {
        compute_and_send_fake_position();
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        true_x_ = msg->pose.pose.position.x;
        true_y_ = msg->pose.pose.position.y;
        true_z_ = msg->pose.pose.position.z;
    }

    void target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        target_x_ = msg->pose.position.x;
        target_y_ = msg->pose.position.y;
        target_z_ = msg->pose.position.z;
        RCLCPP_INFO(get_logger(), "🎯 Новая цель: (%.2f, %.2f, %.2f)", target_x_, target_y_, target_z_);
    }

    void mode_callback(const std_msgs::msg::String::SharedPtr msg) {
        mode_ = msg->data;
        RCLCPP_INFO(get_logger(), "🔀 Режим: %s", mode_.c_str());
    }

    // === Члены класса (порядок важен для инициализации!) ===
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    int uart_fd_;
    double kp_;
    double base_lat_, base_lon_, base_alt_;
    float true_x_, true_y_, true_z_;
    float target_x_, target_y_, target_z_;
    std::string mode_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GPSBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
