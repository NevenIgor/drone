#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <cmath>
#include <vector>
#include <thread>
#include <chrono>
#include <errno.h>
#include <cstring>

class ICM20948Node : public rclcpp::Node {
public:
    ICM20948Node() : Node("icm20948_node"), i2c_fd_(-1), addr_(0x68) {
        declare_parameter<std::string>("i2c_bus", "/dev/i2c-1");
        declare_parameter<int>("i2c_address", 0x68);
        declare_parameter<double>("accel_scale", 8.0);
        declare_parameter<double>("gyro_scale", 1000.0);

        std::string bus = get_parameter("i2c_bus").as_string();
        addr_ = get_parameter("i2c_address").as_int();
        accel_scale_ = get_parameter("accel_scale").as_double();
        gyro_scale_ = get_parameter("gyro_scale").as_double();

        i2c_fd_ = open(bus.c_str(), O_RDWR);
        if (i2c_fd_ < 0) {
            RCLCPP_FATAL(get_logger(), "Не удалось открыть %s: %s", bus.c_str(), strerror(errno));
            rclcpp::shutdown(); return;
        }

        // I2C_SLAVE не обязателен при I2C_RDWR, но оставим для совместимости
        if (ioctl(i2c_fd_, I2C_SLAVE, addr_) < 0) {
            RCLCPP_WARN(get_logger(), "I2C_SLAVE ioctl failed (не критично)");
        }

        if (!init_sensor()) {
            RCLCPP_FATAL(get_logger(), "Критическая ошибка инициализации ICM-20948");
            rclcpp::shutdown(); return;
        }

        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(5),
                                         std::bind(&ICM20948Node::read_and_publish, this));
        RCLCPP_INFO(get_logger(), "✅ ICM-20948 запущен. Публикация на /imu/data @ 200Hz");
    }

    ~ICM20948Node() { if (i2c_fd_ >= 0) close(i2c_fd_); }

private:
    bool write_reg(uint8_t reg, uint8_t val) {
        uint8_t buf[2] = {reg, val};
        return write(i2c_fd_, buf, 2) == 2;
    }

    uint8_t read_reg(uint8_t reg) {
        uint8_t val = 0;
        if (write(i2c_fd_, &reg, 1) == 1 && read(i2c_fd_, &val, 1) == 1) return val;
        return 0;
    }

    // ✅ Надёжное блочное чтение с Repeated Start через I2C_RDWR
    int read_block(uint8_t reg, uint8_t *buf, size_t len) {
        struct i2c_msg msgs[2];
        struct i2c_rdwr_ioctl_data rdwr;

        msgs[0].addr = addr_;
        msgs[0].flags = 0;           // Запись
        msgs[0].len = 1;
        msgs[0].buf = &reg;

        msgs[1].addr = addr_;
        msgs[1].flags = I2C_M_RD;    // Чтение
        msgs[1].len = len;
        msgs[1].buf = buf;

        rdwr.msgs = msgs;
        rdwr.nmsgs = 2;

        if (ioctl(i2c_fd_, I2C_RDWR, &rdwr) < 0) return -1;
        return len;
    }

    void switch_bank(uint8_t bank) {
        write_reg(0x7F, bank & 0x03);
        std::this_thread::sleep_for(std::chrono::microseconds(200));
    }

    bool init_sensor() {
        RCLCPP_INFO(get_logger(), "Инициализация ICM-20948...");
        if (!write_reg(0x06, 0x80)) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        write_reg(0x06, 0x01);
        write_reg(0x07, 0x00);
        switch_bank(2);
        write_reg(0x14, 0x10); // ±8g
        write_reg(0x01, 0x02); // ±1000 dps
        switch_bank(0);
        uint8_t whoami = read_reg(0x00);
        if (whoami != 0xEA) {
            RCLCPP_ERROR(get_logger(), "WHO_AM_I mismatch: 0x%02X (ожидается 0xEA)", whoami);
            return false;
        }
        RCLCPP_INFO(get_logger(), "✅ Сенсор успешно инициализирован");
        return true;
    }

    void read_and_publish() {
        switch_bank(2);
        uint8_t buf[14];
        // 0x2D = ACCEL_XOUT_H, читаем 14 байт (Accel+Temp+Gyro)
        if (read_block(0x2D, buf, 14) != 14) {
            RCLCPP_WARN_THROTTLE(get_logger(), *this, 2000, "Ошибка чтения данных IMU");
            return;
        }

        auto parse16 = [&](size_t i) -> int16_t {
            return static_cast<int16_t>((buf[i] << 8) | buf[i+1]);
        };

        int16_t ax = parse16(0), ay = parse16(2), az = parse16(4);
        int16_t gx = parse16(8), gy = parse16(10), gz = parse16(12);

        double a_scale = accel_scale_ / 32768.0 * 9.80665;
        double g_scale = gyro_scale_ / 32768.0 * (M_PI / 180.0);

        sensor_msgs::msg::Imu msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "imu_link";
        msg.linear_acceleration.x = ax * a_scale;
        msg.linear_acceleration.y = ay * a_scale;
        msg.linear_acceleration.z = az * a_scale;
        msg.angular_velocity.x = gx * g_scale;
        msg.angular_velocity.y = gy * g_scale;
        msg.angular_velocity.z = gz * g_scale;
        msg.orientation_covariance[0] = -1.0;
        msg.linear_acceleration_covariance[0] = 0.01;
        msg.angular_velocity_covariance[0] = 0.001;

        imu_pub_->publish(msg);
    }

    int i2c_fd_;
    int addr_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    double accel_scale_, gyro_scale_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ICM20948Node>());
    rclcpp::shutdown();
    return 0;
}
