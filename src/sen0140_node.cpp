#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <unistd.h>

using namespace std::chrono_literals;

class SEN0140Node : public rclcpp::Node
{
public:
    SEN0140Node()
    : Node("sen0140_node")
    {
        publisher_ =
            this->create_publisher<sensor_msgs::msg::Imu>(
                "/imu/data_raw", 10);

        open_i2c_devices();

        configure_adxl345();
        configure_itg3200();

        // 400 Hz
        timer_ = this->create_wall_timer(
            2500us,
            std::bind(&SEN0140Node::read_and_publish, this));

        RCLCPP_INFO(
            this->get_logger(),
            "SEN0140 IMU node started");
    }

    ~SEN0140Node()
    {
        if (adxl_fd_ >= 0)
            close(adxl_fd_);

        if (itg_fd_ >= 0)
            close(itg_fd_);
    }

private:

    static constexpr uint8_t ADXL_ADDR = 0x53;
    static constexpr uint8_t ITG_ADDR  = 0x68;

    int adxl_fd_ = -1;
    int itg_fd_ = -1;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ----------------------------------------------------------
    // I2C helpers
    // ----------------------------------------------------------

    int open_device(
        const std::string &device,
        uint8_t address)
    {
        int fd = open(device.c_str(), O_RDWR);

        if (fd < 0)
        {
            throw std::runtime_error(
                "Failed to open " + device);
        }

        if (ioctl(fd, I2C_SLAVE, address) < 0)
        {
            close(fd);

            throw std::runtime_error(
                "Failed to select I2C slave");
        }

        return fd;
    }

    void write_register(
        int fd,
        uint8_t reg,
        uint8_t value)
    {
        uint8_t buffer[2] = {
            reg,
            value
        };

        if (write(fd, buffer, 2) != 2)
        {
            throw std::runtime_error(
                "Failed to write I2C register");
        }
    }

    void read_registers(
        int fd,
        uint8_t start_reg,
        uint8_t *buffer,
        size_t length)
    {
        if (write(fd, &start_reg, 1) != 1)
        {
            throw std::runtime_error(
                "Failed to select I2C register");
        }

        if (read(fd, buffer, length) !=
            static_cast<ssize_t>(length))
        {
            throw std::runtime_error(
                "Failed to read I2C registers");
        }
    }

    // ----------------------------------------------------------
    // Initialisation
    // ----------------------------------------------------------

    void open_i2c_devices()
    {
        const std::string bus = "/dev/i2c-1";

        adxl_fd_ =
            open_device(bus, ADXL_ADDR);

        itg_fd_ =
            open_device(bus, ITG_ADDR);
    }

    void configure_adxl345()
    {
        constexpr uint8_t BW_RATE     = 0x2C;
        constexpr uint8_t POWER_CTL   = 0x2D;
        constexpr uint8_t DATA_FORMAT = 0x31;

        // Standby while configuring
        write_register(adxl_fd_, POWER_CTL, 0x00);

        // 800 Hz output data rate
        write_register(adxl_fd_, BW_RATE, 0x0D);

        // Full resolution, ±16 g
        write_register(adxl_fd_, DATA_FORMAT, 0x0B);

        // Measurement mode
        write_register(adxl_fd_, POWER_CTL, 0x08);
    }

    void configure_itg3200()
    {
        constexpr uint8_t SMPLRT_DIV = 0x15;
        constexpr uint8_t DLPF_FS    = 0x16;
        constexpr uint8_t PWR_MGM    = 0x3E;

        // Use X gyro PLL as clock source
        write_register(itg_fd_, PWR_MGM, 0x01);

        // 1 kHz output
        write_register(itg_fd_, SMPLRT_DIV, 0x00);

        // FS_SEL = 3 -> ±2000 deg/s
        // DLPF_CFG = 1 -> 188 Hz bandwidth, 1 kHz internal sampling
        write_register(itg_fd_, DLPF_FS, 0x19);
    }

    // ----------------------------------------------------------
    // Sensor reads
    // ----------------------------------------------------------

    void read_accel(
        double &ax,
        double &ay,
        double &az)
    {
        constexpr uint8_t DATAX0 = 0x32;

        uint8_t data[6];

        read_registers(
            adxl_fd_,
            DATAX0,
            data,
            6);

        // ADXL345 is little endian
        int16_t raw_x =
            static_cast<int16_t>(
                (data[1] << 8) | data[0]);

        int16_t raw_y =
            static_cast<int16_t>(
                (data[3] << 8) | data[2]);

        int16_t raw_z =
            static_cast<int16_t>(
                (data[5] << 8) | data[4]);

        constexpr double G = 9.80665;
        constexpr double SCALE_G = 0.0039;

        ax = raw_x * SCALE_G * G;
        ay = raw_y * SCALE_G * G;
        az = raw_z * SCALE_G * G;
    }

    void read_gyro(
        double &gx,
        double &gy,
        double &gz)
    {
        constexpr uint8_t GYRO_XOUT_H = 0x1D;

        uint8_t data[6];

        read_registers(
            itg_fd_,
            GYRO_XOUT_H,
            data,
            6);

        // ITG-3200 is big endian
        int16_t raw_x =
            static_cast<int16_t>(
                (data[0] << 8) | data[1]);

        int16_t raw_y =
            static_cast<int16_t>(
                (data[2] << 8) | data[3]);

        int16_t raw_z =
            static_cast<int16_t>(
                (data[4] << 8) | data[5]);

        constexpr double LSB_PER_DEG_S = 14.375;
        constexpr double DEG_TO_RAD =
            M_PI / 180.0;

        gx =
            (raw_x / LSB_PER_DEG_S) *
            DEG_TO_RAD;

        gy =
            (raw_y / LSB_PER_DEG_S) *
            DEG_TO_RAD;

        gz =
            (raw_z / LSB_PER_DEG_S) *
            DEG_TO_RAD;
    }

    // ----------------------------------------------------------
    // ROS publish callback
    // ----------------------------------------------------------

    void read_and_publish()
    {
        double ax, ay, az;
        double gx, gy, gz;

        auto before = this->now();

        read_gyro(gx, gy, gz);
        read_accel(ax, ay, az);

        auto after = this->now();

        // Approximate acquisition time as midpoint of
        // the two host timestamps.
        int64_t midpoint_ns =
            (before.nanoseconds() +
             after.nanoseconds()) / 2;

        rclcpp::Time stamp(
            midpoint_ns,
            this->get_clock()->get_clock_type());

        sensor_msgs::msg::Imu msg;

        msg.header.stamp = stamp;
        msg.header.frame_id = "imu_link";

        msg.angular_velocity.x = gx;
        msg.angular_velocity.y = gy;
        msg.angular_velocity.z = gz;

        msg.linear_acceleration.x = ax;
        msg.linear_acceleration.y = ay;
        msg.linear_acceleration.z = az;

        // We do not provide an orientation estimate.
        msg.orientation_covariance[0] = -1.0;

        publisher_->publish(msg);
    }
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<SEN0140Node>());

    rclcpp::shutdown();

    return 0;
}