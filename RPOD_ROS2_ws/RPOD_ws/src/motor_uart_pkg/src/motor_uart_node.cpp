#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>
#include <deque>

using std::placeholders::_1;

class UARTNode : public rclcpp::Node
{
public:
    UARTNode() : Node("uart_node")
    {
        uart_fd_ = openUART("/dev/ttyTHS1");

        rpm_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/adjusted_rpms",
            10,
            std::bind(&UARTNode::rpmCallback, this, _1));

        // NEW: live PID gain tuning, forwarded to STM32 as MSG_TUNE_PID (0x05).
        // Expects exactly 3 values: [kp, ki, kd]. The firmware clamps these
        // to TUNE_KP_MAX/TUNE_KI_MAX/TUNE_KD_MAX and silently drops the
        // update if the robot is currently moving (see ControlLoop_SetGains()
        // in control_loop.c) -- this node does not need to know or enforce
        // that itself, it just forwards the frame.
        tune_pid_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/tune_pid",
            10,
            std::bind(&UARTNode::tunePidCallback, this, _1));

        enc_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
            "/encoder_ticks", 10);

        rx_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&UARTNode::readUART, this));

        RCLCPP_INFO(this->get_logger(), "UART Node Started (TX + RX on shared fd)");
    }

    ~UARTNode()
    {
        if (uart_fd_ >= 0) close(uart_fd_);
    }

private:
    // -------- Protocol constants (Base / Chaser controller) --------
    static constexpr uint8_t SYNC1 = 0xAA;
    static constexpr uint8_t SYNC2 = 0x55;
    static constexpr uint8_t SYNC3 = 0xA5;

    static constexpr uint8_t CONTROL_CMD_TYPE = 0x01;
    static constexpr uint8_t CONTROL_CMD_LEN  = 0x09; // 4x int16 + 1x uint8

    static constexpr uint8_t ACTUATOR_FB_TYPE = 0x02;
    static constexpr uint8_t ACTUATOR_FB_LEN  = 0x25; // 37 bytes

    static constexpr uint8_t TUNE_PID_TYPE = 0x05;
    static constexpr uint8_t TUNE_PID_LEN  = 0x0C; // 3x float32 = 12 bytes

    static constexpr uint8_t CONTROL_MODE = 0; // 0=PID, 1=PWM

    // Frame sizes
    static constexpr size_t TX_FRAME_SIZE       = 3 + 1 + 1 + CONTROL_CMD_LEN + 1;   // 15
    static constexpr size_t RX_FRAME_SIZE       = 3 + 1 + 1 + ACTUATOR_FB_LEN + 1;   // 43
    static constexpr size_t TUNE_PID_FRAME_SIZE = 3 + 1 + 1 + TUNE_PID_LEN + 1;      // 18

    int uart_fd_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr rpm_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr tune_pid_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr enc_pub_;
    rclcpp::TimerBase::SharedPtr rx_timer_;

    std::deque<uint8_t> rx_buffer_;

    //--------------------------------------------//
    int openUART(const char *port)
    {
        int fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open UART: %s", port);
            return -1;
        }

        struct termios tty;
        std::memset(&tty, 0, sizeof(tty));

        if (tcgetattr(fd, &tty) != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "tcgetattr failed");
            close(fd);
            return -1;
        }

        cfsetispeed(&tty, B921600);
        cfsetospeed(&tty, B921600);

        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        tty.c_iflag = 0;
        tty.c_oflag = 0;
        tty.c_lflag = 0;

        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 0;

        if (tcsetattr(fd, TCSANOW, &tty) != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "tcsetattr failed");
            close(fd);
            return -1;
        }

        tcflush(fd, TCIOFLUSH);

        RCLCPP_INFO(this->get_logger(), "UART opened: %s at 921600 baud", port);
        return fd;
    }

    //--------------------------------------------//
    // CRC-8, poly 0x07, over TYPE + LEN + PAYLOAD only
    uint8_t calculateCRC8(const uint8_t *data, size_t length)
    {
        uint8_t crc = 0x00;
        for (size_t i = 0; i < length; ++i)
        {
            crc ^= data[i];
            for (int bit = 0; bit < 8; ++bit)
            {
                crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x07)
                                    : static_cast<uint8_t>(crc << 1);
            }
        }
        return crc;
    }

    void appendInt16LE(std::vector<uint8_t> &packet, int16_t value)
    {
        uint16_t raw = static_cast<uint16_t>(value);
        packet.push_back(static_cast<uint8_t>(raw & 0xFF));
        packet.push_back(static_cast<uint8_t>((raw >> 8) & 0xFF));
    }

    // NEW: append a float32 in the STM32's native (little-endian) byte order.
    // Both the Jetson/x86 host and the STM32F4 target are little-endian, so a
    // raw memcpy of the float's bit pattern round-trips correctly without a
    // separate serialization step. This does NOT convert IEEE-754 layout --
    // it assumes both ends already agree on that, which they do here.
    void appendFloatLE(std::vector<uint8_t> &packet, float value)
    {
        uint8_t raw[4];
        std::memcpy(raw, &value, sizeof(float));
        packet.insert(packet.end(), raw, raw + 4);
    }

    int16_t readInt16LE(const uint8_t *p)
    {
        uint16_t raw = static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
        return static_cast<int16_t>(raw);
    }

    int32_t readInt32LE(const uint8_t *p)
    {
        uint32_t raw = static_cast<uint32_t>(p[0])
                      | (static_cast<uint32_t>(p[1]) << 8)
                      | (static_cast<uint32_t>(p[2]) << 16)
                      | (static_cast<uint32_t>(p[3]) << 24);
        return static_cast<int32_t>(raw);
    }

    uint32_t readUint32LE(const uint8_t *p)
    {
        return static_cast<uint32_t>(p[0])
             | (static_cast<uint32_t>(p[1]) << 8)
             | (static_cast<uint32_t>(p[2]) << 16)
             | (static_cast<uint32_t>(p[3]) << 24);
    }

    //--------------------------------------------//
    // TX: control command (GNC -> UART node -> STM32)
    //--------------------------------------------//
    void rpmCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() != 4)
        {
            RCLCPP_WARN(this->get_logger(),
                "Expected exactly 4 RPM values, received %zu.", msg->data.size());
            return;
        }
        sendControlCommand(msg->data);
    }

    void sendControlCommand(const std::vector<double> &rpm)
    {
        if (uart_fd_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "UART is not open.");
            return;
        }

        // Frame: SYNC1 SYNC2 SYNC3 TYPE LEN [9-byte payload] CRC = 15 bytes
        std::vector<uint8_t> packet;
        packet.reserve(TX_FRAME_SIZE);

        packet.push_back(SYNC1);
        packet.push_back(SYNC2);
        packet.push_back(SYNC3);
        packet.push_back(CONTROL_CMD_TYPE);
        packet.push_back(CONTROL_CMD_LEN);

        // Payload matches Python struct "<hhhhB":
        // 4x int16_t (RPM * 10), little-endian, then 1x uint8_t mode
        for (int i = 0; i < 4; ++i)
        {
            double scaled = std::round(rpm[i] * 10.0);
            scaled = std::clamp(scaled, -32768.0, 32767.0);
            appendInt16LE(packet, static_cast<int16_t>(scaled));
        }
        packet.push_back(CONTROL_MODE);

        // CRC over TYPE + LEN + PAYLOAD = 11 bytes (indices 3..13)
        uint8_t crc = calculateCRC8(&packet[3], 11);
        packet.push_back(crc);

        ssize_t bytes_sent = write(uart_fd_, packet.data(), packet.size());

        if (bytes_sent < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to send UART packet.");
            return;
        }
        if (static_cast<size_t>(bytes_sent) != packet.size())
        {
            RCLCPP_WARN(this->get_logger(),
                "Partial UART write: sent %zd of %zu bytes.", bytes_sent, packet.size());
            return;
        }

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "Sent Base CMD | RPM: [%.2f, %.2f, %.2f, %.2f] | mode=%d | bytes=%zd",
            rpm[0], rpm[1], rpm[2], rpm[3], CONTROL_MODE, bytes_sent);
    }

    //--------------------------------------------//
    // TX: live PID gain update (GNC/tuning tool -> UART node -> STM32)
    //--------------------------------------------//
    void tunePidCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() != 3)
        {
            RCLCPP_WARN(this->get_logger(),
                "Expected exactly 3 values [kp, ki, kd] on /tune_pid, received %zu.",
                msg->data.size());
            return;
        }
        sendTunePID(msg->data[0], msg->data[1], msg->data[2]);
    }

    void sendTunePID(double kp, double ki, double kd)
    {
        if (uart_fd_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "UART is not open.");
            return;
        }

        // Frame: SYNC1 SYNC2 SYNC3 TYPE LEN [12-byte payload] CRC = 18 bytes
        std::vector<uint8_t> packet;
        packet.reserve(TUNE_PID_FRAME_SIZE);

        packet.push_back(SYNC1);
        packet.push_back(SYNC2);
        packet.push_back(SYNC3);
        packet.push_back(TUNE_PID_TYPE);
        packet.push_back(TUNE_PID_LEN);

        // Payload matches firmware's TunePIDCmd_t: 3x float32, little-endian.
        // No clamping done here -- the firmware clamps to TUNE_KP_MAX/
        // TUNE_KI_MAX/TUNE_KD_MAX and drops the update entirely if the robot
        // isn't stationary. This node just forwards whatever it's given.
        appendFloatLE(packet, static_cast<float>(kp));
        appendFloatLE(packet, static_cast<float>(ki));
        appendFloatLE(packet, static_cast<float>(kd));

        // CRC over TYPE + LEN + PAYLOAD = 14 bytes (indices 3..16)
        uint8_t crc = calculateCRC8(&packet[3], 2 + TUNE_PID_LEN);
        packet.push_back(crc);

        ssize_t bytes_sent = write(uart_fd_, packet.data(), packet.size());

        if (bytes_sent < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to send PID tune packet.");
            return;
        }
        if (static_cast<size_t>(bytes_sent) != packet.size())
        {
            RCLCPP_WARN(this->get_logger(),
                "Partial UART write on PID tune: sent %zd of %zu bytes.",
                bytes_sent, packet.size());
            return;
        }

        RCLCPP_INFO(this->get_logger(),
            "Sent Tune PID | kp=%.3f ki=%.3f kd=%.3f | bytes=%zd",
            kp, ki, kd, bytes_sent);
    }

    //--------------------------------------------//
    // RX: actuator feedback (0x02) from STM32, buffered + framed
    //--------------------------------------------//
    void readUART()
    {
        if (uart_fd_ < 0) return;

        uint8_t buf[256];
        ssize_t n = read(uart_fd_, buf, sizeof(buf));
        if (n > 0)
        {
            rx_buffer_.insert(rx_buffer_.end(), buf, buf + n);
        }

        while (true)
        {
            // Drop bytes until SYNC1, SYNC2, SYNC3 found at front of buffer
            while (rx_buffer_.size() >= 3 &&
                   !(rx_buffer_[0] == SYNC1 && rx_buffer_[1] == SYNC2 && rx_buffer_[2] == SYNC3))
            {
                rx_buffer_.pop_front();
            }

            if (rx_buffer_.size() < RX_FRAME_SIZE) break; // wait for more bytes

            uint8_t type = rx_buffer_[3];
            uint8_t len  = rx_buffer_[4];

            if (type != ACTUATOR_FB_TYPE || len != ACTUATOR_FB_LEN)
            {
                // Not the frame we expect here; resync by dropping the false SYNC1
                rx_buffer_.pop_front();
                continue;
            }

            std::vector<uint8_t> frame(rx_buffer_.begin(), rx_buffer_.begin() + RX_FRAME_SIZE);

            // CRC over TYPE + LEN + PAYLOAD = 39 bytes (indices 3..41)
            uint8_t expected_crc = calculateCRC8(&frame[3], 2 + ACTUATOR_FB_LEN);
            uint8_t received_crc = frame[RX_FRAME_SIZE - 1];

            if (expected_crc != received_crc)
            {
                RCLCPP_WARN(this->get_logger(), "CRC mismatch on Actuator FB, dropping frame.");
                rx_buffer_.pop_front(); // resync
                continue;
            }

            parseActuatorFeedback(frame);

            rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + RX_FRAME_SIZE);
        }
    }

    void parseActuatorFeedback(const std::vector<uint8_t> &frame)
    {
        // Payload starts at index 5, 37 bytes total.
        // Layout: 4x int16 RPM, 4x int32 encoder, 4x int16 PWM,
        //         1x uint32 timestamp, 1x uint8 status
        const uint8_t *p = &frame[5];

        int16_t rpm[4];
        for (int i = 0; i < 4; ++i) rpm[i] = readInt16LE(p + i * 2);
        p += 8;

        int32_t encoder[4];
        for (int i = 0; i < 4; ++i) encoder[i] = readInt32LE(p + i * 4);
        p += 16;

        int16_t pwm[4];
        for (int i = 0; i < 4; ++i) pwm[i] = readInt16LE(p + i * 2);
        p += 8;

        uint32_t timestamp_ms = readUint32LE(p);
        p += 4;

        uint8_t sys_status = p[0];

        // Publish encoder ticks for downstream use (e.g. GNC/estimation)
        std_msgs::msg::Int32MultiArray msg;
        msg.data.assign(encoder, encoder + 4);
        enc_pub_->publish(msg);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "FB | RPM:[%d %d %d %d] Enc:[%d %d %d %d] PWM:[%d %d %d %d] t=%u status=%u",
            rpm[0], rpm[1], rpm[2], rpm[3],
            encoder[0], encoder[1], encoder[2], encoder[3],
            pwm[0], pwm[1], pwm[2], pwm[3],
            timestamp_ms, sys_status);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UARTNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
