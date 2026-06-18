#include <rclcpp/rclcpp.hpp>
#include <librealsense2/rs.hpp>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <thread>
#include <cstring>
#include <atomic>
#include <algorithm>

#pragma pack(push, 1)
struct UDPChunkHeader {
    uint32_t frame_id;
    uint8_t stream_type; // 0 = Color, 1 = Depth, 2 = IR
    uint32_t total_size;
    uint16_t chunk_index;
    uint16_t total_chunks;
    uint32_t chunk_size;
    uint32_t width;
    uint32_t height;
    uint32_t format; 
};
#pragma pack(pop)

rs2_format getFormat(const std::string& fmt) {
    if (fmt == "rgb8") return RS2_FORMAT_RGB8;
    if (fmt == "bgr8") return RS2_FORMAT_BGR8;
    if (fmt == "z16") return RS2_FORMAT_Z16;
    if (fmt == "y8") return RS2_FORMAT_Y8;
    if (fmt == "y16") return RS2_FORMAT_Y16;
    return RS2_FORMAT_ANY;
}

class RealsenseUDPStreamer : public rclcpp::Node {
public:
    RealsenseUDPStreamer() : Node("realsense_udp_streamer") {
        // Network
        this->declare_parameter<std::string>("target_ip", "127.0.0.1");
        this->declare_parameter<int>("target_port", 8080);
        this->declare_parameter<int>("chunk_size", 60000);

        // Device
        this->declare_parameter<std::string>("device_id", "");

        // Streams enabled
        this->declare_parameter<bool>("enable_rgb", true);
        this->declare_parameter<bool>("enable_depth", true);
        this->declare_parameter<bool>("enable_ir", false);

        // Stream Configs
        this->declare_parameter<int>("rgb_width", 640);
        this->declare_parameter<int>("rgb_height", 480);
        this->declare_parameter<int>("rgb_fps", 30);
        this->declare_parameter<std::string>("rgb_format", "rgb8");

        this->declare_parameter<int>("depth_width", 640);
        this->declare_parameter<int>("depth_height", 480);
        this->declare_parameter<int>("depth_fps", 30);
        this->declare_parameter<std::string>("depth_format", "z16");

        this->declare_parameter<int>("ir_width", 640);
        this->declare_parameter<int>("ir_height", 480);
        this->declare_parameter<int>("ir_fps", 30);
        this->declare_parameter<std::string>("ir_format", "y8");

        // Load configs
        target_ip_ = this->get_parameter("target_ip").as_string();
        target_port_ = this->get_parameter("target_port").as_int();
        chunk_size_ = this->get_parameter("chunk_size").as_int();

        discoverDevices();

        sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Error creating socket");
            return;
        }

        memset(&target_addr_, 0, sizeof(target_addr_));
        target_addr_.sin_family = AF_INET;
        target_addr_.sin_port = htons(target_port_);
        if (inet_pton(AF_INET, target_ip_.c_str(), &target_addr_.sin_addr) <= 0) {
            RCLCPP_ERROR(this->get_logger(), "Invalid address / Address not supported");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Streaming to %s:%d", target_ip_.c_str(), target_port_);

        // Configure pipeline
        rs2::config cfg;
        std::string device_id = this->get_parameter("device_id").as_string();
        if (!device_id.empty()) {
            RCLCPP_INFO(this->get_logger(), "Requesting specific device_id: %s", device_id.c_str());
            cfg.enable_device(device_id);
        }

        if (this->get_parameter("enable_rgb").as_bool()) {
            cfg.enable_stream(RS2_STREAM_COLOR,
                              this->get_parameter("rgb_width").as_int(),
                              this->get_parameter("rgb_height").as_int(),
                              getFormat(this->get_parameter("rgb_format").as_string()),
                              this->get_parameter("rgb_fps").as_int());
        }

        if (this->get_parameter("enable_depth").as_bool()) {
            cfg.enable_stream(RS2_STREAM_DEPTH,
                              this->get_parameter("depth_width").as_int(),
                              this->get_parameter("depth_height").as_int(),
                              getFormat(this->get_parameter("depth_format").as_string()),
                              this->get_parameter("depth_fps").as_int());
        }

        if (this->get_parameter("enable_ir").as_bool()) {
            cfg.enable_stream(RS2_STREAM_INFRARED,
                              this->get_parameter("ir_width").as_int(),
                              this->get_parameter("ir_height").as_int(),
                              getFormat(this->get_parameter("ir_format").as_string()),
                              this->get_parameter("ir_fps").as_int());
        }

        try {
            profile_ = pipe_.start(cfg);
            
            // HARD-LOCK TO 30 FPS: 
            // By default, RealSense firmware lowers the framerate in low-light conditions to increase exposure time.
            // Disabling 'AUTO_EXPOSURE_PRIORITY' forces the camera to maintain strict 30 FPS even if the image gets dark.
            auto dev = profile_.get_device();
            for (auto& sensor : dev.query_sensors()) {
                if (sensor.supports(RS2_OPTION_AUTO_EXPOSURE_PRIORITY)) {
                    sensor.set_option(RS2_OPTION_AUTO_EXPOSURE_PRIORITY, 0.0f);
                }
            }
            
            RCLCPP_INFO(this->get_logger(), "RealSense pipeline started successfully. Hardware locked to requested FPS.");
        } catch (const rs2::error & e) {
            RCLCPP_ERROR(this->get_logger(), "RealSense config error: %s", e.what());
            return;
        }

        running_ = true;
        stream_thread_ = std::thread(&RealsenseUDPStreamer::streamLoop, this);
    }

    ~RealsenseUDPStreamer() {
        running_ = false;
        if (stream_thread_.joinable()) {
            stream_thread_.join();
        }
        try {
            pipe_.stop();
        } catch (...) {
        }
        if (sockfd_ >= 0) {
            close(sockfd_);
        }
    }

private:
    void discoverDevices() {
        try {
            rs2::context ctx;
            auto devices = ctx.query_devices();
            RCLCPP_INFO(this->get_logger(), "--- RealSense Device Discovery ---");
            RCLCPP_INFO(this->get_logger(), "Found %zu RealSense device(s)", (size_t)devices.size());
            for (size_t i = 0; i < (size_t)devices.size(); ++i) {
                auto dev = devices[i];
                std::string name = dev.supports(RS2_CAMERA_INFO_NAME) ? dev.get_info(RS2_CAMERA_INFO_NAME) : "Unknown";
                std::string serial = dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER) ? dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) : "Unknown";
                RCLCPP_INFO(this->get_logger(), "  [%zu] %s (Serial: %s)", i, name.c_str(), serial.c_str());
            }
            RCLCPP_INFO(this->get_logger(), "----------------------------------");
        } catch (const rs2::error & e) {
            RCLCPP_WARN(this->get_logger(), "Could not discover devices: %s", e.what());
        }
    }

    void streamLoop() {
        uint32_t frame_id = 0;
        
        bool e_rgb = this->get_parameter("enable_rgb").as_bool();
        bool e_dep = this->get_parameter("enable_depth").as_bool();
        bool e_ir  = this->get_parameter("enable_ir").as_bool();

        auto last_time = std::chrono::steady_clock::now();

        while (running_ && rclcpp::ok()) {
            rs2::frameset frames;
            try {
                frames = pipe_.wait_for_frames(5000);
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Timeout waiting for frames: %s", e.what());
                continue;
            }

            if (frame_id > 0 && frame_id % 30 == 0) {
                auto now = std::chrono::steady_clock::now();
                double elapsed = std::chrono::duration<double>(now - last_time).count();
                RCLCPP_INFO(this->get_logger(), "Sending FPS: %.1f Hz", 30.0 / elapsed);
                last_time = now;
            }

            rs2::video_frame color_frame = frames.get_color_frame();
            rs2::depth_frame depth_frame = frames.get_depth_frame();
            rs2::video_frame ir_frame = frames.get_infrared_frame();

            if (e_rgb && color_frame) sendFrame(color_frame, 0, frame_id);
            if (e_dep && depth_frame) sendFrame(depth_frame, 1, frame_id);
            if (e_ir && ir_frame) sendFrame(ir_frame, 2, frame_id);
            
            frame_id++;
        }
    }

    void sendFrame(const rs2::video_frame& frame, uint8_t stream_type, uint32_t frame_id) {
        const uint8_t* data = (const uint8_t*)frame.get_data();
        uint32_t total_size = frame.get_height() * frame.get_stride_in_bytes();
        uint16_t total_chunks = (total_size + chunk_size_ - 1) / chunk_size_;

        UDPChunkHeader header;
        header.frame_id = frame_id;
        header.stream_type = stream_type;
        header.total_size = total_size;
        header.total_chunks = total_chunks;
        header.width = frame.get_width();
        header.height = frame.get_height();
        header.format = frame.get_profile().format();

        auto next_send = std::chrono::steady_clock::now();
        for (uint16_t i = 0; i < total_chunks; ++i) {
            uint32_t chunk_len = std::min((uint32_t)chunk_size_, total_size - i * chunk_size_);
            std::vector<uint8_t> packet(sizeof(UDPChunkHeader) + chunk_len);

            // Copy header
            header.chunk_index = i;
            header.chunk_size = chunk_len;
            memcpy(packet.data(), &header, sizeof(UDPChunkHeader));

            // Copy payload
            memcpy(packet.data() + sizeof(UDPChunkHeader), data + i * chunk_size_, chunk_len);

            ssize_t sent = sendto(sockfd_, packet.data(), packet.size(), 0,
                                  (struct sockaddr*)&target_addr_, sizeof(target_addr_));
            if (sent < 0) {
                RCLCPP_WARN(this->get_logger(), "Failed to send chunk: %s", strerror(errno));
            }
            
            // Precise busy-wait pacing (50us). Standard Linux sleep_for() oversleeps by 1-2 milliseconds!
            next_send += std::chrono::microseconds(50);
            while (std::chrono::steady_clock::now() < next_send) {
                // busy wait to guarantee microsecond precision without yielding to OS scheduler
            }
        }
    }

    std::string target_ip_;
    int target_port_;
    int chunk_size_;
    int sockfd_;
    struct sockaddr_in target_addr_;

    rs2::pipeline pipe_;
    rs2::pipeline_profile profile_;
    std::thread stream_thread_;
    std::atomic<bool> running_{false};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RealsenseUDPStreamer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
