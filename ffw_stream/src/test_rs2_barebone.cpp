// Barebone test: auto-detect all D405 cameras, open each with
// 480x270 IR @ 30fps (no depth, no RGB, no GStreamer, no config files).
// This isolates RS2 multi-camera init from everything else.

#include <librealsense2/rs.hpp>
#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

std::atomic<bool> running{true};

void stream_camera(const std::string& serial, int index) {
    std::cout << "[CAM" << index << " " << serial << "] Opening..." << std::endl;
    try {
        rs2::pipeline pipe;
        rs2::config cfg;

        cfg.enable_device(serial);
        // IR only, index 0, 480x270 @ 30fps
        cfg.enable_stream(RS2_STREAM_INFRARED, 0, 480, 270, RS2_FORMAT_Y8, 30);

        pipe.start(cfg);
        std::cout << "[CAM" << index << " " << serial << "] Pipeline started OK!" << std::endl;

        uint32_t frame_count = 0;
        auto last_print = std::chrono::steady_clock::now();
        while (running) {
            rs2::frameset frames = pipe.wait_for_frames(5000);
            if (frames) {
                frame_count++;
                auto now = std::chrono::steady_clock::now();
                if (std::chrono::duration_cast<std::chrono::seconds>(now - last_print).count() >= 2) {
                    std::cout << "[CAM" << index << " " << serial << "] "
                              << frame_count << " frames received" << std::endl;
                    frame_count = 0;
                    last_print = now;
                }
            }
        }

        pipe.stop();
        std::cout << "[CAM" << index << " " << serial << "] Stopped." << std::endl;

    } catch (const rs2::error& e) {
        std::cerr << "[CAM" << index << " " << serial << "] FAILED: " << e.what() << std::endl;
    }
}

int main() {
    std::cout << "=== Querying connected RealSense devices ===" << std::endl;

    rs2::context ctx;
    auto devices = ctx.query_devices();
    size_t n = devices.size();

    if (n == 0) {
        std::cerr << "No RealSense devices found!" << std::endl;
        return 1;
    }

    std::vector<std::string> serials;
    for (size_t i = 0; i < n; ++i) {
        auto dev = devices[i];
        std::string name = dev.supports(RS2_CAMERA_INFO_NAME)
                               ? dev.get_info(RS2_CAMERA_INFO_NAME) : "Unknown";
        std::string serial = dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER)
                                 ? dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) : "";
        std::cout << "  Found [" << i << "]: " << name << " (Serial: " << serial << ")" << std::endl;
        serials.push_back(serial);
    }
    std::cout << "=== Opening " << n << " camera(s) with 480x270 IR @ 30fps ===" << std::endl;

    std::vector<std::thread> threads;
    for (size_t i = 0; i < serials.size(); ++i) {
        threads.emplace_back(stream_camera, serials[i], (int)i);
        // Stagger startup slightly to avoid USB enumeration conflicts
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    std::cout << "=== Press Ctrl+C to stop ===" << std::endl;
    // Run until Ctrl+C
    while (running) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    for (auto& t : threads) t.join();
    return 0;
}
