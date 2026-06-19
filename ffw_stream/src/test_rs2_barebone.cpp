#include <librealsense2/rs.hpp>
#include <iostream>
#include <set>

int main() {
    try {
        rs2::context ctx;
        auto devices = ctx.query_devices();
        if (devices.size() == 0) {
            std::cout << "No RealSense devices found." << std::endl;
            return 1;
        }
        
        auto dev = devices[0];
        std::cout << "===========================================" << std::endl;
        std::cout << "Device Name: " << dev.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
        std::cout << "Firmware:    " << dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION) << std::endl;
        std::cout << "===========================================" << std::endl;
        
        std::set<std::string> depth_res;
        std::set<std::string> color_res;
        std::set<std::string> ir_res;
        
        for (auto sensor : dev.query_sensors()) {
            for (auto profile : sensor.get_stream_profiles()) {
                if (profile.is<rs2::video_stream_profile>()) {
                    auto video = profile.as<rs2::video_stream_profile>();
                    std::string desc = std::to_string(video.width()) + "x" + std::to_string(video.height()) + 
                                       " @ " + std::to_string(video.fps()) + "fps (" + rs2_format_to_string(video.format()) + ")";
                    
                    if (profile.stream_type() == RS2_STREAM_DEPTH) depth_res.insert(desc);
                    else if (profile.stream_type() == RS2_STREAM_COLOR) color_res.insert(desc);
                    else if (profile.stream_type() == RS2_STREAM_INFRARED) ir_res.insert(desc);
                }
            }
        }
        
        std::cout << "\nSupported DEPTH Profiles:" << std::endl;
        for (const auto& r : depth_res) std::cout << "  - " << r << std::endl;
        
        std::cout << "\nSupported COLOR Profiles:" << std::endl;
        for (const auto& r : color_res) std::cout << "  - " << r << std::endl;
        
        std::cout << "\nSupported IR Profiles:" << std::endl;
        if (ir_res.empty()) std::cout << "  (None)" << std::endl;
        for (const auto& r : ir_res) std::cout << "  - " << r << std::endl;
        
        std::cout << "===========================================" << std::endl;
        
    } catch (const rs2::error & e) {
        std::cerr << "RealSense error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
