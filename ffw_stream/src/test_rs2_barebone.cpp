#include <librealsense2/rs.hpp>
#include <iostream>

void test_camera(const std::string& serial, int ir_index) {
    std::cout << "\n===========================================" << std::endl;
    std::cout << "Testing Camera: " << serial << " with IR Index: " << ir_index << std::endl;
    try {
        rs2::pipeline pipe;
        rs2::config cfg;
        
        cfg.enable_device(serial);
        cfg.enable_stream(RS2_STREAM_DEPTH, 480, 270, RS2_FORMAT_Z16, 30);
        
        // Try enabling IR stream with the given index (D435 uses 1, D405 might need 0)
        cfg.enable_stream(RS2_STREAM_INFRARED, ir_index, 480, 270, RS2_FORMAT_Y8, 30);
        
        std::cout << "Starting pipeline..." << std::endl;
        pipe.start(cfg);
        std::cout << "SUCCESS! Pipeline started." << std::endl;
        
        pipe.stop();
    } catch (const rs2::error & e) {
        std::cerr << "FAILED! RealSense error: " << e.what() << std::endl;
    }
}

int main() {
    std::string cam1 = "230422272589";
    std::string cam2 = "230422271116";

    std::cout << "The streamer node failed because it defaults to IR index 1 (for D435)." << std::endl;
    std::cout << "Let's see if D405 fails with IR index 1, but succeeds with IR index 0:\n" << std::endl;

    test_camera(cam1, 1); // This is what the streamer was doing
    test_camera(cam1, 0); // This is what D405 probably requires

    test_camera(cam2, 1);
    test_camera(cam2, 0);

    return 0;
}
