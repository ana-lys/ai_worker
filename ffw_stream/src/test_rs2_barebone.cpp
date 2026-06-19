// Probe test: find exactly which stream combination the D405 accepts.
// Tests IR with different indices, and depth+IR together.

#include <librealsense2/rs.hpp>
#include <iostream>

bool try_open(const std::string& serial, const std::string& label,
              std::function<void(rs2::config&)> setup) {
    std::cout << "  [" << label << "] trying..." << std::flush;
    try {
        rs2::pipeline pipe;
        rs2::config cfg;
        cfg.enable_device(serial);
        setup(cfg);
        pipe.start(cfg);
        // grab one frameset to confirm frames arrive
        pipe.wait_for_frames(5000);
        pipe.stop();
        std::cout << " OK!" << std::endl;
        return true;
    } catch (const rs2::error& e) {
        std::cout << " FAILED: " << e.what() << std::endl;
        return false;
    }
}

void probe(const std::string& serial, int index) {
    std::cout << "\n=== CAM" << index << " " << serial << " ===" << std::endl;

    try_open(serial, "IR idx 1 only",    [](rs2::config& c){ c.enable_stream(RS2_STREAM_INFRARED, 1, 480, 270, RS2_FORMAT_Y8, 30); });
    try_open(serial, "IR idx 0 only",    [](rs2::config& c){ c.enable_stream(RS2_STREAM_INFRARED, 0, 480, 270, RS2_FORMAT_Y8, 30); });
    try_open(serial, "Depth only",       [](rs2::config& c){ c.enable_stream(RS2_STREAM_DEPTH,    480, 270, RS2_FORMAT_Z16, 30); });
    try_open(serial, "Depth + IR idx 1", [](rs2::config& c){
        c.enable_stream(RS2_STREAM_DEPTH,     480, 270, RS2_FORMAT_Z16, 30);
        c.enable_stream(RS2_STREAM_INFRARED, 1, 480, 270, RS2_FORMAT_Y8, 30);
    });
    try_open(serial, "Depth + IR idx 0", [](rs2::config& c){
        c.enable_stream(RS2_STREAM_DEPTH,     480, 270, RS2_FORMAT_Z16, 30);
        c.enable_stream(RS2_STREAM_INFRARED, 0, 480, 270, RS2_FORMAT_Y8, 30);
    });
}

int main() {
    rs2::context ctx;
    auto devices = ctx.query_devices();
    if (devices.size() == 0) {
        std::cerr << "No devices found!" << std::endl;
        return 1;
    }

    for (size_t i = 0; i < devices.size(); ++i) {
        std::string serial = devices[i].supports(RS2_CAMERA_INFO_SERIAL_NUMBER)
                                 ? devices[i].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) : "";
        probe(serial, (int)i);
    }

    std::cout << "\n=== Done ===" << std::endl;
    return 0;
}
