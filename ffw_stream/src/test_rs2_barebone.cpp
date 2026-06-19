#include <librealsense2/rs.hpp>
#include <iostream>

int main() {
    std::cout << "Initializing RealSense pipeline..." << std::endl;
    try {
        rs2::pipeline pipe;
        
        std::cout << "Starting pipeline with default settings..." << std::endl;
        // Start with no config object; uses default stream profile for the first detected camera
        rs2::pipeline_profile profile = pipe.start();
        std::cout << "Pipeline started successfully!" << std::endl;
        
        std::cout << "Waiting for frames..." << std::endl;
        // Wait for 30 frames to let auto-exposure settle
        for (int i = 0; i < 30; ++i) {
            // Just wait for any frame to prove the pipeline is running
            rs2::frameset frames = pipe.wait_for_frames(5000);
            if (frames) {
                if (i == 29) {
                    std::cout << "Successfully received 30 frames!" << std::endl;
                }
            } else {
                std::cout << "Warning: Dropped a frame." << std::endl;
            }
        }
        
        std::cout << "Test passed. Stopping pipeline..." << std::endl;
        pipe.stop();
        
    } catch (const rs2::error & e) {
        std::cerr << "RealSense error calling " << e.get_failed_function() 
                  << "(" << e.get_failed_args() << "):\n    " 
                  << e.what() << std::endl;
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "Standard exception: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
