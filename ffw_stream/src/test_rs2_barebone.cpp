// Dual-thread test: open Depth + IR (index 1) simultaneously for every camera.
// Each camera runs its pipeline in its own thread, so all cameras stream at the
// same time.

#include <iostream>
#include <librealsense2/rs.hpp>
#include <mutex>
#include <sstream>
#include <thread>
#include <vector>

std::mutex cout_mutex;

void log(const std::string &msg) {
  std::lock_guard<std::mutex> lock(cout_mutex);
  std::cout << msg << std::endl;
}

void probe_dual(const std::string &serial, int index) {
  std::ostringstream header;
  header << "\n=== CAM" << index << " " << serial
         << " : starting Depth + IR idx 1 ===";
  log(header.str());

  try {
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_device(serial);
    cfg.enable_stream(RS2_STREAM_DEPTH, 480, 270, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, 480, 270, RS2_FORMAT_Y8, 30);

    pipe.start(cfg);

    // Pull a handful of framesets to confirm both streams keep arriving
    // while every other camera's thread is also streaming concurrently.
    for (int i = 0; i < 10; ++i) {
      rs2::frameset frames = pipe.wait_for_frames(5000);

      bool got_depth = false, got_ir = false;
      for (auto &&f : frames) {
        auto type = f.get_profile().stream_type();
        if (type == RS2_STREAM_DEPTH)
          got_depth = true;
        if (type == RS2_STREAM_INFRARED)
          got_ir = true;
      }

      std::ostringstream msg;
      msg << "CAM" << index << " " << serial << " frame " << i
          << ": depth=" << (got_depth ? "yes" : "no")
          << " ir=" << (got_ir ? "yes" : "no");
      log(msg.str());
    }

    pipe.stop();

    std::ostringstream ok;
    ok << "=== CAM" << index << " " << serial << " : OK ===";
    log(ok.str());

  } catch (const rs2::error &e) {
    std::ostringstream err;
    err << "=== CAM" << index << " " << serial
        << " : FAILED (rs2::error): " << e.what() << " ===";
    log(err.str());
  } catch (const std::exception &e) {
    std::ostringstream err;
    err << "=== CAM" << index << " " << serial << " : EXCEPTION: " << e.what()
        << " ===";
    log(err.str());
  }
}

int main() {
  rs2::context ctx;
  auto devices = ctx.query_devices();
  if (devices.size() == 0) {
    std::cerr << "No devices found!" << std::endl;
    return 1;
  }

  // Collect serials first (query_devices() result can become awkward to
  // pass into threads directly).
  std::vector<std::string> serials;
  for (size_t i = 0; i < devices.size(); ++i) {
    std::string serial =
        devices[i].supports(RS2_CAMERA_INFO_SERIAL_NUMBER)
            ? devices[i].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)
            : "";
    serials.push_back(serial);
  }

  // Launch one thread per camera; each opens Depth + IR idx 1 at the same time.
  std::vector<std::thread> threads;
  for (size_t i = 0; i < serials.size(); ++i) {
    threads.emplace_back(probe_dual, serials[i], static_cast<int>(i));
  }

  for (auto &t : threads) {
    t.join();
  }

  log("\n=== Done ===");
  return 0;
}