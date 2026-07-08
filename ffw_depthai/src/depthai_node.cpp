#include "depthai/depthai.hpp"
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

void printSystemInformation(const dai::SystemInformation &info) {
  const float m = 1024.0f * 1024.0f; // MiB
  const auto &t = info.chipTemperature;
  std::cout << "\r[Telemetry] "
            << "CPU CSS: " << std::fixed << std::setprecision(1)
            << info.leonCssCpuUsage.average * 100.0f << "% | "
            << "MSS: " << info.leonMssCpuUsage.average * 100.0f << "% | "
            << "RAM: " << info.ddrMemoryUsage.used / m << "/"
            << info.ddrMemoryUsage.total / m << " MiB | "
            << "Temp: " << t.average << "C    " << std::flush;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("depthai_dummy_node");

  std::shared_ptr<dai::Device> device;
  try {
    device = std::make_shared<dai::Device>();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node->get_logger(), "Failed to connect to device: %s",
                 e.what());
    rclcpp::shutdown();
    return -1;
  }

  RCLCPP_INFO(node->get_logger(), "USB Speed: %d",
              static_cast<int>(device->getUsbSpeed()));

  dai::Pipeline pipeline(device);

  // Create Camera node
  auto cam = pipeline.create<dai::node::Camera>()->build(
      dai::CameraBoardSocket::CAM_A);

  // Request 4K output for the encoder in NV12 format
  auto *encOut = cam->requestOutput({3840, 2160}, dai::ImgFrame::Type::NV12);

  // Create VideoEncoder node (Back to H.264 to fix ffplay pipe decoding
  // smearing)
  auto videoEnc = pipeline.create<dai::node::VideoEncoder>();
  videoEnc->setDefaultProfilePreset(
      30, dai::VideoEncoderProperties::Profile::H264_MAIN);
  videoEnc->setBitrateKbps(30000); // 30 Mbps (Very sharp, but decodable)
  videoEnc->setKeyframeFrequency(
      15); // Send an I-Frame every 0.5 seconds to instantly fix gray screens

  encOut->link(videoEnc->input);

  // Create SystemLogger node
  auto sysLog = pipeline.create<dai::node::SystemLogger>();
  sysLog->setRate(1.0f); // 1 Hz update rate

  auto h264Queue = videoEnc->bitstream.createOutputQueue();
  auto sysLogQueue = sysLog->out.createOutputQueue();

  pipeline.start();

  RCLCPP_INFO(node->get_logger(), "Starting live 4K H.264 stream natively. "
                                  "Spawning ffplay hardware decoder...");
  RCLCPP_INFO(node->get_logger(), "Press Ctrl+C in this terminal to exit.");

  // Spawn an extremely low-latency ffplay instance and stream the H.264 packets
  // directly into it via a pipe
  FILE *ffplayPipe =
      popen("ffplay -f h264 -framerate 30 -fflags nobuffer -flags low_delay "
            "-framedrop -strict experimental -window_title 'Live 4K H.264 "
            "Stream' -i - 2>/dev/null",
            "w");

  if (!ffplayPipe) {
    RCLCPP_ERROR(node->get_logger(),
                 "Failed to open ffplay pipe! Make sure ffmpeg is installed.");
    rclcpp::shutdown();
    return -1;
  }

  while (rclcpp::ok() && pipeline.isRunning()) {
    // Handle H.264 bitstream
    auto h264Frame = h264Queue->tryGet<dai::ImgFrame>();
    if (h264Frame) {
      fwrite(h264Frame->getData().data(), 1, h264Frame->getData().size(),
             ffplayPipe);
      fflush(ffplayPipe);
    }

    // Handle System Telemetry (arrives at 1 Hz)
    auto sysInfo = sysLogQueue->tryGet<dai::SystemInformation>();
    if (sysInfo) {
      printSystemInformation(*sysInfo);
    }

    // Let ROS spin once to handle signals
    rclcpp::spin_some(node);
  }

  std::cout << "\nShutting down..." << std::endl;
  pclose(ffplayPipe);
  rclcpp::shutdown();
  return 0;
}
