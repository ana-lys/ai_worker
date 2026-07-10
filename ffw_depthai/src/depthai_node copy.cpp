#include "depthai/depthai.hpp"
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <memory>

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
  auto node = std::make_shared<rclcpp::Node>("depthai_stream_node");

  // Parse command line argument
  std::string mode = "h264"; // Default
  if (argc > 1) {
    mode = argv[1];
    // Convert to lowercase for robustness
    std::transform(mode.begin(), mode.end(), mode.begin(),
                   [](unsigned char c){ return std::tolower(c); });
  }

  std::shared_ptr<dai::Device> device;
  try {
    device = std::make_shared<dai::Device>();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node->get_logger(), "Failed to connect to device: %s", e.what());
    rclcpp::shutdown();
    return -1;
  }

  RCLCPP_INFO(node->get_logger(), "USB Speed: %d", static_cast<int>(device->getUsbSpeed()));

  dai::Pipeline pipeline(device);

  // Create Camera node (CAM_A is usually the RGB camera)
  auto cam = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);

  // Image quality tuning
  cam->initialControl.setSharpness(3);
  cam->initialControl.setLumaDenoise(2);
  cam->initialControl.setChromaDenoise(3);

  // Request 1080p NV12 output
  auto *videoOut = cam->requestOutput({1920, 1080}, dai::ImgFrame::Type::NV12);

  std::shared_ptr<dai::node::VideoEncoder> videoEnc;
  std::shared_ptr<dai::MessageQueue> videoQueue;
  std::string ffplayCmd;

  if (mode == "raw") {
    RCLCPP_INFO(node->get_logger(), "Selected Mode: RAW 1080p NV12 (Uncompressed)");
    videoQueue = videoOut->createOutputQueue(5, true); // Larger queue for raw
    ffplayCmd = "ffplay -f rawvideo -pixel_format nv12 -video_size 1920x1080 -framerate 30 "
                "-fflags nobuffer -flags low_delay -framedrop "
                "-window_title 'Live Raw 1080p NV12' -i - 2>/dev/null";
  }
  else if (mode == "h265") {
    RCLCPP_INFO(node->get_logger(), "Selected Mode: H.265 @ 40 Mbps");
    videoEnc = pipeline.create<dai::node::VideoEncoder>();
    videoEnc->setDefaultProfilePreset(30, dai::VideoEncoderProperties::Profile::H265_MAIN);
    videoEnc->setBitrateKbps(40000);
    videoEnc->setRateControlMode(dai::VideoEncoderProperties::RateControlMode::CBR);
    videoEnc->setKeyframeFrequency(5);

    videoOut->link(videoEnc->input);
    videoQueue = videoEnc->bitstream.createOutputQueue();

    ffplayCmd = "ffplay -f hevc -framerate 30 -probesize 32 -analyzeduration 0 "
                "-fflags nobuffer -flags low_delay -framedrop -strict experimental "
                "-window_title 'Live 1080p H.265' -i - 2>/dev/null";
  }
  else if (mode == "jpeg" || mode == "mjpeg") {
    RCLCPP_INFO(node->get_logger(), "Selected Mode: MJPEG @ Quality 90");
    videoEnc = pipeline.create<dai::node::VideoEncoder>();
    videoEnc->setDefaultProfilePreset(30, dai::VideoEncoderProperties::Profile::MJPEG);
    videoEnc->setQuality(90);

    videoOut->link(videoEnc->input);
    videoQueue = videoEnc->bitstream.createOutputQueue();

    ffplayCmd = "ffplay -f mjpeg -framerate 30 -probesize 32 -analyzeduration 0 "
                "-fflags nobuffer -flags low_delay -framedrop "
                "-window_title 'Live 1080p MJPEG' -i - 2>/dev/null";
  }
  else {  // Default: H.264
    RCLCPP_INFO(node->get_logger(), "Selected Mode: H.264 @ 40 Mbps");
    videoEnc = pipeline.create<dai::node::VideoEncoder>();
    videoEnc->setDefaultProfilePreset(30, dai::VideoEncoderProperties::Profile::H264_HIGH);
    videoEnc->setBitrateKbps(40000);
    videoEnc->setRateControlMode(dai::VideoEncoderProperties::RateControlMode::CBR);
    videoEnc->setKeyframeFrequency(5);

    videoOut->link(videoEnc->input);
    videoQueue = videoEnc->bitstream.createOutputQueue();

    ffplayCmd = "ffplay -f h264 -framerate 30 -probesize 32 -analyzeduration 0 "
                "-fflags nobuffer -flags low_delay -framedrop -strict experimental "
                "-window_title 'Live 1080p H.264' -i - 2>/dev/null";
  }

  // System logger for telemetry
  auto sysLog = pipeline.create<dai::node::SystemLogger>();
  sysLog->setRate(0.2f);
  auto sysLogQueue = sysLog->out.createOutputQueue();

  pipeline.start();

  RCLCPP_INFO(node->get_logger(), "Spawning ffplay...");
  FILE *ffplayPipe = popen(ffplayCmd.c_str(), "w");
  if (!ffplayPipe) {
    RCLCPP_ERROR(node->get_logger(), "Failed to open ffplay pipe!");
    rclcpp::shutdown();
    return -1;
  }

  while (rclcpp::ok() && pipeline.isRunning()) {
    auto videoFrame = videoQueue->get<dai::ImgFrame>();
    if (videoFrame) {
      const auto& data = videoFrame->getData();
      fwrite(data.data(), 1, data.size(), ffplayPipe);
      fflush(ffplayPipe);
    }

    auto sysInfo = sysLogQueue->tryGet<dai::SystemInformation>();
    if (sysInfo) {
      printSystemInformation(*sysInfo);
    }

    rclcpp::spin_some(node);
  }

  std::cout << "\nShutting down..." << std::endl;
  pclose(ffplayPipe);
  rclcpp::shutdown();
  return 0;
}