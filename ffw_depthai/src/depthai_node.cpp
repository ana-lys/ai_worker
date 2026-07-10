#include "depthai/depthai.hpp"
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <memory>

void printSystemInformation(const dai::SystemInformation &info) {
  const float m = 1024.0f * 1024.0f;
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
  auto node = std::make_shared<rclcpp::Node>("depthai_mjpeg_udp_node");

  gst_init(nullptr, nullptr);

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

  auto cam = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
  cam->initialControl.setSharpness(4);
  cam->initialControl.setLumaDenoise(0);
  cam->initialControl.setChromaDenoise(2);
  cam->initialControl.setManualFocus(120);

  auto *videoOut = cam->requestOutput({1920, 1080}, dai::ImgFrame::Type::NV12);

  auto videoEnc = pipeline.create<dai::node::VideoEncoder>();
  videoEnc->setDefaultProfilePreset(30, dai::VideoEncoderProperties::Profile::MJPEG);
  videoEnc->setQuality(85);

  videoOut->link(videoEnc->input);
  auto videoQueue = videoEnc->bitstream.createOutputQueue(8, false);

  auto sysLog = pipeline.create<dai::node::SystemLogger>();
  sysLog->setRate(0.2f);
  auto sysLogQueue = sysLog->out.createOutputQueue();

  pipeline.start();

  // ==================== GStreamer Pipeline ====================
  GError *error = nullptr;
  GstElement *gst_pipeline = gst_parse_launch(
      "appsrc name=src is-live=true format=3 do-timestamp=true block=true ! "
      "image/jpeg,framerate=30/1,width=1920,height=1080 ! "
      "rtpjpegpay ! "
      "udpsink host=192.168.0.241 port=9100 sync=false async=false",
      &error);

  if (error) {
    RCLCPP_ERROR(node->get_logger(), "GStreamer error: %s", error->message);
    g_error_free(error);
    rclcpp::shutdown();
    return -1;
  }

  GstElement *appsrc = gst_bin_get_by_name(GST_BIN(gst_pipeline), "src");
  gst_element_set_state(gst_pipeline, GST_STATE_PLAYING);

  RCLCPP_INFO(node->get_logger(), "Streaming MJPEG over UDP to 192.168.0.241:9100");

  int frame_count = 0;

  while (rclcpp::ok() && pipeline.isRunning()) {
    // Fixed: Use the correct get() overload
    bool hasTimedOut = false;
    auto videoFrame = videoQueue->get<dai::ImgFrame>(std::chrono::milliseconds(500), hasTimedOut);

    if (videoFrame && !hasTimedOut) {
      const auto& data = videoFrame->getData();

      GstBuffer *buffer = gst_buffer_new_allocate(nullptr, data.size(), nullptr);
      GstMapInfo map;
      if (gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
        memcpy(map.data, data.data(), data.size());
        gst_buffer_unmap(buffer, &map);
      }

      GST_BUFFER_PTS(buffer) = GST_CLOCK_TIME_NONE;
      GST_BUFFER_DTS(buffer) = GST_CLOCK_TIME_NONE;

      GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(appsrc), buffer);
      if (ret != GST_FLOW_OK) {
        RCLCPP_WARN(node->get_logger(), "Failed to push buffer (ret=%d)", ret);
      }

      frame_count++;
      if (frame_count % 300 == 0) {
        RCLCPP_INFO(node->get_logger(), "Sent %d frames", frame_count);
      }
    }

    auto sysInfo = sysLogQueue->tryGet<dai::SystemInformation>();
    if (sysInfo) {
      printSystemInformation(*sysInfo);
    }

    rclcpp::spin_some(node);
  }

  std::cout << "\nShutting down..." << std::endl;
  gst_element_set_state(gst_pipeline, GST_STATE_NULL);
  gst_object_unref(appsrc);
  gst_object_unref(gst_pipeline);

  rclcpp::shutdown();
  return 0;
}