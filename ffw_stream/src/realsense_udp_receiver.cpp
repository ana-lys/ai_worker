#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>

#include <opencv2/opencv.hpp>

#include <iostream>
#include <thread>
#include <mutex>

class RealsenseUDPReceiver : public rclcpp::Node {
public:
    RealsenseUDPReceiver() : Node("realsense_udp_receiver") {
        // Ports
        this->declare_parameter<int>("target_port_rgb", 8080);
        this->declare_parameter<int>("target_port_depth", 8082);
        this->declare_parameter<int>("target_port_ir", 8084);

        // Camera identity for display namespaces
        this->declare_parameter<std::string>("camera_name", "Camera");
        camera_name_ = this->get_parameter("camera_name").as_string();

        // Streams enabled
        this->declare_parameter<bool>("enable_rgb", true);
        this->declare_parameter<bool>("enable_depth", true);
        this->declare_parameter<bool>("enable_ir", false);

        // Stream Configs
        this->declare_parameter<int>("rgb_width", 640);
        this->declare_parameter<int>("rgb_height", 480);
        this->declare_parameter<int>("depth_width", 640);
        this->declare_parameter<int>("depth_height", 480);
        this->declare_parameter<int>("ir_width", 640);
        this->declare_parameter<int>("ir_height", 480);

        // Publishers
        pub_rgb_ = this->create_publisher<sensor_msgs::msg::Image>("camera/color/image_raw", 10);
        pub_depth_ = this->create_publisher<sensor_msgs::msg::Image>("camera/depth/image_rect_raw", 10);
        pub_ir_ = this->create_publisher<sensor_msgs::msg::Image>("camera/infra1/image_rect_raw", 10);

        if (this->get_parameter("enable_rgb").as_bool()) {
            int port = this->get_parameter("target_port_rgb").as_int();
            std::string pipe_str = "udpsrc port=" + std::to_string(port) + 
                                   " ! application/x-rtp,media=video,clock-rate=90000,encoding-name=H264 ! "
                                   "rtph264depay ! decodebin ! videoconvert ! video/x-raw,format=BGR ! appsink name=sink drop=true max-buffers=1";
            initGstPipeline("RGB", pipe_str, pipeline_rgb_, appsink_rgb_);
        }

        if (this->get_parameter("enable_depth").as_bool()) {
            int port = this->get_parameter("target_port_depth").as_int();
            int w = this->get_parameter("depth_width").as_int();
            int h = this->get_parameter("depth_height").as_int();
            std::string pipe_str = "udpsrc port=" + std::to_string(port) + " buffer-size=2500000"
                                   " ! application/x-rtp,media=video,clock-rate=90000,encoding-name=RAW,sampling=YCbCr-4:2:2,depth=(string)8,"
                                   "width=(string)" + std::to_string(w) + ",height=(string)" + std::to_string(h) + " ! queue ! "
                                   "rtpvrawdepay ! queue ! appsink name=sink drop=true max-buffers=1";
            initGstPipeline("Depth", pipe_str, pipeline_depth_, appsink_depth_);
        }

        if (this->get_parameter("enable_ir").as_bool()) {
            int port = this->get_parameter("target_port_ir").as_int();
            std::string pipe_str = "udpsrc port=" + std::to_string(port) + 
                                   " ! application/x-rtp,media=video,clock-rate=90000,encoding-name=H264 ! "
                                   "rtph264depay ! decodebin ! videoconvert ! video/x-raw,format=GRAY8 ! appsink name=sink drop=true max-buffers=1";
            initGstPipeline("IR", pipe_str, pipeline_ir_, appsink_ir_);
        }

        RCLCPP_INFO(this->get_logger(), "C++ UDP Receiver Initialized.");
        
        display_thread_ = std::thread(&RealsenseUDPReceiver::displayLoop, this);
    }

    ~RealsenseUDPReceiver() {
        running_ = false;
        if (display_thread_.joinable()) display_thread_.join();
        
        if (pipeline_rgb_) { gst_element_set_state(pipeline_rgb_, GST_STATE_NULL); gst_object_unref(pipeline_rgb_); }
        if (pipeline_depth_) { gst_element_set_state(pipeline_depth_, GST_STATE_NULL); gst_object_unref(pipeline_depth_); }
        if (pipeline_ir_) { gst_element_set_state(pipeline_ir_, GST_STATE_NULL); gst_object_unref(pipeline_ir_); }
        
        cv::destroyAllWindows();
    }

private:
    void initGstPipeline(const std::string& name, const std::string& pipe_str, GstElement*& pipeline, GstElement*& appsink) {
        GError *error = nullptr;
        pipeline = gst_parse_launch(pipe_str.c_str(), &error);
        if (error) {
            RCLCPP_ERROR(this->get_logger(), "GStreamer parse error for %s: %s", name.c_str(), error->message);
            g_error_free(error);
            return;
        }
        
        appsink = gst_bin_get_by_name(GST_BIN(pipeline), "sink");
        gst_app_sink_set_emit_signals(GST_APP_SINK(appsink), true);
        
        // Pass instance and name
        auto* data = new std::pair<RealsenseUDPReceiver*, std::string>(this, name);
        g_signal_connect(appsink, "new-sample", G_CALLBACK(onNewSample), data);
        
        gst_element_set_state(pipeline, GST_STATE_PLAYING);
        RCLCPP_INFO(this->get_logger(), "Started %s Pipeline: %s", name.c_str(), pipe_str.c_str());
    }

    static GstFlowReturn onNewSample(GstElement* sink, gpointer user_data) {
        auto* data = static_cast<std::pair<RealsenseUDPReceiver*, std::string>*>(user_data);
        data->first->processSample(sink, data->second);
        return GST_FLOW_OK;
    }

    void processSample(GstElement* sink, const std::string& stream_name) {
        GstSample* sample = gst_app_sink_pull_sample(GST_APP_SINK(sink));
        if (!sample) return;

        GstBuffer* buffer = gst_sample_get_buffer(sample);
        GstCaps* caps = gst_sample_get_caps(sample);
        GstStructure* s = gst_caps_get_structure(caps, 0);

        int w, h;
        gst_structure_get_int(s, "width", &w);
        gst_structure_get_int(s, "height", &h);

        GstMapInfo map;
        gst_buffer_map(buffer, &map, GST_MAP_READ);

        cv::Mat frame;
        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = "camera_link";

        if (stream_name == "RGB") {
            frame = cv::Mat(h, w, CV_8UC3, map.data).clone();
            sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
            pub_rgb_->publish(*msg);
        } else if (stream_name == "IR") {
            frame = cv::Mat(h, w, CV_8UC1, map.data).clone();
            sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header, "mono8", frame).toImageMsg();
            pub_ir_->publish(*msg);
        } else if (stream_name == "Depth") {
            // Data is RAW UYVY format but represents GRAY16_LE depth
            frame = cv::Mat(h, w, CV_16UC1, map.data).clone();
            sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header, "16UC1", frame).toImageMsg();
            pub_depth_->publish(*msg);
        }

        gst_buffer_unmap(buffer, &map);
        gst_sample_unref(sample);

        // Save for display loop
        std::lock_guard<std::mutex> lock(img_mutex_);
        latest_images_[stream_name] = frame;
        
        frames_received_[stream_name]++;
        auto now = std::chrono::steady_clock::now();
        if (frames_received_[stream_name] == 30) {
            double elapsed = std::chrono::duration<double>(now - fps_timers_[stream_name]).count();
            RCLCPP_INFO(this->get_logger(), "[%s] Receiver FPS: %.1f", stream_name.c_str(), 30.0 / elapsed);
            frames_received_[stream_name] = 0;
            fps_timers_[stream_name] = now;
        }
    }

    void displayLoop() {
        while (running_ && rclcpp::ok()) {
            {
                std::lock_guard<std::mutex> lock(img_mutex_);
                for (auto& pair : latest_images_) {
                    if (!pair.second.empty()) {
                        cv::Mat display_img = pair.second;
                        
                        if (pair.first == "Depth") {
                            // Apply colormap
                            cv::Mat img_clamped, img_normalized;
                            display_img.convertTo(img_clamped, CV_32F);
                            cv::threshold(img_clamped, img_clamped, 3000, 3000, cv::THRESH_TRUNC);
                            img_normalized = img_clamped / 3000.0 * 255.0;
                            img_normalized.convertTo(img_normalized, CV_8UC1);
                            cv::applyColorMap(img_normalized, display_img, cv::COLORMAP_JET);
                        }
                        
                        std::string window_name = camera_name_ + " - " + pair.first;
                        cv::imshow(window_name, display_img);
                        pair.second = cv::Mat(); // consume
                    }
                }
            }
            int key = cv::waitKey(1);
            if (key == 27 || key == 'q') {
                rclcpp::shutdown();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    std::string camera_name_;
    GstElement* pipeline_rgb_ = nullptr;
    GstElement* appsink_rgb_ = nullptr;
    GstElement* pipeline_depth_ = nullptr;
    GstElement* appsink_depth_ = nullptr;
    GstElement* pipeline_ir_ = nullptr;
    GstElement* appsink_ir_ = nullptr;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_rgb_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_ir_;

    std::mutex img_mutex_;
    std::map<std::string, cv::Mat> latest_images_;
    std::map<std::string, int> frames_received_;
    std::map<std::string, std::chrono::steady_clock::time_point> fps_timers_;

    std::thread display_thread_;
    bool running_{true};
};

int main(int argc, char** argv) {
    gst_init(&argc, &argv);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RealsenseUDPReceiver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
