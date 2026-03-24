#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <sstream>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

using std::placeholders::_1;

// --- System Enums ---
enum class Mode         { RETURN = -1, MANUAL = 0, OFFBOARD = 1 }; // T0
enum class Objective    { END_EFFECTOR = -1, POSE = 0, NAVIGATE = 1 }; // T1
enum class SpareT       { LOW = -1, MID = 0, HIGH = 1 }; // T2

enum class Trigger      { ON = -1, OFF = 1 };        // B0
enum class Selector     { SECONDARY = -1, PRIMARY = 1 }; // B1
enum class KillSwitch   { KILL = -1, RUN = 1 };       // B2

std::vector<int> decode_universal_switches(float clean_val, float unit_length, int n, int count) {
    float unit_scaled = (clean_val * 100.0f) / unit_length;
    int remaining = static_cast<int>(std::round(unit_scaled));
    std::vector<int> states;
    for (int i = count - 1; i >= 0; --i) {
        int weight = static_cast<int>(std::pow(n, i));
        int s = 0;
        if (n == 3) {
            if (remaining > weight / 2)       { s = 1;  remaining -= weight; }
            else if (remaining < -weight / 2) { s = -1; remaining += weight; }
            else                               { s = 0; }
        } else {
            s = (remaining >= 0) ? 1 : -1;
            remaining -= (s * weight);
        }
        states.push_back(s);
    }
    return states;
}

class RadioDashboard : public rclcpp::Node {
public:
    RadioDashboard() : Node("radio_dashboard") {
        this->declare_parameter("center_offset", 0.0);
        this->declare_parameter("display_mode", 2);
        
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&RadioDashboard::joy_callback, this, _1));
        
        channel_names_ = {"CH1 (RY)", "CH2 (LX)", "CH3 (RX)", "CH4 (LY)", "CH5 (LZ)", "CH6 (RZ)", "CH7 (2P)", "CH8 (3P)"};
        
        B.assign(5, -1); // B0-B4
        T.assign(3, 0);  // T0-T2
    }

    float RX, RY, RZ, LX, LY, LZ;
    std::vector<int> B; 
    std::vector<int> T; 

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        int mode = this->get_parameter("display_mode").as_int();
        std::stringstream ss;
        ss << "\033[2J\033[H"; 
        if (mode == 0 || msg->axes.size() < 8) return;
        else
        ss << "\n------------------ RADIO DASHBOARD ------------------\n";
        double offset = this->get_parameter("center_offset").as_double();
       
       
        for (size_t i = 0; i < 8; ++i) {
            float clean_val = std::clamp(static_cast<float>(msg->axes[i] - offset), -1.0f, 1.0f);
            
            if (i < 6) {
                // Update Sticks
                switch(i) {
                    case 0: RY = clean_val; break; case 1: LX = clean_val; break;
                    case 2: RX = clean_val; break; case 3: LY = clean_val; break;
                    case 4: LZ = clean_val; break; case 5: RZ = clean_val; break;
                }
                if (mode == 2) {
                    char buf[128];
                    sprintf(buf, " %-10s | %7.2f %%\n", channel_names_[i].c_str(), clean_val * 100.0f);
                    ss << buf;
                }
            } else {
                float U = 2.76f;
                int n = (i == 7) ? 3 : 2;
                int count = (i == 7) ? 3 : 5;
                auto sw_states = decode_universal_switches(clean_val, U, n, count);

                // Mapping: Decoder returns [S_max ... S1]. 
                // We want B0/T0 to be S1 (the index count-1 in the vector).
                if (i == 6) { // CH7 -> B0-B4
                    for (int j = 0; j < 5; ++j) B[j] = sw_states[j]; 
                } else if (i == 7) { // CH8 -> T0-T2
                    for (int j = 0; j < 3; ++j) T[j] = sw_states[j];
                }

                if (mode == 2) {
                    ss << " " << channel_names_[i] << " | ";
                    for (int s : sw_states) ss << (s == 1 ? "UP " : (s == -1 ? "DN " : "-- "));
                    ss << "\n";
                }
            }
        }

        if (mode == 2) {
            ss << "----------------------------------------------------------------------------\n";
            // Logic for Enum Strings
            std::string mode_str = (T[1] == -1) ? "RETURN"  : (T[1] == 0) ? "MANUAL" : "OFFBOARD";
            std::string obj_str  = (T[0] == -1) ? "END_EFFECTOR"  : (T[0] == 0) ? "POSE"   : "NAVIGATE";
            std::string kill_str = (B[2] == -1) ? "KILL"      : "RUN";
            std::string trig_str = (B[0] == -1) ? "ON"       : "OFF";
            std::string sel_str  = (B[1] == -1) ? "SECONDARY" : "PRIMARY";

            char status[256];
            sprintf(status, " [STATE] MODE(T1): %-10s | OBJ(T0): %-12s\n", mode_str.c_str(), obj_str.c_str());
            ss << status;
            sprintf(status, " [HARD ] KILL(B2): %-10s | TRIG(B0): %-8s | SEL(B1): %-10s\n", 
                    kill_str.c_str(), trig_str.c_str(), sel_str.c_str());
            ss << status;
            ss << "----------------------------------------------------------------------------\n";
        }
        std::cout << ss.str() << std::flush;
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    std::vector<std::string> channel_names_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RadioDashboard>());
    rclcpp::shutdown();
    return 0;
}