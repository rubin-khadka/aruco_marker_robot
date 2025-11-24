#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <set>
#include <vector>
#include <algorithm>
#include <cmath>

class SequentialCenteringScanner : public rclcpp::Node
{
public:
    SequentialCenteringScanner() : Node("sequential_centering_scanner")
    {
        // Parameters
        camera_topic = "/camera/image";
        imu_topic = "/imu";
        output_topic = "/processed_image";
        rotation_speed = 0.5;
        
        // Subscribe to camera and IMU
        image_sub = this->create_subscription<sensor_msgs::msg::Image>(
            camera_topic, 10,
            std::bind(&SequentialCenteringScanner::image_callback, this, std::placeholders::_1));
            
        imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, 10,
            std::bind(&SequentialCenteringScanner::imu_callback, this, std::placeholders::_1));
        
        // Publishers
        cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        image_pub = this->create_publisher<sensor_msgs::msg::Image>(output_topic, 10);
        
        // Initialize ArUco detector
        dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
        parameters = cv::aruco::DetectorParameters::create();
        
        // Set detection parameters
        parameters->adaptiveThreshWinSizeMin = 3;
        parameters->adaptiveThreshWinSizeMax = 23;
        parameters->adaptiveThreshConstant = 7.0;
        parameters->minMarkerPerimeterRate = 0.03;
        
        // State variables
        state = SCANNING;
        total_rotation = 0.0;
        last_imu_time = this->now();
        current_target_index = 0;
        is_centered = false;
        
        // Visual servoing parameters
        kp_angular = 0.01;
        center_threshold = 10.0;
        wait_duration = 10.0;
        
        RCLCPP_INFO(this->get_logger(), "=== SEQUENTIAL CENTERING SCANNER STARTED ===");
        RCLCPP_INFO(this->get_logger(), "State: SCANNING - Rotating 360° to detect all markers");
    }

private:
    enum State { SCANNING, SORTING, SEARCHING, CENTERING, COMPLETE };
    
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        if (state != SCANNING) return;
        
        auto current_time = this->now();
        double dt = (current_time - last_imu_time).seconds();
        last_imu_time = current_time;
        
        if (dt <= 0) return;
        
        // Integrate angular velocity for scanning
        double angular_vel_z = msg->angular_velocity.z;
        total_rotation += std::abs(angular_vel_z) * dt;
    }
    
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            cv::Mat image = cv_ptr->image;
            cv::Mat display_image = image.clone();
            
            // Detect markers
            std::vector<int> marker_ids;
            std::vector<std::vector<cv::Point2f>> marker_corners;
            cv::aruco::detectMarkers(image, dictionary, marker_corners, marker_ids, parameters);
            
            // State machine
            switch (state) {
                case SCANNING:
                    handle_scanning_state(marker_ids, marker_corners, display_image);
                    break;
                case SORTING:
                    handle_sorting_state(display_image);
                    break;
                case SEARCHING:
                    handle_searching_state(marker_ids, marker_corners, display_image);
                    break;
                case CENTERING:
                    handle_centering_state(marker_ids, marker_corners, display_image, msg->header);
                    break;
                case COMPLETE:
                    handle_complete_state(display_image, msg->header);
                    break;
            }
            
            cv::imshow("Sequential Centering Scanner", display_image);
            cv::waitKey(1);
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
        }
    }
    
    void handle_scanning_state(const std::vector<int>& marker_ids,
                              const std::vector<std::vector<cv::Point2f>>& marker_corners,
                              cv::Mat& display_image)
    {
        // Add detected markers to our set
        for (int id : marker_ids) {
            if (detected_marker_ids.insert(id).second) {
                RCLCPP_INFO(this->get_logger(), "Discovered marker: ID %d", id);
            }
        }
        
        // Draw detected markers
        if (!marker_ids.empty()) {
            cv::aruco::drawDetectedMarkers(display_image, marker_corners, marker_ids);
        }
        
        // Continue rotating
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.angular.z = rotation_speed;
        cmd_vel_pub->publish(twist_msg);
        
        // Check if scanning is complete
        if (total_rotation >= 2.0 * M_PI) {
            state = SORTING;
            auto twist_msg = geometry_msgs::msg::Twist();
            cmd_vel_pub->publish(twist_msg);
            
            RCLCPP_INFO(this->get_logger(), "=== SCANNING COMPLETE ===");
            RCLCPP_INFO(this->get_logger(), "Found %zu markers", detected_marker_ids.size());
        }
        
        cv::putText(display_image, "SCANNING: Rotating 360°", cv::Point(10, 30),
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
        cv::putText(display_image, "Detected: " + std::to_string(detected_marker_ids.size()) + " markers", 
                   cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
    }
    
    void handle_sorting_state(cv::Mat& display_image)
    {
        // Sort the detected markers
        sorted_marker_ids.assign(detected_marker_ids.begin(), detected_marker_ids.end());
        std::sort(sorted_marker_ids.begin(), sorted_marker_ids.end());
        
        RCLCPP_INFO(this->get_logger(), "=== SORTING COMPLETE ===");
        RCLCPP_INFO(this->get_logger(), "Marker visitation order:");
        for (size_t i = 0; i < sorted_marker_ids.size(); i++) {
            RCLCPP_INFO(this->get_logger(), "  %zu: ID %d", i + 1, sorted_marker_ids[i]);
        }
        
        if (!sorted_marker_ids.empty()) {
            current_target_index = 0;
            state = SEARCHING;
            is_centered = false;
            RCLCPP_INFO(this->get_logger(), "Starting search for marker ID: %d", 
                       sorted_marker_ids[current_target_index]);
        } else {
            state = COMPLETE;
            RCLCPP_INFO(this->get_logger(), "No markers found!");
        }
        
        cv::putText(display_image, "SORTING: Found " + std::to_string(sorted_marker_ids.size()) + " markers", 
                   cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 165, 0), 2);
    }
    
    void handle_searching_state(const std::vector<int>& marker_ids,
                               const std::vector<std::vector<cv::Point2f>>& marker_corners,
                               cv::Mat& display_image)
    {
        int target_id = sorted_marker_ids[current_target_index];
        
        // Check if target marker is in view
        auto it = std::find(marker_ids.begin(), marker_ids.end(), target_id);
        if (it != marker_ids.end()) {
            // Found target! Transition to centering
            state = CENTERING;
            is_centered = false;
            center_start_time = this->now();
            RCLCPP_INFO(this->get_logger(), "=== FOUND TARGET MARKER ID: %d ===", target_id);
            
            cv::aruco::drawDetectedMarkers(display_image, marker_corners, marker_ids);
            
        } else {
            // Continue searching by rotating
            auto twist_msg = geometry_msgs::msg::Twist();
            twist_msg.angular.z = 0.3;
            cmd_vel_pub->publish(twist_msg);
            
            if (!marker_ids.empty()) {
                cv::aruco::drawDetectedMarkers(display_image, marker_corners, marker_ids);
            }
        }
        
        cv::putText(display_image, "SEARCHING: Marker ID " + std::to_string(target_id), 
                   cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 165, 255), 2);
        cv::putText(display_image, "Progress: " + std::to_string(current_target_index + 1) + 
                   "/" + std::to_string(sorted_marker_ids.size()), 
                   cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 165, 255), 2);
    }
    
    void handle_centering_state(const std::vector<int>& marker_ids,
                            const std::vector<std::vector<cv::Point2f>>& marker_corners,
                            cv::Mat& display_image,
                            const std_msgs::msg::Header& header)
    {
        int target_id = sorted_marker_ids[current_target_index];
        auto current_time = this->now();
        
        // Look for the target marker
        auto it = std::find(marker_ids.begin(), marker_ids.end(), target_id);
        
        if (it != marker_ids.end()) {
            size_t idx = std::distance(marker_ids.begin(), it);
            
            // Calculate marker center more precisely
            cv::Point2f center(0, 0);
            for (const auto& corner : marker_corners[idx]) {
                center.x += corner.x;
                center.y += corner.y;
            }
            center.x /= 4.0;
            center.y /= 4.0;
            
            // Calculate image center
            cv::Point2f image_center(display_image.cols / 2.0, display_image.rows / 2.0);
            double error_x = center.x - image_center.x;
            
            // Create processed image with SMALLER circle on marker pattern
            cv::Mat processed_image = display_image.clone();
            cv::aruco::drawDetectedMarkers(processed_image, marker_corners, marker_ids);
            
            // OPTION 1 + 2: Smaller circle radius (25 instead of 40) on precise center
            cv::circle(processed_image, center, 25, cv::Scalar(0, 255, 0), 3);
            
            if (!is_centered) {
                // Still centering - use visual servoing
                if (std::abs(error_x) > center_threshold) {
                    auto twist_msg = geometry_msgs::msg::Twist();
                    twist_msg.angular.z = -kp_angular * error_x;
                    cmd_vel_pub->publish(twist_msg);
                    
                    cv::putText(display_image, "CENTERING: Marker ID " + std::to_string(target_id), 
                            cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
                    cv::putText(display_image, "Error X: " + std::to_string((int)error_x) + " pixels", 
                            cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
                    
                } else {
                    // Just centered! Stop robot and start dwell time
                    is_centered = true;
                    center_start_time = current_time;
                    auto twist_msg = geometry_msgs::msg::Twist();
                    cmd_vel_pub->publish(twist_msg);
                    
                    RCLCPP_INFO(this->get_logger(), "=== MARKER %d CENTERED! Starting 10-second dwell ===", target_id);
                }
            }
            
            if (is_centered) {
                // Already centered - just maintain position and count down
                double elapsed = (current_time - center_start_time).seconds();
                double time_left = wait_duration - elapsed;
                
                // Draw smaller circle on display image too
                cv::circle(display_image, center, 25, cv::Scalar(0, 255, 0), 3);
                cv::circle(display_image, image_center, 5, cv::Scalar(255, 0, 0), -1);
                cv::line(display_image, image_center, center, cv::Scalar(0, 255, 0), 2);
                
                cv::putText(display_image, "CENTERED: Marker ID " + std::to_string(target_id), 
                        cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
                cv::putText(display_image, "Time left: " + std::to_string((int)time_left) + " seconds", 
                        cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
                
                // Keep robot stopped
                auto twist_msg = geometry_msgs::msg::Twist();
                cmd_vel_pub->publish(twist_msg);
                
                // Check if dwell time is complete
                if (elapsed >= wait_duration) {
                    RCLCPP_INFO(this->get_logger(), "=== Dwell complete. Moving to next marker ===");
                    
                    current_target_index++;
                    if (current_target_index < sorted_marker_ids.size()) {
                        state = SEARCHING;
                        is_centered = false;
                        RCLCPP_INFO(this->get_logger(), "Searching for next marker: ID %d", 
                                sorted_marker_ids[current_target_index]);
                    } else {
                        state = COMPLETE;
                        RCLCPP_INFO(this->get_logger(), "=== ALL MARKERS COMPLETED ===");
                    }
                }
            }
            
            // PUBLISH PROCESSED IMAGE EVERY FRAME (with SMALLER circle)
            publish_processed_image(processed_image, header);
            
        } else {
            // Lost the marker - go back to searching
            state = SEARCHING;
            is_centered = false;
            auto twist_msg = geometry_msgs::msg::Twist();
            twist_msg.angular.z = 0.3;
            cmd_vel_pub->publish(twist_msg);
        }
    }
    
    void handle_complete_state(cv::Mat& display_image, const std_msgs::msg::Header& header)
    {
        cv::putText(display_image, "MISSION COMPLETE!", cv::Point(10, 30),
                   cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
        cv::putText(display_image, "All markers centered in order", cv::Point(10, 60),
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        
        // Create and publish final processed image
        cv::Mat processed_image = display_image.clone();
        publish_processed_image(processed_image, header);
        
        auto twist_msg = geometry_msgs::msg::Twist();
        cmd_vel_pub->publish(twist_msg);
    }
    
    void publish_processed_image(const cv::Mat& image, const std_msgs::msg::Header& header)
    {
        auto msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
        image_pub->publish(*msg);
    }
    
    // Member variables
    std::string camera_topic;
    std::string imu_topic;
    std::string output_topic;
    double rotation_speed;
    double kp_angular;
    double center_threshold;
    double wait_duration;
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;
    
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Ptr<cv::aruco::DetectorParameters> parameters;
    
    State state;
    double total_rotation;
    rclcpp::Time last_imu_time;
    rclcpp::Time center_start_time;
    
    std::set<int> detected_marker_ids;
    std::vector<int> sorted_marker_ids;
    size_t current_target_index;
    bool is_centered;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SequentialCenteringScanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}