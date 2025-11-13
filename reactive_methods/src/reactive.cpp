#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <opencv2/opencv.hpp>
#include <utility>
#include <cmath>
/// CHECK: include needed ROS msg type headers and libraries

class ReactiveFollowGap : public rclcpp::Node {
// Implement Reactive Follow Gap on the car
// This is just a template, you are free to implement your own node!

public:
    ReactiveFollowGap() : Node("reactive_node")
    {
        drive_publisher = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic,10);
        laser_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(lidarscan_topic,10,
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg){lidar_callback(msg)});
    
    }

private:
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr laser_subscriber;
    float VISION_ANGLE_MIN = -1.5707;
    float VISION_ANGLE_MAX = 1.5707;
    int CONV_SIZE = 3;
    int MAX_LIDAR_DIST = 3;  // in m

    cv::Mat preprocess_lidar(float* ranges, float rad_increment, int length, float rad_min, float rad_max)
    {   
        //coverting to a cv::Mat will make calculations easier
        cv::Mat ranges_mat(1, length, CV_32F, ranges);

        //slicing the lidar ranges to a specific range to only use what in front of the vehicle
        int min_index = std::round(VISION_ANGLE_MIN - rad_min / rad_increment);
        int max_index = std::round(VISION_ANGLE_MAX - rad_max / rad_increment);
        cv::Mat sliced_ranges_mat = ranges_mat.colRange(min_index, length + max_index);

        //reducing noise in lidar data by setting each value to a mean
        cv::Mat kernel = cv::Mat::ones(1,CONV_SIZE, CV_32F);
        cv::Mat convolution;
        cv::filter2D(sliced_ranges_mat, convolution, -1, kernel);
        convolution /= CONV_SIZE;
        
        //clip ranges to a set max value and change all negative values to 0
        cv::max(convolution,0,convolution);
        cv::min(convolution,MAX_LIDAR_DIST,convolution);

        return convolution;
    }

    void find_max_gap(cv::Mat ranges, int* indices)
    {   int current_gap = 0;
        int max_gap = 0;
        int end_i = 0;

        //finds where largest gap of space away from nearest obstacle
        for (int i = 0; i<ranges.cols; i++) {
            if (ranges.at<float>(0,i) != 0){
                current_gap++;
            } else {
                if (current_gap > max_gap) {
                    max_gap = current_gap;
                    end_i = i;
                    current_gap = 0;
                }
            }
        }
        
        if (current_gap > max_gap) {
            max_gap = current_gap;
            end_i = i;
        }
        
        //modifies the passed in indices array to the start and end indices of the max gap
        indices[0] = end_i - max_gap;
        indices[1] =  end_i;
    }

    int find_best_point(float* ranges, int* indices)
    {   
        //returns index center of max gap
        return (indices[0] + indices[1])/2;
    }

    float get_angle(int index, int ranges_length, float angle_increment){
        //the max and min angles for our cutoff are the same, so the forward facing angle, 0, should be the midpoint
        int midpoint = ranges_length/2;
        //get index distance from the 0 angle, and multiply by the angle increment
        float angle = (index - midpoint) * angle_increment;

        //divide by 2 for steering angle
        return angle/2;
    }


    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {   
        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message

        /// TODO:
        // Find closest point to LiDAR

        // Eliminate all points inside 'bubble' (set them to zero) 

        // Find max length gap 

        // Find the best point in the gap 

        // Publish Drive message
    }



};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}