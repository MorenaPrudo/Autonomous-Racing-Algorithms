#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include <iostream>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <opencv2/opencv.hpp>
#include <utility>
#include <cmath>

class ReactiveFollowGap : public rclcpp::Node {

public:
    ReactiveFollowGap() : Node("reactive_node")
    {
        drive_publisher = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic,10);
        laser_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(lidarscan_topic,10,
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg){lidar_callback(msg);});
    
    }

private:
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber;
    float VISION_ANGLE_MIN = -2.2;
    float VISION_ANGLE_MAX = 2.2;
    int CONV_SIZE = 5;
    int BEST_POINT_CONV_SIZE = 75;
    int MAX_LIDAR_DIST = 6;  // in m
    float BUBBLE_RADIUS = 0.5; // in m
    float SPEED = 3;
    float CORNER_ANGLE = 0.175;
    float CORNER_SPEED = 1;

    cv::Mat preprocess_lidar(std::vector<float> ranges, float rad_increment, int length, float rad_min, float rad_max)
    {   
        //coverting to a cv::Mat will make calculations easier
        cv::Mat ranges_mat = cv::Mat(ranges).t();

        //slicing the lidar ranges to a specific range to only use what in front of the vehicle
        int min_index = std::round((VISION_ANGLE_MIN - rad_min) / rad_increment);
        int max_index = std::round((VISION_ANGLE_MAX - rad_max) / rad_increment);
        cv::Mat sliced_ranges_mat = ranges_mat.colRange(std::max(0,min_index), std::min(ranges_mat.cols, length + max_index));
       
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
            end_i = ranges.cols - 1;
        }
        
        //modifies the passed in indices array to the start and end indices of the max gap
        indices[0] = end_i - max_gap;
        indices[1] =  end_i;

    }

    int find_best_point(int* indices, cv::Mat ranges)
    {   
        cv::Mat gap = ranges.colRange(indices[0], indices[1]);
        
        cv::Mat kernel = cv::Mat::ones(1,BEST_POINT_CONV_SIZE, CV_32F);
        cv::Mat convolution;
        cv::filter2D(gap, convolution, -1, kernel);
        convolution /= BEST_POINT_CONV_SIZE;
        
        cv::Point max_location;
        cv::minMaxLoc(convolution, nullptr, nullptr, nullptr, &max_location);
        int max_i = max_location.x + indices[0];
        return max_i;
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
        float angle_increment = scan_msg->angle_increment;
        float angle_min = scan_msg->angle_min;
        float angle_max = scan_msg->angle_max;
        std::vector<float> ranges = scan_msg->ranges;
        int length = ranges.size();

        cv::Mat processed_ranges = preprocess_lidar(ranges,angle_increment,length,angle_min,angle_max);
        length = processed_ranges.cols;

        //create a safety bubble around the closest obstacle
        double min;
        cv::Point min_location;
        cv::minMaxLoc(processed_ranges, &min, nullptr, &min_location, nullptr);
        int min_i = min_location.x;

        //get angular size of bubble 
        //note the factor of 2 was omitted as it would be added to
        //both sides of closest obtacle location
        float angular_size = std::atan(BUBBLE_RADIUS/(float)min);
        
        //get angular size as indices
        int indices = std::ceil(angular_size/angle_increment);

        //make sure bubble indices wont be out of bounds
        int bubble_min_i = std::max(0,min_i - indices);
        int bubble_max_i = std::min(length-1, indices + min_i);

        //zero out all values in safety bubble
        processed_ranges.colRange(bubble_min_i, bubble_max_i) = 0;

        //get indices of the max gap (largest gap of freespace around the closest obstacle)
        int gap_indices[] = {0,0};
        find_max_gap(processed_ranges,gap_indices);

        //get steering angle from best point in max gap
        float steering_angle = get_angle(find_best_point(gap_indices,processed_ranges),length,angle_increment);

        //publish an Ackermann Drive Stamped message
        auto msg = ackermann_msgs::msg::AckermannDriveStamped();
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "base_link";

        msg.drive.speed = SPEED;
        msg.drive.steering_angle = steering_angle;
        if (std::abs(steering_angle) > CORNER_ANGLE){
           msg.drive.speed = CORNER_SPEED; 
        }

        drive_publisher->publish(msg);

        


    }



};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}