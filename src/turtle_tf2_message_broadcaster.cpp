// Copyright 2021 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
// Header files
#include "geometry_msgs/msg/point_stamped.hpp" // PointStamped message, used to publish to the tf2 transform tree
#include "geometry_msgs/msg/twist.hpp" // Twist message, used to publish velocity commands
#include "rclcpp/rclcpp.hpp" // ROS Client Library for C++
#include "turtlesim/msg/pose.hpp" // Pose message, used to subscribe to turtle pose messages
#include "turtlesim/srv/spawn.hpp" // Spawn service, used to spawn turtles

// chrono_literals for time literals
using namespace std::chrono_literals;

class PointPublisher : public rclcpp::Node{
    public:
        // Constructor, initialises the node with the name "turtle_tf2_message_broadcaster"
        PointPublisher()
        : Node("turtle_tf2_message_broadcaster"),
        turtle_spawning_service_ready_(false),
        turtle_spawned_(false),
        turtle_pose_cansubscribe_(false)
        {
            // Create a client to spawn a turtle
            spawner_ = this->create_client<turtlesim::srv::Spawn>("spawn");
            // Call on_timer function every second
            timer_ = this->create_wall_timer(1s, [this]() {return this->on_timer();});
        }
    private:
        void on_timer()
        {
            // Check if the turtle was spawned
            if (turtle_spawning_service_ready_) {
                if (turtle_spawned_) {
                    // Set the turtle_pose_cansubscribe_ flag to true
                    turtle_pose_cansubscribe_ = true;
                } else {
                    // Inform that the turtle was spawned
                    RCLCPP_INFO(this->get_logger(), "Successfully spawned");
                    // Set the turtle_spawned_ flag to true
                    turtle_spawned_ = true;
                }
            } else {
                // Check if the spawning service is ready
                if (spawner_->service_is_ready()) {
                    // Initialize request with turtle name and coordinates
                    // Note that x, y and theta are defined as floats in turtlesim/srv/Spawn
                    auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
                    request->name = "turtle3";
                    request->x = 4.0;
                    request->y = 2.0;
                    request->theta = 0.0;
                    // Call request
                    using ServiceResponseFuture =
                    rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture;
                    auto response_received_callback = [this](ServiceResponseFuture future) {
                        auto result = future.get();
                        if (strcmp(result->name.c_str(), "turtle3") == 0) {
                        turtle_spawning_service_ready_ = true;
                        } else {
                        RCLCPP_ERROR(this->get_logger(), "Service callback result mismatch");
                        }
                    };
                    auto result = spawner_->async_send_request(request, response_received_callback);
                } else {
                    RCLCPP_INFO(this->get_logger(), "Service is not ready");
                }
            }
            if (turtle_pose_cansubscribe_){
                // Lambda function to handle turtle pose messages
                auto handle_turtle_pose = [this](const std::shared_ptr<turtlesim::msg::Pose> msg) {
                    // Create a Twist message
                    geometry_msgs::msg::Twist vel_msg;
                    // Compute the angular velocity
                    vel_msg.linear.x = 1.0;
                    vel_msg.angular.z = 1.0;
                    // Publish the velocity command
                    vel_publisher_->publish(vel_msg);
                    // Create a PointStamped message
                    geometry_msgs::msg::PointStamped point_msg;
                    point_msg.header.stamp = this->get_clock()->now();
                    point_msg.header.frame_id = "world";
                    point_msg.point.x = msg->x;
                    point_msg.point.y = msg->y;
                    point_msg.point.z = 0.0;
                    // Publish the point message
                    point_publisher_->publish(point_msg);
                };
                // Create a velocity publisher
                vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
                    "turtle3/cmd_vel", 10);
                // Create a pose subscriber
                pose_subscriber_= this->create_subscription<turtlesim::msg::Pose>(
                    "turtle3/pose", 10, handle_turtle_pose);
                // Create a PointStamped publisher
                point_publisher_= this->create_publisher<geometry_msgs::msg::PointStamped>(
                    "turtle3/turtle_point_stamped", 10);
            }
        }
        // Boolean values to store the information
        // if the service for spawning turtle is available
        bool turtle_spawning_service_ready_;
        // if the turtle was successfully spawned
        bool turtle_spawned_;
        // if the topics of turtle3 can be subscribed to
        bool turtle_pose_cansubscribe_;
        rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawner_{nullptr};
        rclcpp::TimerBase::SharedPtr timer_{nullptr};
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_{nullptr};
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_publisher_{nullptr};
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscriber_{nullptr};
};

int main(int argc, char * argv[])
{
  // Pass parameters and initialise the dynamic broadcaster node
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointPublisher>());
  // Shutdown the node
  rclcpp::shutdown();
  return 0;
}
