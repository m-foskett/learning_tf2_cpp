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
#include <string>
// Header files
#include "geometry_msgs/msg/transform_stamped.hpp" // TransformStamped message, used to publish to the tf2 transform tree
#include "geometry_msgs/msg/twist.hpp" // Twist message, used to publish velocity commands
#include "rclcpp/rclcpp.hpp" // ROS Client Library for C++
#include "tf2/exceptions.h" // TransformException class, used to handle exceptions
#include "tf2_ros/transform_listener.h" // TransformListener class, used to listen for transformations
#include "tf2_ros/buffer.h" // Buffer class, used to store transformations
#include "turtlesim/srv/spawn.hpp" // Spawn service, used to spawn turtles

// chrono_literals for time literals
using namespace std::chrono_literals;

class FrameListener : public rclcpp::Node
{
public:
  // Constructor, initialises the node with the name "turtle_tf2_frame_listener"
  FrameListener()
  : Node("turtle_tf2_frame_listener"),
    turtle_spawning_service_ready_(false),
    turtle_spawned_(false)
  {
    // Declare and acquire `target_frame` parameter
    target_frame_ = this->declare_parameter<std::string>("target_frame", "turtle1");
    // Initialize the transform buffer
    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    // Initialize the transform listener, starts receiving transformations
    // - buffers transformations for up to 10 seconds
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Create a client to spawn a turtle
    spawner_ =
      this->create_client<turtlesim::srv::Spawn>("spawn");

    // Create turtle2 velocity publisher
    publisher_ =
      this->create_publisher<geometry_msgs::msg::Twist>("turtle2/cmd_vel", 1);

    // Call on_timer function every second
    timer_ = this->create_wall_timer(
      1s, [this]() {return this->on_timer();});
  }

private:
  void on_timer()
  {
    // Store frame names in variables that will be used to
    // compute transformations
    std::string fromFrameRel = target_frame_.c_str();
    std::string toFrameRel = "turtle2";
    // Check if the turtle was spawned
    if (turtle_spawning_service_ready_) {
      if (turtle_spawned_) {
        // Create a TransformStamped message
        geometry_msgs::msg::TransformStamped t;

        // Get the current time
        rclcpp::Time now = this->get_clock()->now();
        // Get the time to transform the data from
        rclcpp::Time when = now - rclcpp::Duration(5, 0);
        // tf2 performs the following steps:
        // 1. Compute the transform from the source frame to the fixed frame in the past
        // 2. In the fixed frame, time travel from the past to now
        // 3. Compute the transform from the fixed frame to the target frame
        try {
          t = tf_buffer_->lookupTransform(
            toFrameRel, // Target frame
            now, // The time to transform the data to
            fromFrameRel, // The source frame to transform the data from
            when, // The time at which the source frame will be evaluated 
            "world", // Fixed frame (doesn't change over time)
            50ms // Timeout duration to wait for the transform to become available
          );
        } catch (const tf2::TransformException & ex) {
          // Handle exceptions
          RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
          return;
        }
        // Create a Twist message
        geometry_msgs::msg::Twist msg;
        // Compute the angular velocity
        static const double scaleRotationRate = 1.0;
        msg.angular.z = scaleRotationRate * atan2(
          t.transform.translation.y,
          t.transform.translation.x);
        // Compute the linear velocity
        static const double scaleForwardSpeed = 0.5;
        msg.linear.x = scaleForwardSpeed * sqrt(
          pow(t.transform.translation.x, 2) +
          pow(t.transform.translation.y, 2));
        // Publish the velocity command
        publisher_->publish(msg);
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
        request->x = 4.0;
        request->y = 2.0;
        request->theta = 0.0;
        request->name = "turtle2";

        // Call request
        using ServiceResponseFuture =
          rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture;
        auto response_received_callback = [this](ServiceResponseFuture future) {
            auto result = future.get();
            if (strcmp(result->name.c_str(), "turtle2") == 0) {
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
  }

  // Boolean values to store the information
  // if the service for spawning turtle is available
  bool turtle_spawning_service_ready_;
  // if the turtle was successfully spawned
  bool turtle_spawned_;
  rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawner_{nullptr};
  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string target_frame_;
};

int main(int argc, char * argv[])
{
  // Initialize the ROS client library
  rclcpp::init(argc, argv);
  // Create the FrameListener node
  rclcpp::spin(std::make_shared<FrameListener>());
  // Shutdown the ROS client library
  rclcpp::shutdown();
  return 0;
}
