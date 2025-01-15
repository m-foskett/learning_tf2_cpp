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
// Headers
#include "geometry_msgs/msg/transform_stamped.hpp" // TransformStamped message, used to publish to the tf2 transform tree
#include "rclcpp/rclcpp.hpp" // ROS Client Library for C++
#include "tf2_ros/transform_broadcaster.h" // TransformBroadcaster class, used to publish transforms
// chrono_literals for time literals
using namespace std::chrono_literals;

class FixedFrameBroadcaster : public rclcpp::Node{
  public:
    // Constructor, initialises the node with the name "fixed_frame_tf2_broadcaster"/
    FixedFrameBroadcaster()
    : Node("fixed_frame_tf2_broadcaster")
    {
      // Initialize the transform broadcaster
      tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
      // Call broadcast_timer_callback function every 100 milliseconds
      timer_ = this->create_wall_timer(
        100ms, std::bind(&FixedFrameBroadcaster::broadcast_timer_callback, this));
    }

  private:
    // Function to broadcast a fixed frame
    void broadcast_timer_callback()
    {
      // Create a TransformStamped message
      geometry_msgs::msg::TransformStamped t;
      // Set the header of the message
      t.header.stamp = this->get_clock()->now();
      // Set the frame_id and child_frame_id of the message
      t.header.frame_id = "turtle1";
      t.child_frame_id = "carrot1";
      // Set the translation and rotation of the message
      t.transform.translation.x = 0.0;
      t.transform.translation.y = 2.0;
      t.transform.translation.z = 0.0;
      t.transform.rotation.x = 0.0;
      t.transform.rotation.y = 0.0;
      t.transform.rotation.z = 0.0;
      t.transform.rotation.w = 1.0;
      // Broadcast the message
      tf_broadcaster_->sendTransform(t);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
  // Initialize the ROS client library
  rclcpp::init(argc, argv);
  // Create the FixedFrameBroadcaster node
  rclcpp::spin(std::make_shared<FixedFrameBroadcaster>());
  // Shutdown the ROS client library
  rclcpp::shutdown();
  return 0;
}
