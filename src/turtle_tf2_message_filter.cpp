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
#include <memory>
#include <string>
// Headers
#include "geometry_msgs/msg/point_stamped.hpp" // PointStamped message
#include "message_filters/subscriber.h" // MessageFilter subscriber
#include "rclcpp/rclcpp.hpp" // ROS2 C++ client library
#include "tf2_ros/buffer.h" // Buffer for storing transforms
#include "tf2_ros/create_timer_ros.h" // Timer interface for tf2_ros
#include "tf2_ros/message_filter.h" // MessageFilter for filtering messages
#include "tf2_ros/transform_listener.h" // TransformListener for listening to transforms
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" // Transform geometry_msgs to tf2
using namespace std::chrono_literals;

class PoseDrawer : public rclcpp::Node
{
public:
  // Constructor, create a node named "turtle_tf2_pose_drawer"
  PoseDrawer()
  : Node("turtle_tf2_pose_drawer")
  {
    // Declare and acquire `target_frame` parameter
    target_frame_ = this->declare_parameter<std::string>("target_frame", "turtle1");
    // Set the buffer timeout to 1 second
    std::chrono::duration<int> buffer_timeout(1);
    // Create a buffer to store the transform data
    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    // Create the timer interface before call to waitForTransform,
    // to avoid a tf2_ros::CreateTimerInterfaceException exception
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(),
      this->get_node_timers_interface());
    // Set the timer interface in the buffer
    tf2_buffer_->setCreateTimerInterface(timer_interface);
    // Create a TransformListener to listen to the transform data
    tf2_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
    // Create a message filter to subscribe to the turtle3/turtle_point_stamped topic
    point_sub_.subscribe(this, "/turtle3/turtle_point_stamped");
    tf2_filter_ = std::make_shared<tf2_ros::MessageFilter<geometry_msgs::msg::PointStamped>>(
      point_sub_, *tf2_buffer_, target_frame_, 100, this->get_node_logging_interface(),
      this->get_node_clock_interface(), buffer_timeout);
    // Register a callback with tf2_ros::MessageFilter to be called when transforms are available
    tf2_filter_->registerCallback(&PoseDrawer::msgCallback, this);
  }

private:
  // Callback function to be called when a message is received
  void msgCallback(const geometry_msgs::msg::PointStamped::SharedPtr point_ptr)
  {
    // Create a PointStamped message to store the transformed point
    geometry_msgs::msg::PointStamped point_out;
    try {
      // Transform the point from the turtle3 frame to the turtle1 frame when the data is ready
      tf2_buffer_->transform(*point_ptr, point_out, target_frame_);
      // Print the transformed point to the console
      RCLCPP_INFO(
        this->get_logger(), "Point of turtle3 in frame of turtle1: x:%f y:%f z:%f\n",
        point_out.point.x,
        point_out.point.y,
        point_out.point.z);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        // Print exception which was caught
        this->get_logger(), "Failure %s\n", ex.what());
    }
  }
  // Member variables
  // There must be persistent instances of the buffer, listener, and message filter
  // So that the callback can access them, thus they are shared pointers
  std::string target_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  message_filters::Subscriber<geometry_msgs::msg::PointStamped> point_sub_;
  std::shared_ptr<tf2_ros::MessageFilter<geometry_msgs::msg::PointStamped>> tf2_filter_;
};

int main(int argc, char * argv[])
{
  // Initialize the ROS2 client library
  rclcpp::init(argc, argv);
  // Create a PoseDrawer node instance
  rclcpp::spin(std::make_shared<PoseDrawer>());
  // Shutdown the ROS2 client library
  rclcpp::shutdown();
  return 0;
}
