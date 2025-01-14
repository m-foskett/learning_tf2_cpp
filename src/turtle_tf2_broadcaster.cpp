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

#include <functional>
#include <memory>
#include <sstream>
#include <string>
// Header files
#include "geometry_msgs/msg/transform_stamped.hpp" // TransformStamped message, used to publish to the tf2 transform tree
#include "rclcpp/rclcpp.hpp" // ROS Client Library for C++
#include "tf2/LinearMath/Quaternion.h" // Quaternion class, useful for converting between Euler angles and quaternions
#include "tf2_ros/transform_broadcaster.h" // TransformBroadcaster class, used to publish transforms
#include "turtlesim/msg/pose.hpp" // Pose message, used to subscribe to turtle pose messages

class FramePublisher : public rclcpp::Node
{
public:
  // Constructor, initialises the node with the name "turtle_tf2_frame_publisher"
  FramePublisher()
  : Node("turtle_tf2_frame_publisher")
  {
    // Declare and acquire `turtlename` parameter
    turtlename_ = this->declare_parameter<std::string>("turtlename", "turtle");

    // Initialize the transform broadcaster
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Subscribe to a turtle{1}{2}/pose topic and call handle_turtle_pose
    // callback function on each message
    std::ostringstream stream;
    stream << "/" << turtlename_.c_str() << "/pose";
    std::string topic_name = stream.str();
    // Lambda function to handle turtle pose messages
    auto handle_turtle_pose = [this](const std::shared_ptr<turtlesim::msg::Pose> msg) {
        // Create a TransformStamped message
        geometry_msgs::msg::TransformStamped t;

        // Read message content and assign it to
        // corresponding tf variables
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "world";
        t.child_frame_id = turtlename_.c_str();

        // Turtle only exists in 2D, thus we get x and y translation
        // coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = msg->x;
        t.transform.translation.y = msg->y;
        t.transform.translation.z = 0.0;

        // For the same reason, turtle can only rotate around one axis
        // and this why we set rotation in x and y to 0 and obtain
        // rotation in z axis from the message
        tf2::Quaternion q;
        q.setRPY(0, 0, msg->theta);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        // Send the transformation
        tf_broadcaster_->sendTransform(t);
      };
    // Create a subscription to the turtle pose topic
    // Runs function handle_turtle_pose on each incoming message
    subscription_ = this->create_subscription<turtlesim::msg::Pose>(
      topic_name, 10,
      handle_turtle_pose);
  }

private:
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::string turtlename_;
};

int main(int argc, char * argv[])
{
  // Pass parameters and initialise the dynamic broadcaster node
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FramePublisher>());
  // Shutdown the node
  rclcpp::shutdown();
  return 0;
}
