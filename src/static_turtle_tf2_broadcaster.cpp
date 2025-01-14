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

#include <memory>
// Header files
#include "geometry_msgs/msg/transform_stamped.hpp" // TransformStamped message, used to publish to the tf2 transform tree
#include "rclcpp/rclcpp.hpp" // ROS Client Library for C++
#include "tf2/LinearMath/Quaternion.h" // Quaternion class, useful for converting between Euler angles and quaternions
#include "tf2_ros/static_transform_broadcaster.h" // StaticTransformBroadcaster class, used to publish static transforms


class StaticFramePublisher : public rclcpp::Node{
  public:
    // Constructor, initialises the node with the name "static_turtle_tf2_broadcaster"
    explicit StaticFramePublisher(char * transformation[])
    : Node("static_turtle_tf2_broadcaster")
    {
      // StaticTransformBroadcaster object to publish the static transform
      tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

      // Publish static transform once at startup
      this->make_transforms(transformation);
    }

  private:
    // Function to create and publish the static transform
    void make_transforms(char * transformation[])
    {
      // Creates a TransformStamped message and sends it to the tf2 transform tree once populated
      geometry_msgs::msg::TransformStamped t;
      // Before passing the transform values, populate it with the appropriate metadata
      // The timestamp is set to the current time
      t.header.stamp = this->get_clock()->now();
      // The parent frame of the transform is "world"
      t.header.frame_id = "world";
      // The child frame of the transform is passed as a parameter
      t.child_frame_id = transformation[1];
      // Populate the 6D pose (translation and rotation) of the transform
      t.transform.translation.x = atof(transformation[2]);
      t.transform.translation.y = atof(transformation[3]);
      t.transform.translation.z = atof(transformation[4]);
      // Convert the Euler angles to a quaternion and populate the rotation
      tf2::Quaternion q;
      // Set the roll, pitch, and yaw values from the command line arguments
      q.setRPY(
        atof(transformation[5]),
        atof(transformation[6]),
        atof(transformation[7]));
      // Set the quaternion values in the transform message
      t.transform.rotation.x = q.x();
      t.transform.rotation.y = q.y();
      t.transform.rotation.z = q.z();
      t.transform.rotation.w = q.w();
      // Publish the transform
      tf_static_broadcaster_->sendTransform(t);
    }
    // Shared pointer to the StaticTransformBroadcaster object
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};

int main(int argc, char * argv[])
{
  // Initialize the logger
  auto logger = rclcpp::get_logger("logger");

  // Obtain parameters from command line arguments
  if (argc != 8) {
    RCLCPP_INFO(
      logger, "Invalid number of parameters\nusage: "
      "$ ros2 run learning_tf2_cpp static_turtle_tf2_broadcaster "
      "child_frame_name x y z roll pitch yaw");
    return 1;
  }

  // As the parent frame of the transform is `world`, it is
  // necessary to check that the frame name passed is different
  if (strcmp(argv[1], "world") == 0) {
    RCLCPP_INFO(logger, "Your static turtle name cannot be named 'world'");
    return 2;
  }

  // Pass parameters and initialize the static broadcaster node
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StaticFramePublisher>(argv));
  // Shutdown the node
  rclcpp::shutdown();
  return 0;
}
