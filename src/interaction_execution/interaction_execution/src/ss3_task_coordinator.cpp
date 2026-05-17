#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

#include <vector>
#include <string>
#include <sstream>
#include <algorithm>
#include <cmath>

class SS3TaskCoordinator : public rclcpp::Node
{
public:
  SS3TaskCoordinator() : Node("ss3_task_coordinator")
  {
    detected_objects_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/raw_detected_objects", 10,
      std::bind(&SS3TaskCoordinator::detectedObjectsCallback, this, std::placeholders::_1));

    object_labels_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/object_labels", 10,
      std::bind(&SS3TaskCoordinator::objectLabelsCallback, this, std::placeholders::_1));

    start_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/client/start", 10,
      std::bind(&SS3TaskCoordinator::startCallback, this, std::placeholders::_1));

    ordered_objects_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
      "/ordered_detected_objects", 10);

    RCLCPP_INFO(this->get_logger(), "SS3 Task Coordinator ready. Waiting for SS1 data and /client/start...");
  }

private:
  geometry_msgs::msg::PoseArray latest_objects_;
  std::vector<std::string> latest_labels_;

  bool objects_received_ = false;
  bool labels_received_ = false;

  void detectedObjectsCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    latest_objects_ = *msg;
    objects_received_ = true;

    RCLCPP_INFO(this->get_logger(), "Received %zu detected object poses.", latest_objects_.poses.size());
  }

  void objectLabelsCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    latest_labels_ = splitLabels(msg->data);
    labels_received_ = true;

    RCLCPP_INFO(this->get_logger(), "Received object labels: %s", msg->data.c_str());
  }

  void startCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data)
      return;

    RCLCPP_INFO(this->get_logger(), "Client START received.");

    if (!objects_received_ || !labels_received_)
    {
      RCLCPP_WARN(this->get_logger(), "Cannot start: waiting for detected objects and labels.");
      return;
    }

    if (latest_objects_.poses.size() != latest_labels_.size())
    {
      RCLCPP_ERROR(this->get_logger(), "Pose count and label count mismatch.");
      return;
    }

    auto ordered_msg = generateOrderedPoseArray();
    ordered_objects_pub_->publish(ordered_msg);

    RCLCPP_INFO(this->get_logger(), "Published ordered detected objects to SS2.");
  }

  std::vector<std::string> splitLabels(const std::string &label_string)
  {
    std::vector<std::string> labels;
    std::stringstream ss(label_string);
    std::string label;

    while (std::getline(ss, label, ','))
    {
      label.erase(remove(label.begin(), label.end(), ' '), label.end());
      labels.push_back(label);
    }

    return labels;
  }

  double planarDistance(const geometry_msgs::msg::Pose &pose)
  {
    return std::sqrt(
      pose.position.x * pose.position.x +
      pose.position.y * pose.position.y);
  }

  geometry_msgs::msg::PoseArray generateOrderedPoseArray()
  {
    geometry_msgs::msg::PoseArray ordered_msg;
    ordered_msg.header = latest_objects_.header;

    std::vector<size_t> red_indices;
    std::vector<size_t> blue_indices;
    std::vector<size_t> yellow_indices;

    for (size_t i = 0; i < latest_labels_.size(); ++i)
    {
      if (latest_labels_[i] == "red")
        red_indices.push_back(i);
      else if (latest_labels_[i] == "blue")
        blue_indices.push_back(i);
      else if (latest_labels_[i] == "yellow")
        yellow_indices.push_back(i);
    }

    sortByNearest(red_indices);
    sortByNearest(blue_indices);
    sortByNearest(yellow_indices);

	RCLCPP_INFO(this->get_logger(), "Ordered sequence for SS2:");

	appendPoses(ordered_msg, red_indices, "red/base");
	appendPoses(ordered_msg, blue_indices, "blue/second");
	appendPoses(ordered_msg, yellow_indices, "yellow/top");

	RCLCPP_INFO(this->get_logger(), "Ordering complete: red base, blue second layer, yellow top.");

    return ordered_msg;
  }

  void sortByNearest(std::vector<size_t> &indices)
  {
    std::sort(indices.begin(), indices.end(),
      [this](size_t a, size_t b)
      {
        return planarDistance(latest_objects_.poses[a]) <
               planarDistance(latest_objects_.poses[b]);
      });
  }

void appendPoses(
  geometry_msgs::msg::PoseArray &msg,
  const std::vector<size_t> &indices,
  const std::string &role)
{
  for (size_t index : indices)
  {
    const auto &pose = latest_objects_.poses[index];
    msg.poses.push_back(pose);

    RCLCPP_INFO(this->get_logger(),
      "Added %s | x=%.3f, y=%.3f, z=%.3f, distance=%.3f",
      role.c_str(),
      pose.position.x,
      pose.position.y,
      pose.position.z,
      planarDistance(pose));
  }
}

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr detected_objects_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr object_labels_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr ordered_objects_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SS3TaskCoordinator>());
  rclcpp::shutdown();
  return 0;
}
