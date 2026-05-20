#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

#include <moveit/move_group_interface/move_group_interface.h>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

class SS3TaskCoordinator : public rclcpp::Node
{
public:
  SS3TaskCoordinator() : Node("ss3_task_coordinator")
  {
    raw_objects_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/raw_detected_objects", 10,
      std::bind(&SS3TaskCoordinator::rawObjectsCallback, this, std::placeholders::_1));

    labels_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/object_labels", 10,
      std::bind(&SS3TaskCoordinator::labelsCallback, this, std::placeholders::_1));

    start_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/client/start", 10,
      std::bind(&SS3TaskCoordinator::startCallback, this, std::placeholders::_1));

    reset_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/client/reset", 10,
      std::bind(&SS3TaskCoordinator::resetCallback, this, std::placeholders::_1));

    client_retrieve_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/client/retrieve_cube", 10,
      std::bind(&SS3TaskCoordinator::retrieveCallback, this, std::placeholders::_1));

    ordered_objects_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
      "/detected_objects", 10);

    retrieve_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/retrieve_cube", 10);

    RCLCPP_INFO(this->get_logger(),
                "SS3 ready. Waiting for /client/start.");
  }

private:
  geometry_msgs::msg::PoseArray latest_objects_;
  std::vector<std::string> latest_labels_;

  bool objects_received_ = false;
  bool labels_received_ = false;
  bool task_active_ = false;
  bool ordered_sequence_published_ = false;
  bool pyramid_complete_ = false;

  bool moveToSearchPosition()
  {
    RCLCPP_INFO(this->get_logger(), "Moving UR3e to search position...");

    moveit::planning_interface::MoveGroupInterface move_group(
      shared_from_this(),
      "ur_onrobot_manipulator");

    // TODO: Replace with real safe camera-search joint values from /joint_states.
    std::vector<double> search_joint_values =
    {
      -1.57,
      -1.20,
      -1.80,
      -1.50,
       1.57,
       0.00
    };

    move_group.setJointValueTarget(search_joint_values);
    move_group.setPlanningTime(10.0);

    auto result = move_group.move();

    if (result == moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(this->get_logger(), "Search position reached.");
      return true;
    }

    RCLCPP_ERROR(this->get_logger(), "Failed to reach search position.");
    return false;
  }

  void startCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data)
      return;

    if (task_active_)
    {
      RCLCPP_WARN(this->get_logger(),
                  "Task already active. Ignoring START.");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Client START received.");

    task_active_ = true;
    ordered_sequence_published_ = false;
    pyramid_complete_ = false;

    clearPerceptionCache();

    if (!moveToSearchPosition())
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Aborting task: search position failed.");
      task_active_ = false;
      return;
    }

    RCLCPP_INFO(this->get_logger(),
                "Search position complete. Waiting for SS1 to publish all 6 cube poses and labels.");
  }

  void resetCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data)
      return;

    task_active_ = false;
    ordered_sequence_published_ = false;
    pyramid_complete_ = false;

    clearPerceptionCache();

    RCLCPP_INFO(this->get_logger(),
                "SS3 reset complete. Ready for next task.");
  }

  void rawObjectsCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    latest_objects_ = *msg;
    objects_received_ = true;

    RCLCPP_INFO(this->get_logger(),
                "Received %zu raw detected object poses.",
                latest_objects_.poses.size());

    tryPublishFullSequence();
  }

  void labelsCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    latest_labels_ = splitLabels(msg->data);
    labels_received_ = true;

    RCLCPP_INFO(this->get_logger(),
                "Received object labels: %s",
                msg->data.c_str());

    tryPublishFullSequence();
  }

  void retrieveCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    auto out = std_msgs::msg::String();
    out.data = msg->data;

    retrieve_pub_->publish(out);

    RCLCPP_INFO(this->get_logger(),
                "Forwarded retrieval request to SS2: %s",
                msg->data.c_str());
  }

  void tryPublishFullSequence()
  {
    if (!task_active_)
      return;

    if (ordered_sequence_published_)
      return;

    if (!objects_received_ || !labels_received_)
      return;

    if (!validateDetections())
      return;

    geometry_msgs::msg::PoseArray ordered_msg = generateOrderedPoseArray();

    ordered_objects_pub_->publish(ordered_msg);

    ordered_sequence_published_ = true;
    pyramid_complete_ = true;

    RCLCPP_INFO(this->get_logger(),
                "Published full ordered 6-cube PoseArray to SS2 on /detected_objects.");
    RCLCPP_INFO(this->get_logger(),
                "SS3 handoff complete. SS2 can now execute full pyramid build.");
  }

  bool validateDetections()
  {
    const size_t pose_count = latest_objects_.poses.size();
    const size_t label_count = latest_labels_.size();

    if (pose_count != label_count)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Pose-label mismatch. Poses: %zu | Labels: %zu",
                   pose_count,
                   label_count);
      return false;
    }

    if (pose_count != 6)
    {
      RCLCPP_WARN(this->get_logger(),
                  "Waiting for all 6 cubes. Current detected poses: %zu",
                  pose_count);
      return false;
    }

    int red_count = countColour("red");
    int yellow_count = countColour("yellow");
    int blue_count = countColour("blue");

    if (red_count != 3 || yellow_count != 2 || blue_count != 1)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Invalid colour distribution. Expected red=3, yellow=2, blue=1 | Received red=%d, yellow=%d, blue=%d",
                   red_count,
                   yellow_count,
                   blue_count);
      return false;
    }

    RCLCPP_INFO(this->get_logger(),
                "Detection validation passed: 3 red, 2 yellow, 1 blue.");

    return true;
  }

  geometry_msgs::msg::PoseArray generateOrderedPoseArray()
  {
    geometry_msgs::msg::PoseArray ordered_msg;
    ordered_msg.header = latest_objects_.header;
    ordered_msg.header.frame_id = "base_link";

    std::vector<size_t> red_indices = getColourIndices("red");
    std::vector<size_t> yellow_indices = getColourIndices("yellow");
    std::vector<size_t> blue_indices = getColourIndices("blue");

    sortByNearest(red_indices);
    sortByNearest(yellow_indices);
    sortByNearest(blue_indices);

    RCLCPP_INFO(this->get_logger(), "Ordered sequence for SS2:");

    appendPoses(ordered_msg, red_indices, "red/base");
    appendPoses(ordered_msg, yellow_indices, "yellow/middle");
    appendPoses(ordered_msg, blue_indices, "blue/top");

    RCLCPP_INFO(this->get_logger(),
                "Ordering complete: red base, yellow middle layer, blue top.");

    return ordered_msg;
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

  std::vector<size_t> getColourIndices(const std::string &colour)
  {
    std::vector<size_t> indices;

    for (size_t i = 0; i < latest_labels_.size(); ++i)
    {
      if (latest_labels_[i] == colour)
        indices.push_back(i);
    }

    return indices;
  }

  int countColour(const std::string &colour)
  {
    return std::count(latest_labels_.begin(), latest_labels_.end(), colour);
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

  double planarDistance(const geometry_msgs::msg::Pose &pose)
  {
    return std::sqrt(
      pose.position.x * pose.position.x +
      pose.position.y * pose.position.y);
  }

  std::vector<std::string> splitLabels(const std::string &label_string)
  {
    std::vector<std::string> labels;
    std::stringstream ss(label_string);
    std::string label;

    while (std::getline(ss, label, ','))
    {
      label.erase(
        std::remove(label.begin(), label.end(), ' '),
        label.end());

      std::transform(label.begin(), label.end(), label.begin(), ::tolower);

      if (!label.empty())
        labels.push_back(label);
    }

    return labels;
  }

  void clearPerceptionCache()
  {
    latest_objects_.poses.clear();
    latest_labels_.clear();

    objects_received_ = false;
    labels_received_ = false;
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr raw_objects_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr labels_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr client_retrieve_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr ordered_objects_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr retrieve_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SS3TaskCoordinator>());
  rclcpp::shutdown();
  return 0;
}
