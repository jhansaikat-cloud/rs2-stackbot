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

    ss2_status_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/ss2/execution_status", 10,
      std::bind(&SS3TaskCoordinator::ss2StatusCallback, this, std::placeholders::_1));

    client_retrieve_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/client/retrieve_cube", 10,
      std::bind(&SS3TaskCoordinator::retrieveCallback, this, std::placeholders::_1));

    ordered_objects_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
      "/detected_objects", 10);

    stage_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/ss3/current_stage", 10);

    retrieve_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/retrieve_cube", 10);

    RCLCPP_INFO(this->get_logger(),
                "SS3 Task Coordinator ready. Waiting for /client/start...");
    publishStage(current_stage_);
  }

private:
  geometry_msgs::msg::PoseArray latest_objects_;
  std::vector<std::string> latest_labels_;

  bool objects_received_ = false;
  bool labels_received_ = false;
  bool task_active_ = false;
  bool pyramid_complete_ = false;

  // Main stages:
  // IDLE
  // SEARCH_FOR_BASE
  // WAITING_BASE_COMPLETE
  // SEARCH_FOR_MIDDLE
  // WAITING_MIDDLE_COMPLETE
  // SEARCH_FOR_TOP
  // WAITING_TOP_COMPLETE
  // PYRAMID_COMPLETE
  // RETRIEVAL_READY
  std::string current_stage_ = "IDLE";

  bool moveToSearchPosition()
  {
    RCLCPP_INFO(this->get_logger(), "Moving UR3e to search position...");

    moveit::planning_interface::MoveGroupInterface move_group(
      shared_from_this(),
      "ur_onrobot_manipulator");

    // IMPORTANT:
    // Replace these placeholder values with the real safe camera-search pose
    // collected from /joint_states on the real robot.
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
    pyramid_complete_ = false;
    current_stage_ = "SEARCH_FOR_BASE";
    publishStage(current_stage_);

    clearPerceptionCache();

    if (!moveToSearchPosition())
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Aborting task: search position failed.");
      current_stage_ = "ERROR_SEARCH_POSITION";
      publishStage(current_stage_);
      task_active_ = false;
      return;
    }

    RCLCPP_INFO(this->get_logger(),
                "Waiting for SS1 to detect 3 red cubes for base layer...");
  }

  void resetCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data)
      return;

    task_active_ = false;
    pyramid_complete_ = false;
    current_stage_ = "IDLE";

    clearPerceptionCache();

    RCLCPP_INFO(this->get_logger(),
                "SS3 reset complete. Ready for next task.");
    publishStage(current_stage_);
  }

  void rawObjectsCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    latest_objects_ = *msg;
    objects_received_ = true;

    RCLCPP_INFO(this->get_logger(),
                "Received %zu raw detected object poses.",
                latest_objects_.poses.size());

    tryProcessCurrentStage();
  }

  void labelsCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    latest_labels_ = splitLabels(msg->data);
    labels_received_ = true;

    RCLCPP_INFO(this->get_logger(),
                "Received object labels: %s",
                msg->data.c_str());

    tryProcessCurrentStage();
  }

  void ss2StatusCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(),
                "SS2 status received: %s",
                msg->data.c_str());

    if (msg->data == "BASE_COMPLETE" &&
        current_stage_ == "WAITING_BASE_COMPLETE")
    {
      moveToNextSearchStage("SEARCH_FOR_MIDDLE",
                            "Waiting for SS1 to detect 2 yellow cubes for middle layer...");
    }
    else if (msg->data == "MIDDLE_COMPLETE" &&
             current_stage_ == "WAITING_MIDDLE_COMPLETE")
    {
      moveToNextSearchStage("SEARCH_FOR_TOP",
                            "Waiting for SS1 to detect 1 blue cube for top layer...");
    }
    else if ((msg->data == "TOP_COMPLETE" || msg->data == "TASK_COMPLETE") &&
             current_stage_ == "WAITING_TOP_COMPLETE")
    {
      current_stage_ = "PYRAMID_COMPLETE";
      pyramid_complete_ = true;
      publishStage(current_stage_);

      RCLCPP_INFO(this->get_logger(),
                  "Pyramid build complete. Retrieval mode is now available.");

      current_stage_ = "RETRIEVAL_READY";
      publishStage(current_stage_);
    }
  }

  void retrieveCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (!pyramid_complete_)
    {
      RCLCPP_WARN(this->get_logger(),
                  "Retrieval request ignored. Pyramid build is not complete yet.");
      return;
    }

    auto out = std_msgs::msg::String();
    out.data = msg->data;

    retrieve_pub_->publish(out);

    RCLCPP_INFO(this->get_logger(),
                "Forwarded retrieval request to SS2: %s",
                msg->data.c_str());
  }

  void moveToNextSearchStage(const std::string &next_stage,
                             const std::string &message)
  {
    current_stage_ = next_stage;
    publishStage(current_stage_);

    clearPerceptionCache();

    if (!moveToSearchPosition())
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to return to search position.");
      current_stage_ = "ERROR_SEARCH_POSITION";
      publishStage(current_stage_);
      task_active_ = false;
      return;
    }

    RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
  }

  void tryProcessCurrentStage()
  {
    if (!task_active_)
      return;

    if (!objects_received_ || !labels_received_)
      return;

    if (latest_objects_.poses.size() != latest_labels_.size())
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Pose-label mismatch. Poses: %zu | Labels: %zu",
                   latest_objects_.poses.size(),
                   latest_labels_.size());
      return;
    }

    if (current_stage_ == "SEARCH_FOR_BASE")
    {
      processLayer("red", 3, "BASE_LAYER", "WAITING_BASE_COMPLETE");
    }
    else if (current_stage_ == "SEARCH_FOR_MIDDLE")
    {
      processLayer("yellow", 2, "MIDDLE_LAYER", "WAITING_MIDDLE_COMPLETE");
    }
    else if (current_stage_ == "SEARCH_FOR_TOP")
    {
      processLayer("blue", 1, "TOP_LAYER", "WAITING_TOP_COMPLETE");
    }
  }

  void processLayer(const std::string &colour,
                    size_t required_count,
                    const std::string &layer_stage,
                    const std::string &waiting_stage)
  {
    std::vector<size_t> indices = getColourIndices(colour);

    if (indices.size() < required_count)
    {
      RCLCPP_WARN(this->get_logger(),
                  "Not enough %s cubes detected. Required: %zu | Detected: %zu",
                  colour.c_str(),
                  required_count,
                  indices.size());
      return;
    }

    sortByNearest(indices);

    geometry_msgs::msg::PoseArray ordered_msg;
    ordered_msg.header = latest_objects_.header;
    ordered_msg.header.frame_id = "base_link";

    RCLCPP_INFO(this->get_logger(),
                "Preparing %s with %zu %s cube(s).",
                layer_stage.c_str(),
                required_count,
                colour.c_str());

    for (size_t i = 0; i < required_count; ++i)
    {
      const auto &pose = latest_objects_.poses[indices[i]];
      ordered_msg.poses.push_back(pose);

      RCLCPP_INFO(this->get_logger(),
                  "Added %s cube %zu | x=%.3f, y=%.3f, z=%.3f, distance=%.3f",
                  colour.c_str(),
                  i + 1,
                  pose.position.x,
                  pose.position.y,
                  pose.position.z,
                  planarDistance(pose));
    }

    publishStage(layer_stage);
    ordered_objects_pub_->publish(ordered_msg);

    RCLCPP_INFO(this->get_logger(),
                "Published %s ordered poses to SS2 on /detected_objects.",
                layer_stage.c_str());

    current_stage_ = waiting_stage;
    publishStage(current_stage_);

    clearPerceptionCache();
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

  void publishStage(const std::string &stage)
  {
    auto msg = std_msgs::msg::String();
    msg.data = stage;

    stage_pub_->publish(msg);

    RCLCPP_INFO(this->get_logger(),
                "SS3 stage: %s",
                stage.c_str());
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr raw_objects_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr labels_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ss2_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr client_retrieve_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr ordered_objects_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stage_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr retrieve_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SS3TaskCoordinator>());
  rclcpp::shutdown();
  return 0;
}
