#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

#include <moveit/move_group_interface/move_group_interface.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>
#include <string>
#include <thread>
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

    retrieve_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/client/retrieve_cube", 10,
      std::bind(&SS3TaskCoordinator::retrieveCallback, this, std::placeholders::_1));

    detected_objects_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
      "/detected_objects", 10);

    retrieve_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/retrieve_cube", 10);

    republish_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&SS3TaskCoordinator::republishSequence, this));

    RCLCPP_INFO(this->get_logger(), "SS3 ready. Waiting for /client/start.");
    setState(RobotState::IDLE);
  }

private:
  enum class RobotState
  {
    IDLE,
    MOVING_TO_SEARCH,
    WAITING_FOR_DETECTION,
    VALIDATING,
    SEQUENCE_PUBLISHED,
    RETRIEVAL_READY,
    RESETTING,
    ERROR
  };

  geometry_msgs::msg::PoseArray latest_objects_;
  geometry_msgs::msg::PoseArray last_published_sequence_;
  std::vector<std::string> latest_labels_;

  RobotState state_ = RobotState::IDLE;

  bool objects_received_ = false;
  bool labels_received_ = false;
  bool task_active_ = false;
  bool sequence_published_ = false;
  bool search_position_reached_ = false;

  int search_retry_count_ = 0;
  const int max_search_retries_ = 2;

  int stable_detection_count_ = 0;
  const int required_stable_detections_ = 3;
  std::string last_detection_signature_;

  std::string stateToString(RobotState state)
  {
    switch (state)
    {
      case RobotState::IDLE: return "IDLE";
      case RobotState::MOVING_TO_SEARCH: return "MOVING_TO_SEARCH";
      case RobotState::WAITING_FOR_DETECTION: return "WAITING_FOR_DETECTION";
      case RobotState::VALIDATING: return "VALIDATING";
      case RobotState::SEQUENCE_PUBLISHED: return "SEQUENCE_PUBLISHED";
      case RobotState::RETRIEVAL_READY: return "RETRIEVAL_READY";
      case RobotState::RESETTING: return "RESETTING";
      case RobotState::ERROR: return "ERROR";
      default: return "UNKNOWN";
    }
  }

  void setState(RobotState new_state)
  {
    state_ = new_state;
    RCLCPP_INFO(this->get_logger(),
                "SS3 state changed to: %s",
                stateToString(state_).c_str());
  }

  bool moveToSearchPosition()
  {
    RCLCPP_INFO(this->get_logger(), "Moving UR3e to search position...");

    moveit::planning_interface::MoveGroupInterface move_group(
      shared_from_this(),
      "ur_onrobot_manipulator");

    std::vector<double> search_joint_values =
    {
      -1.3379457632647913,   // shoulder_pan_joint
      -1.676735063592428,    // shoulder_lift_joint
      -0.06869813799858093,  // elbow_joint
      -2.7657791576781214,   // wrist_1_joint
       1.5695604085922241,   // wrist_2_joint
       0.19399519264698029   // wrist_3_joint
    };

//    move_group.setJointValueTarget(search_joint_values);
    move_group.setNamedTarget("test_config");
    move_group.setPlanningTime(10.0);
    move_group.setNumPlanningAttempts(3);
    move_group.setMaxVelocityScalingFactor(0.10);
    move_group.setMaxAccelerationScalingFactor(0.10);

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
      RCLCPP_WARN(this->get_logger(), "Task already active. Ignoring START.");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Client START received.");

    task_active_ = true;
    sequence_published_ = false;
    search_position_reached_ = false;
    search_retry_count_ = 0;
    stable_detection_count_ = 0;
    last_detection_signature_.clear();

    clearPerceptionCache();
    last_published_sequence_.poses.clear();

    setState(RobotState::MOVING_TO_SEARCH);

    std::thread([this]()
    {
      while (task_active_ && search_retry_count_ <= max_search_retries_)
      {
        bool success = moveToSearchPosition();

        if (!task_active_)
        {
          RCLCPP_WARN(this->get_logger(),
                      "Search movement finished, but task was reset/cancelled.");
          return;
        }

        if (success)
        {
          search_position_reached_ = true;
          setState(RobotState::WAITING_FOR_DETECTION);

          RCLCPP_INFO(this->get_logger(),
                      "Search position complete. Waiting for stable SS1 cube poses and labels.");

          tryPublishSequence();
          return;
        }

        search_retry_count_++;

        RCLCPP_WARN(this->get_logger(),
                    "Search position retry %d/%d",
                    search_retry_count_,
                    max_search_retries_);
      }

      RCLCPP_ERROR(this->get_logger(),
                   "Search position failed after maximum retries.");

      setState(RobotState::ERROR);
      task_active_ = false;

    }).detach();

//search_position_reached_ = true;
//
//setState(RobotState::WAITING_FOR_DETECTION);
//
//RCLCPP_WARN(this->get_logger(),
//            "TEST MODE: Search position bypassed (no robot/MoveIt running).");
//
//tryPublishSequence();
  }

  void resetCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data)
      return;

    setState(RobotState::RESETTING);

    task_active_ = false;
    sequence_published_ = false;
    search_position_reached_ = false;
    search_retry_count_ = 0;
    stable_detection_count_ = 0;
    last_detection_signature_.clear();

    clearPerceptionCache();
    last_published_sequence_.poses.clear();

    RCLCPP_INFO(this->get_logger(), "SS3 reset complete. Ready for next task.");
    setState(RobotState::IDLE);
  }

  void rawObjectsCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    latest_objects_ = *msg;
    objects_received_ = true;

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Received %zu raw detected object poses.",
                         latest_objects_.poses.size());

    tryPublishSequence();
  }

  void labelsCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    latest_labels_ = splitLabels(msg->data);
    labels_received_ = true;

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Received object labels: %s",
                         msg->data.c_str());

    tryPublishSequence();
  }

  void retrieveCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (state_ != RobotState::SEQUENCE_PUBLISHED &&
        state_ != RobotState::RETRIEVAL_READY)
    {
      RCLCPP_WARN(this->get_logger(),
                  "Retrieval request received before build handoff. Forwarding anyway.");
    }

    auto out = std_msgs::msg::String();
    out.data = msg->data;

    retrieve_pub_->publish(out);

    setState(RobotState::RETRIEVAL_READY);

    RCLCPP_INFO(this->get_logger(),
                "Forwarded retrieval request to SS2: %s",
                msg->data.c_str());
  }

  void tryPublishSequence()
  {
    if (!task_active_ || sequence_published_)
      return;

    if (!search_position_reached_)
      return;

    if (!objects_received_ || !labels_received_)
      return;

    setState(RobotState::VALIDATING);

    if (!isDetectionStable())
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Waiting for stable detection frames: %d/%d",
                           stable_detection_count_,
                           required_stable_detections_);
      setState(RobotState::WAITING_FOR_DETECTION);
      return;
    }

    geometry_msgs::msg::PoseArray output_msg;

    if (validateFullPyramid())
    {
      output_msg = generateFullPyramidPoseArray();
      RCLCPP_INFO(this->get_logger(), "Publishing FULL PYRAMID sequence.");
    }
    else if (canBuildBaseFallback())
    {
      output_msg = generateBaseFallbackPoseArray();
      RCLCPP_WARN(this->get_logger(), "Publishing BASE-ONLY fallback sequence.");
    }
    else
    {
      RCLCPP_WARN(this->get_logger(),
                  "No valid sequence available yet. Waiting for better SS1 data.");
      setState(RobotState::WAITING_FOR_DETECTION);
      return;
    }

    detected_objects_pub_->publish(output_msg);
    last_published_sequence_ = output_msg;
    sequence_published_ = true;

    setState(RobotState::SEQUENCE_PUBLISHED);

    RCLCPP_INFO(this->get_logger(),
                "Published %zu ordered cube poses to SS2 on /detected_objects.",
                output_msg.poses.size());

    RCLCPP_INFO(this->get_logger(),
                "SS3 handoff complete. SS2 can now execute the received sequence.");
  }

  bool isDetectionStable()
  {
    std::string signature = buildDetectionSignature();

    if (signature == last_detection_signature_)
    {
      stable_detection_count_++;
    }
    else
    {
      stable_detection_count_ = 1;
      last_detection_signature_ = signature;
    }

    return stable_detection_count_ >= required_stable_detections_;
  }

  std::string buildDetectionSignature()
  {
    std::stringstream signature;

    signature << "poses=" << latest_objects_.poses.size();
    signature << "|labels=";

    for (const auto &label : latest_labels_)
    {
      signature << label << ",";
    }

    return signature.str();
  }

  void republishSequence()
  {
    if (!sequence_published_)
      return;

    if (last_published_sequence_.poses.empty())
      return;

    detected_objects_pub_->publish(last_published_sequence_);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Republishing %zu ordered cube poses to /detected_objects.",
                         last_published_sequence_.poses.size());
  }

  bool validateFullPyramid()
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
                  "Full pyramid validation failed. Expected exactly 6 poses, received: %zu",
                  pose_count);
      return false;
    }

    int red_count = countColour("red");
    int yellow_count = countColour("yellow");
    int blue_count = countColour("blue");

    if (red_count != 3 || yellow_count != 2 || blue_count != 1)
    {
      RCLCPP_WARN(this->get_logger(),
                  "Full pyramid validation failed. Expected red=3, yellow=2, blue=1 | Received red=%d, yellow=%d, blue=%d",
                  red_count,
                  yellow_count,
                  blue_count);
      return false;
    }

    RCLCPP_INFO(this->get_logger(),
                "Full pyramid validation passed: 3 red, 2 yellow, 1 blue.");

    return true;
  }

  bool canBuildBaseFallback()
  {
    if (latest_objects_.poses.size() < 3)
    {
      RCLCPP_WARN(this->get_logger(),
                  "Base fallback unavailable: fewer than 3 cubes detected.");
      return false;
    }

    if (latest_objects_.poses.size() != latest_labels_.size())
    {
      RCLCPP_WARN(this->get_logger(),
                  "Base fallback unavailable: pose-label mismatch.");
      return false;
    }

    RCLCPP_WARN(this->get_logger(),
                "Base fallback available: using nearest 3 detected cubes.");

    return true;
  }

  geometry_msgs::msg::PoseArray generateFullPyramidPoseArray()
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

    RCLCPP_INFO(this->get_logger(), "Ordered full sequence for SS2:");

    appendPoses(ordered_msg, red_indices, "red/base");
    appendPoses(ordered_msg, yellow_indices, "yellow/middle");
    appendPoses(ordered_msg, blue_indices, "blue/top");

    RCLCPP_INFO(this->get_logger(),
                "Ordering complete: red base, yellow middle layer, blue top.");

    return ordered_msg;
  }

  geometry_msgs::msg::PoseArray generateBaseFallbackPoseArray()
  {
    geometry_msgs::msg::PoseArray fallback_msg;
    fallback_msg.header = latest_objects_.header;
    fallback_msg.header.frame_id = "base_link";

    std::vector<size_t> all_indices;

    for (size_t i = 0; i < latest_objects_.poses.size(); ++i)
    {
      all_indices.push_back(i);
    }

    sortByNearest(all_indices);

    for (size_t i = 0; i < 3; ++i)
    {
      const auto &pose = latest_objects_.poses[all_indices[i]];
      fallback_msg.poses.push_back(pose);

      RCLCPP_WARN(this->get_logger(),
                  "Fallback base cube %zu | x=%.3f, y=%.3f, z=%.3f",
                  i + 1,
                  pose.position.x,
                  pose.position.y,
                  pose.position.z);
    }

    return fallback_msg;
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
      {
        indices.push_back(i);
      }
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
      {
        labels.push_back(label);
      }
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
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr retrieve_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr detected_objects_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr retrieve_pub_;

  rclcpp::TimerBase::SharedPtr republish_timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SS3TaskCoordinator>());
  rclcpp::shutdown();
  return 0;
}
