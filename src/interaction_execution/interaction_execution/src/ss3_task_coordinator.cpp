#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

#include <moveit/move_group_interface/move_group_interface.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
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
      
    pyramid_config_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/pyramid_config", 10,
      std::bind(&SS3TaskCoordinator::pyramidConfigCallback, this, std::placeholders::_1));

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
  
  double pyramid_x_ = -0.12;
  double pyramid_y_ = 0.30;
  double pyramid_step_ = 0.0555;
  bool pyramid_config_received_ = false;

  int search_retry_count_ = 0;
  const int max_search_retries_ = 2;

  int stable_detection_count_ = 0;
  const int required_stable_detections_ = 3;
  std::string last_detection_signature_;
  
  const int perception_settle_seconds_ = 5;
  const int full_detection_wait_seconds_ = 3;
  bool partial_wait_done_ = false;

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
    move_group.setNamedTarget("search");
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
    partial_wait_done_ = false;

    clearPerceptionCache();
    
    // TEST MODE ONLY: bypass MoveIt search position when no robot/MoveIt is running.
    //search_position_reached_ = true;

    //setState(RobotState::WAITING_FOR_DETECTION);

    //RCLCPP_WARN(this->get_logger(),
	//"TEST MODE: Search position bypassed. Waiting for SS1 cube poses and labels.");
    //tryPublishSequence();
    //return;

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
		      "Search position reached. Waiting %d seconds for SS1 perception to stabilise...",
		      perception_settle_seconds_);

	  std::this_thread::sleep_for(std::chrono::seconds(perception_settle_seconds_));

	  if (!task_active_)
	  {
	    RCLCPP_WARN(this->get_logger(),
		        "Perception settling finished, but task was reset/cancelled.");
	    return;
	  }

	  RCLCPP_INFO(this->get_logger(),
		      "Perception settling complete. SS3 will now process SS1 detections.");

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
    partial_wait_done_ = false;

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

void pyramidConfigCallback(const std_msgs::msg::String::SharedPtr msg)
{
  std::stringstream ss(msg->data);
  std::string item;
  std::vector<double> values;

  while (std::getline(ss, item, ','))
  {
    try
    {
      values.push_back(std::stod(item));
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Invalid /pyramid_config value received: %s",
                   msg->data.c_str());
      return;
    }
  }

  if (values.size() != 3)
  {
    RCLCPP_ERROR(this->get_logger(),
                 "Invalid /pyramid_config format. Expected: PYRAMID_X,PYRAMID_Y,STEP");
    return;
  }

  pyramid_x_ = values[0];
  pyramid_y_ = values[1];
  pyramid_step_ = values[2];
  pyramid_config_received_ = true;

  RCLCPP_INFO(this->get_logger(),
              "Updated pyramid config from SS2: x=%.3f, y=%.3f, step=%.4f",
              pyramid_x_,
              pyramid_y_,
              pyramid_step_);
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
    if (!objects_received_ || !labels_received_)
  return;

if (!pyramid_config_received_)
{
  RCLCPP_WARN_THROTTLE(this->get_logger(),
                       *this->get_clock(),
                       5000,
                       "No /pyramid_config received. Using default pyramid config: x=%.3f, y=%.3f, step=%.4f",
                       pyramid_x_,
                       pyramid_y_,
                       pyramid_step_);
}

if (!isBuildZoneClear())
{
  RCLCPP_ERROR(this->get_logger(),
               "SS3 blocked publishing because pyramid build zone is occupied. Waiting for updated SS1 detections...");

  setState(RobotState::WAITING_FOR_DETECTION);

  // Do not reset task_active_
  // Do not set sequence_published_
  // Do not clear perception cache
  // SS3 will automatically re-check when SS1 publishes updated poses/labels.
  return;
}

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
else if (canBuildPartialPyramid())
{
  if (!partial_wait_done_)
  {
    RCLCPP_WARN(this->get_logger(),
                "Partial sequence available, but waiting %d seconds for possible full 6-cube detection...",
                full_detection_wait_seconds_);

    partial_wait_done_ = true;

    std::thread([this]()
    {
      std::this_thread::sleep_for(std::chrono::seconds(full_detection_wait_seconds_));

      if (!task_active_ || sequence_published_)
        return;

      RCLCPP_WARN(this->get_logger(),
                  "Full detection wait window finished. Proceeding with best available sequence.");

      tryPublishSequence();

    }).detach();

    setState(RobotState::WAITING_FOR_DETECTION);
    return;
  }

  output_msg = generatePartialPyramidPoseArray();
  RCLCPP_WARN(this->get_logger(),
              "Publishing PARTIAL PYRAMID sequence with %zu pose(s).",
              output_msg.poses.size());
}
else
{
  RCLCPP_WARN(this->get_logger(),
              "No valid sequence available yet. Waiting for better SS1 data.");
  setState(RobotState::WAITING_FOR_DETECTION);
  return;
}

      RCLCPP_INFO(this->get_logger(),
            "Valid sequence generated. Waiting 2 seconds before publishing to SS2...");

      std::this_thread::sleep_for(std::chrono::seconds(2));

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

bool isBuildZoneClear()
{
  const double safe_radius = pyramid_step_ * 2.2;

  for (const auto &pose : latest_objects_.poses)
  {
    double dx = pose.position.x - pyramid_x_;
    double dy = pose.position.y - pyramid_y_;
    double distance = std::sqrt(dx * dx + dy * dy);

    if (distance < safe_radius)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Build zone occupied. Cube at x=%.3f, y=%.3f is %.3f m from pyramid centre x=%.3f, y=%.3f. Safe radius=%.3f m.",
                   pose.position.x,
                   pose.position.y,
                   distance,
                   pyramid_x_,
                   pyramid_y_,
                   safe_radius);

      return false;
    }
  }

  RCLCPP_INFO_THROTTLE(this->get_logger(),
                       *this->get_clock(),
                       3000,
                       "Build zone clear. Pyramid centre x=%.3f, y=%.3f, step=%.4f",
                       pyramid_x_,
                       pyramid_y_,
                       pyramid_step_);

  return true;
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

bool canBuildPartialPyramid()
{
  if (latest_objects_.poses.size() != latest_labels_.size())
  {
    RCLCPP_WARN(this->get_logger(),
                "Partial pyramid unavailable: pose-label mismatch.");
    return false;
  }

  if (latest_objects_.poses.size() < 3)
  {
    RCLCPP_WARN(this->get_logger(),
                "Partial pyramid unavailable: fewer than 3 cubes detected.");
    return false;
  }

  RCLCPP_WARN(this->get_logger(),
              "Partial pyramid available with %zu detected cubes.",
              latest_objects_.poses.size());

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

geometry_msgs::msg::PoseArray generatePartialPyramidPoseArray()
{
  geometry_msgs::msg::PoseArray partial_msg;
  partial_msg.header = latest_objects_.header;
  partial_msg.header.frame_id = "base_link";

  const size_t max_publish_count = std::min<size_t>(latest_objects_.poses.size(), 6);

  std::vector<size_t> red_indices = getColourIndices("red");
  std::vector<size_t> yellow_indices = getColourIndices("yellow");
  std::vector<size_t> blue_indices = getColourIndices("blue");

  sortByNearest(red_indices);
  sortByNearest(yellow_indices);
  sortByNearest(blue_indices);

  std::vector<bool> used(latest_objects_.poses.size(), false);

  auto addIndex = [&](size_t index, const std::string &role)
  {
    if (partial_msg.poses.size() >= max_publish_count)
      return;

    if (index >= latest_objects_.poses.size())
      return;

    if (used[index])
      return;

    const auto &pose = latest_objects_.poses[index];
    partial_msg.poses.push_back(pose);
    used[index] = true;

    RCLCPP_WARN(this->get_logger(),
                "Partial sequence added %s | x=%.3f, y=%.3f, z=%.3f, distance=%.3f",
                role.c_str(),
                pose.position.x,
                pose.position.y,
                pose.position.z,
                planarDistance(pose));
  };

  // 1. Fill base layer first: prefer red cubes.
  for (size_t index : red_indices)
  {
    if (partial_msg.poses.size() >= 3)
      break;

    addIndex(index, "base/red");
  }

  // 2. If fewer than 3 red cubes are available, fill base with nearest unused cubes.
  while (partial_msg.poses.size() < 3)
  {
    int nearest_index = findNearestUnusedIndex(used);

    if (nearest_index < 0)
      break;

    addIndex(static_cast<size_t>(nearest_index), "base/fallback");
  }

  // 3. Fill middle layer next: prefer yellow cubes.
  for (size_t index : yellow_indices)
  {
    if (partial_msg.poses.size() >= max_publish_count || partial_msg.poses.size() >= 5)
      break;

    addIndex(index, "middle/yellow");
  }

  // 4. If 4th/5th cube still needed, fill middle with nearest unused cubes.
  while (partial_msg.poses.size() < max_publish_count && partial_msg.poses.size() < 5)
  {
    int nearest_index = findNearestUnusedIndex(used);

    if (nearest_index < 0)
      break;

    addIndex(static_cast<size_t>(nearest_index), "middle/fallback");
  }

  // 5. Add top cube only if 6 cubes are available: prefer blue.
  if (partial_msg.poses.size() < max_publish_count && max_publish_count >= 6)
  {
    bool blue_added = false;

    for (size_t index : blue_indices)
    {
      if (!used[index])
      {
        addIndex(index, "top/blue");
        blue_added = true;
        break;
      }
    }

    if (!blue_added && partial_msg.poses.size() < max_publish_count)
    {
      int nearest_index = findNearestUnusedIndex(used);

      if (nearest_index >= 0)
      {
        addIndex(static_cast<size_t>(nearest_index), "top/fallback");
      }
    }
  }

  RCLCPP_WARN(this->get_logger(),
              "Partial pyramid sequence generated with %zu pose(s).",
              partial_msg.poses.size());

  return partial_msg;
}

int findNearestUnusedIndex(const std::vector<bool> &used)
{
  double best_distance = std::numeric_limits<double>::max();
  int best_index = -1;

  for (size_t i = 0; i < latest_objects_.poses.size(); ++i)
  {
    if (used[i])
      continue;

    double distance = planarDistance(latest_objects_.poses[i]);

    if (distance < best_distance)
    {
      best_distance = distance;
      best_index = static_cast<int>(i);
    }
  }

  return best_index;
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
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr pyramid_config_sub_;
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
