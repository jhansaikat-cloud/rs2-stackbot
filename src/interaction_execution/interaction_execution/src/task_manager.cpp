#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

class TaskManager : public rclcpp::Node
{
public:
  TaskManager() : Node("task_manager")
  {
    start_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/start_task", 10,
      std::bind(&TaskManager::startCallback, this, std::placeholders::_1));

    execution_status_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/execution_status", 10,
      std::bind(&TaskManager::executionStatusCallback, this, std::placeholders::_1));

    gripper_feedback_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/gripper_feedback", 10,
      std::bind(&TaskManager::gripperFeedbackCallback, this, std::placeholders::_1));

    pick_place_pub_ = this->create_publisher<std_msgs::msg::String>("/pick_place_request", 10);
    gripper_command_pub_ = this->create_publisher<std_msgs::msg::String>("/gripper_command", 10);
    task_status_pub_ = this->create_publisher<std_msgs::msg::String>("/task_status", 10);

    current_state_ = "IDLE";
    publishStatus(current_state_);

    RCLCPP_INFO(this->get_logger(), "SS3 Task Manager with state machine ready.");
  }

private:
  std::string current_state_;

  void startCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data && current_state_ == "IDLE")
    {
      RCLCPP_INFO(this->get_logger(), "Start command received.");
      transitionTo("PICK_CUBE_1");
    }
  }

  void executionStatusCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Execution status received: %s", msg->data.c_str());

    if (msg->data == "DONE")
    {
      advanceState();
    }
    else if (msg->data == "FAILED")
    {
      publishStatus("ERROR_MOTION_FAILED");
    }
  }

  void gripperFeedbackCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Gripper feedback received: %s", msg->data.c_str());

    if (msg->data == "GRASP_OK")
    {
      publishStatus("GRASP_VERIFIED");
    }
    else if (msg->data == "GRASP_FAILED")
    {
      publishStatus("ERROR_GRASP_FAILED");
    }
  }

  void transitionTo(const std::string &new_state)
  {
    current_state_ = new_state;
    publishStatus(current_state_);

    if (current_state_ == "PICK_CUBE_1")
    {
      sendPickPlaceRequest("PICK_CUBE_1");
      sendGripperCommand("CLOSE");
    }
    else if (current_state_ == "PLACE_CUBE_1")
    {
      sendPickPlaceRequest("PLACE_CUBE_1");
      sendGripperCommand("OPEN");
    }
    else if (current_state_ == "PICK_CUBE_2")
    {
      sendPickPlaceRequest("PICK_CUBE_2");
      sendGripperCommand("CLOSE");
    }
    else if (current_state_ == "PLACE_CUBE_2")
    {
      sendPickPlaceRequest("PLACE_CUBE_2");
      sendGripperCommand("OPEN");
    }
    else if (current_state_ == "COMPLETE")
    {
      publishStatus("TASK_COMPLETE");
      RCLCPP_INFO(this->get_logger(), "Task completed successfully.");
    }
  }

  void advanceState()
  {
    if (current_state_ == "PICK_CUBE_1")
      transitionTo("PLACE_CUBE_1");
    else if (current_state_ == "PLACE_CUBE_1")
      transitionTo("PICK_CUBE_2");
    else if (current_state_ == "PICK_CUBE_2")
      transitionTo("PLACE_CUBE_2");
    else if (current_state_ == "PLACE_CUBE_2")
      transitionTo("COMPLETE");
  }

  void sendPickPlaceRequest(const std::string &request)
  {
    auto msg = std_msgs::msg::String();
    msg.data = request;
    pick_place_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published pick/place request: %s", request.c_str());
  }

  void sendGripperCommand(const std::string &command)
  {
    auto msg = std_msgs::msg::String();
    msg.data = command;
    gripper_command_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published gripper command: %s", command.c_str());
  }

  void publishStatus(const std::string &status)
  {
    auto msg = std_msgs::msg::String();
    msg.data = status;
    task_status_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Task status: %s", status.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr execution_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gripper_feedback_sub_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pick_place_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gripper_command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr task_status_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TaskManager>());
  rclcpp::shutdown();
  return 0;
}
