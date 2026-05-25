#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <functional>
#include <set>
#include <map>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <std_msgs/msg/string.hpp>

#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

#include "mtc_ss2/pyramid_config.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("retrieval_ss2");
namespace mtc = moveit::task_constructor;

//    ROBOT CONFIG                                                               
static const std::string ARM_GROUP   = "ur_onrobot_manipulator";
static const std::string HAND_GROUP  = "ur_onrobot_gripper";
static const std::string HAND_FRAME  = "gripper_tcp";
static const std::string FIXED_FRAME = "base_link";

//    GRIPPER DOWN ORIENTATION                                                   
static constexpr double GRIPPER_DOWN_QX = 0.7071;
static constexpr double GRIPPER_DOWN_QY = 0.7071;
static constexpr double GRIPPER_DOWN_QZ = 0.0;
static constexpr double GRIPPER_DOWN_QW = 0.0;

//    RETRIEVAL CONSTANTS                                                        
static constexpr double PRE_PLACE_HEIGHT = 0.10;
static constexpr double RETREAT_HEIGHT   = 0.10;

//    CUBE STATE                                                                 
enum class CubeLocation { PYRAMID, STAGING, DELIVERED };

struct CubeState
{
  PyramidCube info;
  CubeLocation location;
  int staging_slot;  // -1 if not staged
};

//    NODE                                                                       
class RetrievalNode
{
public:
  RetrievalNode(const rclcpp::NodeOptions& options);
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
  void run();

private:
  void setupPlanningScene();
  void removeCubeFromScene(const std::string& cube_name);
  void addCubeToStaging(const std::string& cube_name, int slot);
  void cleanupAfterFailedExecution(const std::string& cube_name);

  std::vector<std::string> computeRemovalOrder(const std::string& target);
  bool isCubeInPyramid(const std::string& name);
  CubeState& getCubeState(const std::string& name);
  int getNextFreeSlot();

  bool pickAndPlace(const CubeState& cube, double place_x, double place_y, double place_z);
  bool tryReturnHome();
  void clearScene();  

  mtc::Task createPickTask(const std::string& cube_name,
                           double place_x, double place_y, double place_z,
                           bool is_top_cube = false);

  rclcpp::Node::SharedPtr node_;

  std::vector<CubeState> cube_states_;
  std::vector<StagingSlot> staging_slots_;
  std::map<std::string, std::vector<std::string>> blocked_by_;

  std::string requested_cube_;
  bool request_received_ = false;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

//    CONSTRUCTOR                                                                
RetrievalNode::RetrievalNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("retrieval_ss2", options) }
{
  for (const auto& cube : getPyramidLayout())
    cube_states_.push_back({ cube, CubeLocation::PYRAMID, -1 });

  staging_slots_ = getStagingSlots();
  blocked_by_    = getBlockedBy();
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
RetrievalNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

//    STATE HELPERS                                                              
bool RetrievalNode::isCubeInPyramid(const std::string& name)
{
  for (const auto& s : cube_states_)
    if (s.info.name == name) return s.location == CubeLocation::PYRAMID;
  return false;
}

CubeState& RetrievalNode::getCubeState(const std::string& name)
{
  for (auto& s : cube_states_)
    if (s.info.name == name) return s;
  throw std::runtime_error("Unknown cube: " + name);
}

int RetrievalNode::getNextFreeSlot()
{
  for (int i = 0; i < (int)staging_slots_.size(); ++i)
    if (!staging_slots_[i].occupied) return i;
  return -1;
}

//    REMOVAL ORDER                                                              
std::vector<std::string> RetrievalNode::computeRemovalOrder(const std::string& target)
{
  std::vector<std::string> order;
  std::set<std::string> visited;

  std::function<void(const std::string&)> dfs = [&](const std::string& cube)
  {
    if (visited.count(cube)) return;
    visited.insert(cube);
    for (const auto& blocker : blocked_by_.at(cube))
      if (isCubeInPyramid(blocker))
        dfs(blocker);
    order.push_back(cube);
  };

  dfs(target);
  return order; 
}

void RetrievalNode::clearScene()                                      
{
  moveit::planning_interface::PlanningSceneInterface psi;
  std::vector<std::string> to_remove = {
    "table", "back_trolley",
    "cube_1_placed", "cube_2_placed", "cube_3_placed",
    "cube_4_placed", "cube_5_placed", "cube_6_placed",
    "cube_4_staged", "cube_5_staged", "cube_6_staged",
  };
  for (const auto& id : to_remove)
  {
    moveit_msgs::msg::CollisionObject obj;
    obj.id = id;
    obj.header.frame_id = FIXED_FRAME;
    obj.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    psi.applyCollisionObject(obj);
  }
  RCLCPP_INFO(LOGGER, "Cleared previous scene objects.");
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
}     

void RetrievalNode::setupPlanningScene()
{
  moveit::planning_interface::PlanningSceneInterface psi;

  // Workspace
  moveit_msgs::msg::CollisionObject table;
  table.header.frame_id = FIXED_FRAME;
  table.id = "table";
  shape_msgs::msg::SolidPrimitive prim;
  prim.type = prim.BOX;
  prim.dimensions = { 0.6, 0.6, 0.02 }; 
  geometry_msgs::msg::Pose tpose;
  tpose.orientation.w = 1.0;
  tpose.position.z = SURFACE_Z - 0.011;
  tpose.position.y = 0.38;
  table.primitives.push_back(prim);
  table.primitive_poses.push_back(tpose);
  table.operation = table.ADD;
  psi.applyCollisionObject(table);

  // Back trolley
  moveit_msgs::msg::CollisionObject back_trolley;
  back_trolley.header.frame_id = FIXED_FRAME;
  back_trolley.id = "back_trolley";
  shape_msgs::msg::SolidPrimitive back_prim;
  back_prim.type = prim.BOX;
  back_prim.dimensions = { 0.6, 0.2, 0.02 }; 
  geometry_msgs::msg::Pose back_pose;
  back_pose.orientation.w = 1.0;
  back_pose.position.z = SURFACE_Z - 0.011;
  back_pose.position.y = -0.18;
  back_trolley.primitives.push_back(back_prim);
  back_trolley.primitive_poses.push_back(back_pose);
  back_trolley.operation = back_trolley.ADD;
  psi.applyCollisionObject(back_trolley);
  

  for (const auto& state : cube_states_)
  {
    moveit_msgs::msg::CollisionObject obj;
    obj.id              = state.info.name + "_placed";
    obj.header.frame_id = FIXED_FRAME;
    shape_msgs::msg::SolidPrimitive shape;
    shape.type       = shape_msgs::msg::SolidPrimitive::BOX;
    shape.dimensions = { CUBE_SIZE, CUBE_SIZE, CUBE_SIZE };
    geometry_msgs::msg::Pose pose;
    pose.position.x    = state.info.x;
    pose.position.y    = state.info.y;
    pose.position.z    = state.info.z;
    pose.orientation.w = 1.0;
    obj.primitives.push_back(shape);
    obj.primitive_poses.push_back(pose);
    obj.operation = moveit_msgs::msg::CollisionObject::ADD;
    psi.applyCollisionObject(obj);
  }

  RCLCPP_INFO(LOGGER, "Planning scene ready.");
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

void RetrievalNode::removeCubeFromScene(const std::string& cube_name)
{
  moveit::planning_interface::PlanningSceneInterface psi;
  for (const auto& id : { cube_name + "_placed", cube_name + "_staged" })
  {
    moveit_msgs::msg::CollisionObject obj;
    obj.id              = id;
    obj.header.frame_id = FIXED_FRAME;
    obj.operation       = moveit_msgs::msg::CollisionObject::REMOVE;
    psi.applyCollisionObject(obj);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
}

void RetrievalNode::addCubeToStaging(const std::string& cube_name, int slot)
{
  moveit::planning_interface::PlanningSceneInterface psi;
  const auto& s = staging_slots_[slot];
  moveit_msgs::msg::CollisionObject obj;
  obj.id              = cube_name + "_staged";
  obj.header.frame_id = FIXED_FRAME;
  shape_msgs::msg::SolidPrimitive shape;
  shape.type       = shape_msgs::msg::SolidPrimitive::BOX;
  shape.dimensions = { CUBE_SIZE, CUBE_SIZE, CUBE_SIZE };
  geometry_msgs::msg::Pose pose;
  pose.position.x    = s.x;
  pose.position.y    = s.y;
  pose.position.z    = SURFACE_Z + CUBE_SIZE / 2.0;
  pose.orientation.w = 1.0;
  obj.primitives.push_back(shape);
  obj.primitive_poses.push_back(pose);
  obj.operation = moveit_msgs::msg::CollisionObject::ADD;
  psi.applyCollisionObject(obj);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
}

void RetrievalNode::cleanupAfterFailedExecution(const std::string& cube_name)
{
  moveit::planning_interface::PlanningSceneInterface psi;
  moveit_msgs::msg::AttachedCollisionObject detach;
  detach.object.id              = cube_name;
  detach.object.header.frame_id = FIXED_FRAME;
  detach.object.operation       = moveit_msgs::msg::CollisionObject::REMOVE;
  detach.link_name              = HAND_FRAME;
  psi.applyAttachedCollisionObject(detach);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
}

//    MTC TASK                                                                   
mtc::Task RetrievalNode::createPickTask(
  const std::string& cube_name,
  double place_x, double place_y, double place_z,
  bool is_top_cube)
{
  mtc::Task task;
  task.stages()->setName("retrieve_" + cube_name);
  task.loadRobotModel(node_);

  task.setProperty("group",    ARM_GROUP);
  task.setProperty("eef",      HAND_GROUP);
  task.setProperty("ik_frame", HAND_FRAME);

  auto optimising_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  optimising_planner->setProperty("longest_valid_segment_fraction", 0.001);
  optimising_planner->setPlannerId("RRTstarkConfigDefault");
  optimising_planner->setTimeout(10.0);

  auto optimising_planner_pick = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  optimising_planner_pick->setProperty("longest_valid_segment_fraction", 0.001);
  optimising_planner_pick->setPlannerId("RRTstarkConfigDefault");
  optimising_planner_pick->setTimeout(20.0);

  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  interpolation_planner->setMaxVelocityScalingFactor(0.1);
  interpolation_planner->setMaxAccelerationScalingFactor(0.1);

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.3);
  cartesian_planner->setMaxAccelerationScalingFactor(0.3);
  cartesian_planner->setStepSize(0.001);
  cartesian_planner->setJumpThreshold(2.0);

  //    CURRENT STATE                                                          
  mtc::Stage* current_state_ptr = nullptr;
  {
    auto stage = std::make_unique<mtc::stages::CurrentState>("current state");
    current_state_ptr = stage.get();
    task.add(std::move(stage));
  }

  //    OPEN GRIPPER                                                           
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper", interpolation_planner);
    stage->setGroup(HAND_GROUP);
    stage->setGoal("open");
    task.add(std::move(stage));
  }

  //    MOVE TO PICK                                                           
  {
    auto stage = std::make_unique<mtc::stages::Connect>(
      "move to pick",
      mtc::stages::Connect::GroupPlannerVector{ { ARM_GROUP, optimising_planner_pick } });
    stage->setTimeout(20.0);
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage));
  }

  //    PICK CONTAINER                                                         
  {
    auto pick = std::make_unique<mtc::SerialContainer>("pick " + cube_name);
    task.properties().exposeTo(pick->properties(), { "eef", "group", "ik_frame" });
    pick->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

    // Allow cube vs pyramid collisions before approach
    {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
            "allow cube vs pyramid collisions");
        std::vector<std::string> others;
        for (const auto& s : cube_states_)
            if (s.info.name != cube_name && s.location == CubeLocation::PYRAMID)
                others.push_back(s.info.name + "_placed");
        stage->allowCollisions(cube_name + "_placed", others, true);
        pick->insert(std::move(stage));
    }

    // Approach
    {
        auto stage = std::make_unique<mtc::stages::MoveRelative>("approach", cartesian_planner);
        stage->properties().set("marker_ns", "approach");
        stage->properties().set("link", HAND_FRAME);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        stage->setMinMaxDistance(0.01, 0.10);

        if (is_top_cube)
        {
            geometry_msgs::msg::TwistStamped twist;
            twist.header.frame_id = HAND_FRAME;
            twist.twist.linear.z  = 1.0;
            stage->setDirection(twist);
        }
        else
        {
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = FIXED_FRAME;
            vec.vector.z        = -1.0;
            stage->setDirection(vec);
        }
        pick->insert(std::move(stage));
    }

    // Generate grasp pose + IK
    {
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose("open");
      stage->setObject(cube_name + "_placed");
      stage->setAngleDelta(M_PI / 2);
      stage->setMonitoredStage(current_state_ptr);

      Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
      Eigen::Quaterniond q = Eigen::Quaterniond(
        Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
      grasp_frame_transform.linear() = q.matrix();

      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp IK", std::move(stage));
      wrapper->setMaxIKSolutions(32);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(grasp_frame_transform, HAND_FRAME);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      pick->insert(std::move(wrapper));
    }

    // Allow collision: gripper + cube
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
        "allow collision (gripper, " + cube_name + ")");
      stage->allowCollisions(cube_name + "_placed",
        task.getRobotModel()
          ->getJointModelGroup(HAND_GROUP)
          ->getLinkModelNamesWithCollisionGeometry(),
        true);
      pick->insert(std::move(stage));
    }

    // Close gripper
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close gripper", interpolation_planner);
      stage->setGroup(HAND_GROUP);
      stage->setGoal("closed");
      pick->insert(std::move(stage));
    }

    // Attach cube
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
        "attach " + cube_name);
      stage->attachObject(cube_name + "_placed", HAND_FRAME);
      pick->insert(std::move(stage));
    }

    // Lift
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lift", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.0, 0.20);
      stage->setIKFrame(HAND_FRAME);
      stage->properties().set("marker_ns", "lift");
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = FIXED_FRAME;
      vec.vector.z        = 1.0;
      stage->setDirection(vec);
      pick->insert(std::move(stage));
    }

    // Disallow cube vs pyramid collisions after lift
    {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
            "disallow cube vs pyramid collisions");
        std::vector<std::string> others;
        for (const auto& s : cube_states_)
            if (s.info.name != cube_name && s.location == CubeLocation::PYRAMID)
                others.push_back(s.info.name + "_placed");
        stage->allowCollisions(cube_name + "_placed", others, false);
        pick->insert(std::move(stage));
    }

    task.add(std::move(pick));
  }

  {
  // Ready pose
    auto stage = std::make_unique<mtc::stages::MoveTo>("ready pose", optimising_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGoal("ready_pose2");
    task.add(std::move(stage));
  }

  //    PRE-PLACE                                                              
  //
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("pre-place", optimising_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    geometry_msgs::msg::PoseStamped pre_place;
    pre_place.header.frame_id    = FIXED_FRAME;
    pre_place.pose.position.x    = place_x;
    pre_place.pose.position.y    = place_y;
    pre_place.pose.position.z    = place_z + PRE_PLACE_HEIGHT;
    pre_place.pose.orientation.x = GRIPPER_DOWN_QX;
    pre_place.pose.orientation.y = GRIPPER_DOWN_QY;
    pre_place.pose.orientation.z = GRIPPER_DOWN_QZ;
    pre_place.pose.orientation.w = GRIPPER_DOWN_QW;
    stage->setGoal(pre_place);

    moveit_msgs::msg::Constraints path_constraints;
    moveit_msgs::msg::JointConstraint pan_constraint;
    pan_constraint.joint_name      = "shoulder_pan_joint";
    pan_constraint.position        = 0.0;
    pan_constraint.tolerance_above = 1.6;
    pan_constraint.tolerance_below = 1.6;
    pan_constraint.weight          = 1.0;
    path_constraints.joint_constraints.push_back(pan_constraint);
    stage->setPathConstraints(path_constraints);

    task.add(std::move(stage));
  }

  //    PLACE CONTAINER                                                        
  {
    auto place = std::make_unique<mtc::SerialContainer>("place " + cube_name);
    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
    place->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

    // Lower
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lower", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.05, PRE_PLACE_HEIGHT + 0.05);
      stage->setIKFrame(HAND_FRAME);
      stage->properties().set("marker_ns", "lower");
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = FIXED_FRAME;
      vec.vector.z        = -1.0;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    // Open gripper
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper", interpolation_planner);
      stage->setGroup(HAND_GROUP);
      stage->setGoal("open");
      place->insert(std::move(stage));
    }

    // Detach cube
    {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
            "detach " + cube_name);
        stage->detachObject(cube_name + "_placed", HAND_FRAME);
        place->insert(std::move(stage));
    }

    // Retreat
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.05, RETREAT_HEIGHT);
      stage->setIKFrame(HAND_FRAME);
      stage->properties().set("marker_ns", "retreat");
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = FIXED_FRAME;
      vec.vector.z        = 1.0;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    task.add(std::move(place));
  }

  return task;
}

//    PICK AND PLACE                                                             
bool RetrievalNode::pickAndPlace(
  const CubeState& cube,
  double place_x, double place_y, double place_z)
{
  constexpr int MAX_ATTEMPTS = 3;
  for (int attempt = 1; attempt <= MAX_ATTEMPTS; ++attempt)
  {
    auto task = createPickTask(
      cube.info.name,
      place_x, place_y, place_z,
      cube.info.name == "cube_6");

    try { task.init(); }
    catch (mtc::InitStageException& e)
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Init failed: " << e);
      continue;
    }

    if (!task.plan(3))
    {
      RCLCPP_WARN(LOGGER, "Planning failed (attempt %d/%d)", attempt, MAX_ATTEMPTS);
      continue;
    }

    task.introspection().publishSolution(*task.solutions().front());
    auto result = task.execute(*task.solutions().front());

    if (result.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
      return true;

    RCLCPP_WARN(LOGGER, "Execution failed (attempt %d/%d)", attempt, MAX_ATTEMPTS);
    cleanupAfterFailedExecution(cube.info.name + "_placed");
    tryReturnHome();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  return false;
}

//    RETURN HOME                                                                
bool RetrievalNode::tryReturnHome()
{
  mtc::Task task;
  task.stages()->setName("return_home");
  task.loadRobotModel(node_);
  task.setProperty("group", ARM_GROUP);
  task.setProperty("ik_frame", HAND_FRAME);

  auto optimising_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  optimising_planner->setProperty("longest_valid_segment_fraction", 0.001);
  optimising_planner->setPlannerId("RRTstarkConfigDefault");
  optimising_planner->setTimeout(10.0);

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.3);
  cartesian_planner->setMaxAccelerationScalingFactor(0.3);
  cartesian_planner->setStepSize(0.001);
  cartesian_planner->setJumpThreshold(2.0);

  {
    auto stage = std::make_unique<mtc::stages::CurrentState>("current");
    task.add(std::move(stage));
  }
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setMinMaxDistance(0.005, 0.05);
    stage->setIKFrame(HAND_FRAME);
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = FIXED_FRAME;
    vec.vector.z        = 1.0;
    stage->setDirection(vec);
    task.add(std::move(stage));
  }
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("go home", optimising_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGoal("ready_pose2");
    task.add(std::move(stage));
  }

  try { task.init(); }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_WARN_STREAM(LOGGER, "Return home init failed: " << e);
    return false;
  }
  if (!task.plan(3)) return false;
  auto result = task.execute(*task.solutions().front());
  return result.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
}

//    RUN                                                                        
void RetrievalNode::run()
{
  clearScene();
  setupPlanningScene();

  sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/retrieve_cube", 10,
    [this](std_msgs::msg::String::SharedPtr msg)
    {
      if (!request_received_)
      {
        requested_cube_   = msg->data;
        request_received_ = true;
      }
    });

  RCLCPP_INFO(LOGGER, "Waiting for request on /retrieve_cube ...");
  RCLCPP_INFO(LOGGER, "Example: ros2 topic pub /retrieve_cube std_msgs/msg/String \"data: cube_2\" --once");

  while (rclcpp::ok() && !request_received_)
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

  if (!rclcpp::ok()) return;

  const std::string target = requested_cube_;

  if (!isCubeInPyramid(target))
  {
    RCLCPP_ERROR(LOGGER, "%s is not in the pyramid", target.c_str());
    return;
  }

  auto removal_order = computeRemovalOrder(target);
  RCLCPP_INFO(LOGGER, "Removal order:");
  for (const auto& n : removal_order)
    RCLCPP_INFO(LOGGER, "  → %s", n.c_str());

  for (const auto& cube_name : removal_order)
  {
    auto& state       = getCubeState(cube_name);
    bool  is_target   = (cube_name == target);
    bool  success     = false;

    if (is_target)
    {
      success = pickAndPlace(state, DELIVERY_X, DELIVERY_Y, SURFACE_Z + CUBE_SIZE / 2.0);
      if (success)
      {
        removeCubeFromScene(cube_name);
        state.location = CubeLocation::DELIVERED;
        RCLCPP_INFO(LOGGER, "%s delivered", cube_name.c_str());
      }
    }
    else
    {
      int slot = getNextFreeSlot();
      if (slot < 0) { RCLCPP_ERROR(LOGGER, "No free staging slots"); return; }

      const auto& s = staging_slots_[slot];
      success = pickAndPlace(state, s.x, s.y, SURFACE_Z + CUBE_SIZE / 2.0);
      if (success)
      {
        removeCubeFromScene(cube_name);
        addCubeToStaging(cube_name, slot);
        staging_slots_[slot].occupied = true;
        state.location    = CubeLocation::STAGING;
        state.staging_slot = slot;
        RCLCPP_INFO(LOGGER, "%s staged at slot %d", cube_name.c_str(), slot);
      }
    }

    if (!success)
    {
      RCLCPP_ERROR(LOGGER, "Failed to move %s — aborting", cube_name.c_str());
      tryReturnHome();
      return;
    }

    if (!tryReturnHome())
      RCLCPP_WARN(LOGGER, "Could not return home after %s", cube_name.c_str());
  }

  RCLCPP_INFO(LOGGER, "=== Retrieval complete ===");
  request_received_ = false;
}

//    MAIN                                                                       
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<RetrievalNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto spin_thread = std::make_unique<std::thread>([&executor, &node]() {
    executor.add_node(node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(node->getNodeBaseInterface());
  });
  node->run();
  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}