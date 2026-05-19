#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <geometry_msgs/msg/pose_array.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

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

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_ss2");
namespace mtc = moveit::task_constructor;

//      ROBOT CONFIG                                                                                                                             
static const std::string ARM_GROUP   = "ur_onrobot_manipulator";
static const std::string HAND_GROUP  = "ur_onrobot_gripper";
static const std::string HAND_FRAME  = "gripper_tcp";
static const std::string FIXED_FRAME = "base_link";

//      CUBE DIMENSIONS                                                                                                                       
static constexpr double CUBE_SIZE       = 0.050;
static constexpr double PLACE_CLEARANCE = 0.005;
static constexpr double SURFACE_Z       = 0.001;

//      PYRAMID LAYOUT                                                                                                                         
static constexpr double PYRAMID_X = 0.012;
static constexpr double PYRAMID_Y = 0.320;
static constexpr double STEP      = CUBE_SIZE + 0.005;

//      GRIPPER DOWN ORIENTATION (measured, 180 deg around world Y)                               
static constexpr double GRIPPER_DOWN_QX = 0.0;
static constexpr double GRIPPER_DOWN_QY = 1.0;
static constexpr double GRIPPER_DOWN_QZ = 0.0;
static constexpr double GRIPPER_DOWN_QW = 0.0;

//      PRE-PLACE HEIGHT OFFSET                                                                                                       
static constexpr double PRE_PLACE_HEIGHT = 0.10;

struct CubeInfo
{
  std::string name;
  double pick_x, pick_y;
  double place_x, place_y;
  int layer;
  double yaw;
};

static const std::vector<CubeInfo> HARDCODED_CUBES = {
  // name,    pick_x,  pick_y,  place_x,                place_y,   layer, yaw
  { "cube_1", -0.154,  0.400,  PYRAMID_X + STEP,       PYRAMID_Y, 1,     0.0          }, 
  { "cube_2", -0.090,  0.390,  PYRAMID_X,              PYRAMID_Y, 1,     0.0          }, 
  { "cube_3",  0.130,  0.400,  PYRAMID_X - STEP,       PYRAMID_Y, 1,    -M_PI / 4     },
  { "cube_4", -0.100,  0.250,  PYRAMID_X + STEP / 2.0, PYRAMID_Y, 2,     M_PI / 4     },  
  { "cube_5", -0.150,  0.200,  PYRAMID_X - STEP / 2.0, PYRAMID_Y, 2,     M_PI / 5     }, 
  { "cube_6",  0.100,  0.200,  PYRAMID_X,              PYRAMID_Y, 3,    -M_PI / 3     }, 
};

double placeCentreZ(int layer)
{
  return SURFACE_Z + PLACE_CLEARANCE + (layer - 1) * CUBE_SIZE + CUBE_SIZE / 2.0;
}

static constexpr double GRASP_TCP_TO_CUBE_OFFSET = 0.0;  // Potentially unecessary

double placeTcpZ(int layer)
{
  return placeCentreZ(layer) + GRASP_TCP_TO_CUBE_OFFSET;
}

double prePlaceHeight(int layer) {
  return (layer == 3) ? 0.05 : 0.10;
}

class MTCPyramidNode
{
public:
  MTCPyramidNode(const rclcpp::NodeOptions& options);
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
  void run();

private:
  void clearScene();
  void setupPlanningScene(const std::vector<CubeInfo>& cubes);
  void removePickObject(const std::string& cube_name);
  void addPlacedObject(const CubeInfo& cube);
  bool runPickAndPlace(const CubeInfo& cube);
  bool tryReturnHome();
  mtc::Task createPickAndPlaceTask(const CubeInfo& cube);
  rclcpp::Node::SharedPtr node_;
};

MTCPyramidNode::MTCPyramidNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_ss2", options) }
{
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
MTCPyramidNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

void MTCPyramidNode::clearScene()
{
  moveit::planning_interface::PlanningSceneInterface psi;
  std::vector<std::string> to_remove = {
    "table", "back_trolley",
    "cube_1", "cube_2", "cube_3", "cube_4", "cube_5", "cube_6",
    "cube_1_placed", "cube_2_placed", "cube_3_placed",
    "cube_4_placed", "cube_5_placed", "cube_6_placed"
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

void MTCPyramidNode::setupPlanningScene(const std::vector<CubeInfo>& cubes)
{
  moveit::planning_interface::PlanningSceneInterface psi;
  for (const auto& cube : cubes)
  {
    moveit_msgs::msg::CollisionObject obj;
    obj.id = cube.name;
    obj.header.frame_id = FIXED_FRAME;
    shape_msgs::msg::SolidPrimitive shape;
    shape.type = shape_msgs::msg::SolidPrimitive::BOX;
    shape.dimensions = { CUBE_SIZE, CUBE_SIZE, CUBE_SIZE };
    geometry_msgs::msg::Pose pose;
    pose.position.x    = cube.pick_x;
    pose.position.y    = cube.pick_y;
    pose.position.z    = SURFACE_Z + CUBE_SIZE / 2.0;
    // pose.orientation.w = 1.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, cube.yaw);
    pose.orientation = tf2::toMsg(q);
    
    obj.primitives.push_back(shape);
    obj.primitive_poses.push_back(pose);
    obj.operation = moveit_msgs::msg::CollisionObject::ADD;
    psi.applyCollisionObject(obj);
    RCLCPP_INFO(LOGGER, "Added collision object: %s at (%.3f, %.3f)",
                cube.name.c_str(), cube.pick_x, cube.pick_y);
  }
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
  
  RCLCPP_INFO(LOGGER, "Planning scene ready.");
}

void MTCPyramidNode::removePickObject(const std::string& cube_name)
{
  moveit::planning_interface::PlanningSceneInterface psi;
  moveit_msgs::msg::CollisionObject obj;
  obj.id = cube_name;
  obj.header.frame_id = FIXED_FRAME;
  obj.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  psi.applyCollisionObject(obj);
}

void MTCPyramidNode::addPlacedObject(const CubeInfo& cube)
{
  moveit::planning_interface::PlanningSceneInterface psi;

  {
    moveit_msgs::msg::CollisionObject remove_old;
    remove_old.id = cube.name;
    remove_old.header.frame_id = FIXED_FRAME;
    remove_old.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    psi.applyCollisionObject(remove_old);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  {
    moveit_msgs::msg::CollisionObject obj;
    obj.id = cube.name + "_placed";
    obj.header.frame_id = FIXED_FRAME;
    shape_msgs::msg::SolidPrimitive shape;
    shape.type = shape_msgs::msg::SolidPrimitive::BOX;
    shape.dimensions = { CUBE_SIZE, CUBE_SIZE, CUBE_SIZE };
    geometry_msgs::msg::Pose pose;
    pose.position.x    = cube.place_x;
    pose.position.y    = cube.place_y;
    pose.position.z    = placeCentreZ(cube.layer) - PLACE_CLEARANCE;
    pose.orientation.w = 1.0;
    obj.primitives.push_back(shape);
    obj.primitive_poses.push_back(pose);
    obj.operation = moveit_msgs::msg::CollisionObject::ADD;
    psi.applyCollisionObject(obj);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  RCLCPP_INFO(LOGGER, "Placed %s at (%.3f, %.3f) layer %d",
              cube.name.c_str(), cube.place_x, cube.place_y, cube.layer);
}

mtc::Task MTCPyramidNode::createPickAndPlaceTask(const CubeInfo& cube)
{
  mtc::Task task;
  task.stages()->setName("pick_place_" + cube.name);
  task.loadRobotModel(node_);

  task.setProperty("group",    ARM_GROUP);
  task.setProperty("eef",      HAND_GROUP);
  task.setProperty("ik_frame", HAND_FRAME);

  auto sampling_planner      = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto cartesian_planner     = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(0.002);

  //      CURRENT STATE                                                                                                                       
  mtc::Stage* current_state_ptr = nullptr;
  {
    auto stage = std::make_unique<mtc::stages::CurrentState>("current state");
    current_state_ptr = stage.get();
    task.add(std::move(stage));
  }

  //      OPEN GRIPPER                                                                                                                         
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper", interpolation_planner);
    stage->setGroup(HAND_GROUP);
    stage->setGoal("open");
    task.add(std::move(stage));
  }

  //      ALLOW CABLE CONNECTOR COLLISIONS WITH ALL CUBES                                                   
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
      "allow cable connector collisions");
    stage->allowCollisions(
      std::vector<std::string>{ "cable_connector_0", "cable_connector_1" },
      std::vector<std::string>{
        "cube_1", "cube_2", "cube_3", "cube_4", "cube_5", "cube_6",
        "cube_1_placed", "cube_2_placed", "cube_3_placed",
        "cube_4_placed", "cube_5_placed", "cube_6_placed"
      },
      true);
    task.add(std::move(stage));
  }

  //      MOVE TO PRE-PICK                                                                                                                 
  {
    auto stage = std::make_unique<mtc::stages::Connect>(
      "move to pick",
      mtc::stages::Connect::GroupPlannerVector{ { ARM_GROUP, sampling_planner } });
    stage->setTimeout(15.0);
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage));
  }

  //      PICK CONTAINER                                                                                                                     
  {
    auto pick = std::make_unique<mtc::SerialContainer>("pick " + cube.name);
    task.properties().exposeTo(pick->properties(), { "eef", "group", "ik_frame" });
    pick->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

    // Approach 
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("approach", cartesian_planner);
      stage->properties().set("marker_ns", "approach");
      stage->properties().set("link", HAND_FRAME);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.01, 0.10);
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = FIXED_FRAME;
      vec.vector.z = -1.0;
      stage->setDirection(vec);
      pick->insert(std::move(stage));
    }

    // Generate grasp pose + IK
    {
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose("open");
      stage->setObject(cube.name);
      stage->setAngleDelta(M_PI / 6);  // Param
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
        "allow collision (gripper, " + cube.name + ")");
      stage->allowCollisions(cube.name,
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
        "attach " + cube.name);
      stage->attachObject(cube.name, HAND_FRAME);
      pick->insert(std::move(stage));
    }

    // Allow attached cube to collide with other source cubes during pick
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
        "allow attached cube vs other cubes");
      std::vector<std::string> others;
      for (const auto& c : HARDCODED_CUBES)
        if (c.name != cube.name) others.push_back(c.name);
      stage->allowCollisions(cube.name, others, true);
      pick->insert(std::move(stage));
    }

    // Lift
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lift", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.05, 0.18);
      stage->setIKFrame(HAND_FRAME);
      stage->properties().set("marker_ns", "lift");
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = FIXED_FRAME;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      pick->insert(std::move(stage));
    }

    task.add(std::move(pick));
  }

  //      PRE-PLACE   
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("pre-place", sampling_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    geometry_msgs::msg::PoseStamped pre_place;
    pre_place.header.frame_id = FIXED_FRAME;
    pre_place.pose.position.x = cube.place_x;
    pre_place.pose.position.y = cube.place_y;
    pre_place.pose.position.z = placeTcpZ(cube.layer) + prePlaceHeight(cube.layer);
    pre_place.pose.orientation.x = GRIPPER_DOWN_QX;
    pre_place.pose.orientation.y = GRIPPER_DOWN_QY;
    pre_place.pose.orientation.z = GRIPPER_DOWN_QZ;
    pre_place.pose.orientation.w = GRIPPER_DOWN_QW;
    stage->setGoal(pre_place);
    task.add(std::move(stage));
  }

  // //      PRE-PLACE: Cartesian                          
  // {
  //   auto stage = std::make_unique<mtc::stages::MoveTo>("pre-place", cartesian_planner);
  //   stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  //   geometry_msgs::msg::PoseStamped pre_place;
  //   pre_place.header.frame_id = FIXED_FRAME;
  //   pre_place.pose.position.x = cube.place_x;
  //   pre_place.pose.position.y = cube.place_y;
  //   pre_place.pose.position.z = placeTcpZ(cube.layer) + prePlaceHeight(cube.layer);
  //   pre_place.pose.orientation.x = GRIPPER_DOWN_QX;
  //   pre_place.pose.orientation.y = GRIPPER_DOWN_QY;
  //   pre_place.pose.orientation.z = GRIPPER_DOWN_QZ;
  //   pre_place.pose.orientation.w = GRIPPER_DOWN_QW;
  //   stage->setGoal(pre_place);
  //   task.add(std::move(stage));
  // }

  //      PLACE CONTAINER                                                                                                                   
  {
    auto place = std::make_unique<mtc::SerialContainer>("place " + cube.name);
    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
    place->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

    // Collision for diagonal cubes in simulation
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
        "allow attached cube vs same-layer placed cubes");
      std::vector<std::string> same_layer_placed;
      for (const auto& c : HARDCODED_CUBES)
        if (c.name != cube.name && c.layer == cube.layer)
          same_layer_placed.push_back(c.name + "_placed");
      
      if (!same_layer_placed.empty()) {
        stage->allowCollisions(cube.name, same_layer_placed, true);
        stage->allowCollisions(same_layer_placed,
          task.getRobotModel()
            ->getJointModelGroup(HAND_GROUP)
            ->getLinkModelNamesWithCollisionGeometry(),
          true);
      }
      place->insert(std::move(stage));
    }
    // Lower
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lower", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.04, PRE_PLACE_HEIGHT + 0.05);
      stage->setIKFrame(HAND_FRAME);
      stage->properties().set("marker_ns", "lower");
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = FIXED_FRAME;
      vec.vector.z = -1.0;
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

    task.add(std::move(place));
  }

  return task;
}

bool MTCPyramidNode::runPickAndPlace(const CubeInfo& cube)
{
  RCLCPP_INFO(LOGGER, "=== Starting pick and place for %s (layer %d) ===",
              cube.name.c_str(), cube.layer);

  auto task = createPickAndPlaceTask(cube);

  try { task.init(); }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task init failed: " << e);
    return false;
  }

  if (!task.plan(10))
  {
    RCLCPP_ERROR(LOGGER, "Task planning failed for %s", cube.name.c_str());
    return false;
  }

  task.introspection().publishSolution(*task.solutions().front());

  auto result = task.execute(*task.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR(LOGGER, "Task execution failed for %s", cube.name.c_str());
    removePickObject(cube.name);
    // Also detach in case it was attached during execution
    moveit_msgs::msg::AttachedCollisionObject detach;
    detach.object.id = cube.name;
    detach.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    detach.link_name = HAND_FRAME;
    moveit::planning_interface::PlanningSceneInterface psi;
    psi.applyAttachedCollisionObject(detach);
    return false;
  }

  moveit_msgs::msg::AttachedCollisionObject detach;
  detach.object.id = cube.name;
  detach.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  detach.link_name = HAND_FRAME;
  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyAttachedCollisionObject(detach);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  RCLCPP_INFO(LOGGER, "=== Completed %s ===", cube.name.c_str());
  return true;
}

bool MTCPyramidNode::tryReturnHome()
{
  RCLCPP_INFO(LOGGER, "Attempting return home...");

  mtc::Task home_task;
  home_task.stages()->setName("return_home");
  home_task.loadRobotModel(node_);
  home_task.setProperty("group", ARM_GROUP);
  home_task.setProperty("ik_frame", HAND_FRAME);

  auto sampling_planner  = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(0.002);

  {
    auto stage = std::make_unique<mtc::stages::CurrentState>("current");
    home_task.add(std::move(stage));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setMinMaxDistance(0.005, 0.05);
    stage->setIKFrame(HAND_FRAME);
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = FIXED_FRAME;
    vec.vector.z = 1.0;
    stage->setDirection(vec);
    home_task.add(std::move(stage));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("go home", sampling_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGoal("test_configuration"); // ready_pose or test_congifuration
    home_task.add(std::move(stage));
  }

  try { home_task.init(); }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_WARN_STREAM(LOGGER, "Return home init failed: " << e);
    return false;
  }
  if (!home_task.plan(3))
  {
    RCLCPP_WARN(LOGGER, "Return home planning failed");
    return false;
  }
  auto result = home_task.execute(*home_task.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_WARN(LOGGER, "Return home execution failed");
    return false;
  }
  return true;
}

void MTCPyramidNode::run()
{
  clearScene();

  std::vector<CubeInfo> cubes;
  geometry_msgs::msg::PoseArray::SharedPtr detected_poses = nullptr;

  RCLCPP_INFO(LOGGER, "Waiting for SS3 detected objects on /detected_objects ...");

  auto sub = node_->create_subscription<geometry_msgs::msg::PoseArray>(
    "/detected_objects", 10,
    [&detected_poses](geometry_msgs::msg::PoseArray::SharedPtr msg)
    { detected_poses = msg; });

  auto deadline = node_->now() + rclcpp::Duration::from_seconds(5.0);
  while (rclcpp::ok() && node_->now() < deadline && detected_poses == nullptr)
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

  static const std::array<std::tuple<double, double, int>, 6> place_positions = {{
    { PYRAMID_X + STEP,       PYRAMID_Y, 1 },
    { PYRAMID_X,              PYRAMID_Y, 1 },
    { PYRAMID_X - STEP,       PYRAMID_Y, 1 },
    { PYRAMID_X + STEP / 2.0, PYRAMID_Y, 2 },
    { PYRAMID_X - STEP / 2.0, PYRAMID_Y, 2 },
    { PYRAMID_X,              PYRAMID_Y, 3 },
  }};

  if (detected_poses == nullptr || detected_poses->poses.empty())
  {
    RCLCPP_WARN(LOGGER, "No SS3 data — using hardcoded pick positions.");
    cubes = HARDCODED_CUBES;
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Received %zu poses from SS3.", detected_poses->poses.size());
    static const std::array<std::string, 6> names =
      { "cube_1", "cube_2", "cube_3", "cube_4", "cube_5", "cube_6" };
    for (size_t i = 0; i < std::min(detected_poses->poses.size(), size_t(6)); ++i)
    {
      const auto& p = detected_poses->poses[i];
      auto [px, py, layer] = place_positions[i];

      tf2::Quaternion q(p.orientation.x, p.orientation.y,
                        p.orientation.z, p.orientation.w);
      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

      cubes.push_back({ names[i], p.position.x, p.position.y,
                        px, py, layer, yaw });
    }
  }

  setupPlanningScene(cubes);

  int completed = 0;
  for (const auto& cube : cubes)
  {
    bool success = runPickAndPlace(cube);
    if (success)
    {
      addPlacedObject(cube);
      completed++;
    }
    else
    {
      RCLCPP_WARN(LOGGER, "Skipping %s after failure — continuing with remaining cubes.",
                  cube.name.c_str());
      removePickObject(cube.name);
    }

    if (!tryReturnHome())
      RCLCPP_WARN(LOGGER, "Could not return home after %s — continuing from current pose",
                  cube.name.c_str());
  }

  RCLCPP_INFO(LOGGER, "=== PYRAMID COMPLETE: %d/6 cubes placed ===", completed);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<MTCPyramidNode>(options);
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