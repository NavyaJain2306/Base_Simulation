#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>

#include "sml_messages/action/arm_command.hpp"

#include <map>
#include <set>
#include <utility>
#include <string>
#include <vector>

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

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;

using ArmCommand           = sml_messages::action::ArmCommand;
using GoalHandleArmCommand = rclcpp_action::ServerGoalHandle<ArmCommand>;

struct BlockConfig
{
  std::string id;
  int block_slot = -1;
  std::string pick_frame = "base_link";
  double pick_x;
  double pick_y;
  double pick_z;
  double place_x;
  double place_y;
  double place_z;
  std::string place_frame = "base_link";
};

static constexpr double BLOCK_DIM_X = 0.02;
static constexpr double BLOCK_DIM_Y = 0.02;
static constexpr double BLOCK_DIM_Z = 0.02;

struct StationTemplate
{
  enum class Kind { SHELF, BOX } kind;

  double origin_x;
  double origin_y;
  double origin_z;

  double box_dim_x{ 0.0 };
  double box_dim_y{ 0.0 };
  double box_dim_z{ 0.0 };
};

static const std::map<std::string, StationTemplate> STATION_TEMPLATES = {
  { "s_top",        { StationTemplate::Kind::SHELF, 0.5, -0.35, 0.0 } },
  { "s_left_mid",   { StationTemplate::Kind::SHELF, 0.5, -0.35, 0.0 } },
  { "s_center_mid", { StationTemplate::Kind::SHELF, 0.5, -0.35, 0.0 } },

  { "s_bot_left",   { StationTemplate::Kind::SHELF, 0.5, -0.35, 0.0 } },
  { "s_bot_center", { StationTemplate::Kind::SHELF, 0.5, -0.35, 0.0 } },

  { "cc_1",         { StationTemplate::Kind::BOX, 0.5, -0.35, 0.25, 1.0, 0.5, 0.2 } },
  { "wb_top",       { StationTemplate::Kind::BOX, 0.5, -0.35, 0.25, 1.0, 0.5, 0.2 } },
  { "wb_left",      { StationTemplate::Kind::BOX, 0.5, -0.35, 0.25, 1.0, 0.5, 0.2 } },
  { "wb_bottom",    { StationTemplate::Kind::BOX, 0.5, -0.35, 0.25, 1.0, 0.5, 0.2 } },
};

static const std::map<std::string, std::vector<std::string>> BLOCK_MODEL_NAMES = {
  { "s_top",        { "s_top_block_red",        "s_top_block_green",        "s_top_block_blue"        } },
  { "s_left_mid",   { "s_left_mid_block_red",   "s_left_mid_block_green",   "s_left_mid_block_blue"   } },
  { "s_center_mid", { "s_center_mid_block_red", "s_center_mid_block_green", "s_center_mid_block_blue" } },
  { "s_bot_left",   { "s_bot_left_block_red",   "s_bot_left_block_green",   "s_bot_left_block_blue"   } },
  { "s_bot_center", { "s_bot_center_block_red", "s_bot_center_block_green", "s_bot_center_block_blue" } },
};

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void setupPlanningScene();

private:

  rclcpp_action::GoalResponse handleGoal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const ArmCommand::Goal> goal);

  rclcpp_action::CancelResponse handleCancel(
      const std::shared_ptr<GoalHandleArmCommand> goal_handle);

  void handleAccepted(const std::shared_ptr<GoalHandleArmCommand> goal_handle);

  void execute(const std::shared_ptr<GoalHandleArmCommand> goal_handle);

  void registerBlockCollisionObject(const BlockConfig& block);

  void registerStationGeometry(const std::string& station_name);

  bool lookupLiveStationPose(const std::string& station_name, tf2::Transform& out_pose);

  void registerSiblingBlocks(const std::string& station_name, int picked_slot);

  mtc::Task createTask(const BlockConfig& block);

  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Server<ArmCommand>::SharedPtr action_server_;

  std::atomic<bool> busy_{ false };

  std::vector<std::string> current_station_object_ids_;

  std::vector<std::string> current_sibling_block_ids_;

  std::set<std::pair<std::string, int>> picked_slots_;

  std::map<int, std::string> slot_object_ids_;

  struct StoredPose { double x; double y; double z; std::string frame; };
  std::map<int, StoredPose> slot_last_pose_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  action_server_ = rclcpp_action::create_server<ArmCommand>(
      node_,
      "arm_command",
      std::bind(&MTCTaskNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MTCTaskNode::handleCancel, this, std::placeholders::_1),
      std::bind(&MTCTaskNode::handleAccepted, this, std::placeholders::_1));

  RCLCPP_INFO(LOGGER, "ArmCommand action server ready at /arm_command");
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

void MTCTaskNode::setupPlanningScene()
{

}

bool MTCTaskNode::lookupLiveStationPose(const std::string& station_name, tf2::Transform& out_pose)
{

  try
  {
    auto station_tf = tf_buffer_->lookupTransform(
        "base_footprint", station_name + "_live", tf2::TimePointZero,
        tf2::durationFromSec(0.5));
    tf2::fromMsg(station_tf.transform, out_pose);
    return true;
  }
  catch (const tf2::TransformException&)
  {
    return false;
  }
}

void MTCTaskNode::registerStationGeometry(const std::string& station_name)
{
  moveit::planning_interface::PlanningSceneInterface psi;

  if (!current_station_object_ids_.empty())
  {
    psi.removeCollisionObjects(current_station_object_ids_);
    current_station_object_ids_.clear();
  }

  auto it = STATION_TEMPLATES.find(station_name);
  if (it == STATION_TEMPLATES.end())
  {
    RCLCPP_WARN(LOGGER,
                "No station template registered for '%s' -- proceeding with NO "
                "obstacle geometry for this station. Add an entry to "
                "STATION_TEMPLATES if this station needs collision avoidance.",
                station_name.c_str());
    return;
  }
  const StationTemplate& tpl = it->second;

  tf2::Transform station_pose;
  bool have_live = lookupLiveStationPose(station_name, station_pose);
  if (!have_live)
  {
    RCLCPP_WARN(LOGGER,
                "No live TF for '%s_live' -- falling back to the static "
                "default offset. Is gazebo_station_relay running?",
                station_name.c_str());
    station_pose.setOrigin(tf2::Vector3(tpl.origin_x, tpl.origin_y, tpl.origin_z));
    station_pose.setRotation(tf2::Quaternion(0, 0, 0, 1));
  }

  auto compose_pose = [&](double dx, double dy, double dz) {
    tf2::Transform local(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(dx, dy, dz));
    tf2::Transform world = station_pose * local;
    geometry_msgs::msg::Pose p;
    tf2::toMsg(world, p);
    return p;
  };

  std::vector<moveit_msgs::msg::CollisionObject> objects;

  if (tpl.kind == StationTemplate::Kind::SHELF)
  {
    auto make_post = [&](const std::string& id, double dx, double dy) {
      moveit_msgs::msg::CollisionObject post;
      post.id              = id;
      post.header.frame_id = "base_footprint";
      post.primitives.resize(1);
      post.primitives[0].type       = shape_msgs::msg::SolidPrimitive::CYLINDER;
      post.primitives[0].dimensions = { 0.9, 0.02 };
      post.pose = compose_pose(dx, dy, 0.45);
      return post;
    };

    auto make_row = [&](const std::string& id, double dz) {
      moveit_msgs::msg::CollisionObject row;
      row.id              = id;
      row.header.frame_id = "base_footprint";
      row.primitives.resize(1);
      row.primitives[0].type       = shape_msgs::msg::SolidPrimitive::BOX;
      row.primitives[0].dimensions = { 0.9, 0.4, 0.02 };
      row.pose = compose_pose(0.0, 0.0, dz);
      return row;
    };

    objects.push_back(make_post("station_post_front_right", 0.43, -0.18));
    objects.push_back(make_post("station_post_front_left",  0.43,  0.18));
    objects.push_back(make_post("station_post_back_right", -0.43, -0.18));
    objects.push_back(make_post("station_post_back_left",  -0.43,  0.18));
    objects.push_back(make_row("station_shelf_row_bottom", 0.15));
    objects.push_back(make_row("station_shelf_row_top",    0.85));
  }
  else
  {
    moveit_msgs::msg::CollisionObject box;
    box.id              = "station_box";
    box.header.frame_id = "base_footprint";
    box.primitives.resize(1);
    box.primitives[0].type       = shape_msgs::msg::SolidPrimitive::BOX;
    box.primitives[0].dimensions = { tpl.box_dim_x, tpl.box_dim_y, tpl.box_dim_z };
    box.pose = compose_pose(0.0, 0.0, 0.0);
    objects.push_back(box);
  }

  for (const auto& obj : objects)
  {
    current_station_object_ids_.push_back(obj.id);
  }
  psi.applyCollisionObjects(objects);

  RCLCPP_INFO(LOGGER, "Registered station geometry for '%s' (%zu objects, live_tf=%s)",
              station_name.c_str(), objects.size(), have_live ? "yes" : "NO (fallback)");
}

void MTCTaskNode::registerBlockCollisionObject(const BlockConfig& block)
{
  moveit::planning_interface::PlanningSceneInterface psi;

  if (block.block_slot >= 0)
  {
    auto slot_it = slot_object_ids_.find(block.block_slot);
    if (slot_it != slot_object_ids_.end() && slot_it->second != block.id)
    {
      RCLCPP_INFO(LOGGER,
                  "Slot %d previously held by '%s', now '%s' -- removing the "
                  "stale object.",
                  block.block_slot, slot_it->second.c_str(), block.id.c_str());
      psi.removeCollisionObjects({ slot_it->second });
    }
    slot_object_ids_[block.block_slot] = block.id;
  }

  moveit_msgs::msg::CollisionObject obj;
  obj.id              = block.id;
  obj.header.frame_id = block.pick_frame;
  obj.primitives.resize(1);
  obj.primitives[0].type       = shape_msgs::msg::SolidPrimitive::BOX;
  obj.primitives[0].dimensions = { BLOCK_DIM_X, BLOCK_DIM_Y, BLOCK_DIM_Z };
  obj.pose.position.x    = block.pick_x;
  obj.pose.position.y    = block.pick_y;
  obj.pose.position.z    = block.pick_z;
  obj.pose.orientation.w = 1.0;
  psi.applyCollisionObject(obj);

  RCLCPP_INFO(LOGGER, "Registered block '%s' at (%.2f, %.2f, %.2f)",
              block.id.c_str(), block.pick_x, block.pick_y, block.pick_z);
}

void MTCTaskNode::registerSiblingBlocks(const std::string& station_name, int picked_slot)
{
  moveit::planning_interface::PlanningSceneInterface psi;

  if (!current_sibling_block_ids_.empty())
  {
    psi.removeCollisionObjects(current_sibling_block_ids_);
    current_sibling_block_ids_.clear();
  }

  auto it = BLOCK_MODEL_NAMES.find(station_name);
  if (it == BLOCK_MODEL_NAMES.end())
  {
    return;
  }

  std::vector<moveit_msgs::msg::CollisionObject> objects;

  for (size_t slot = 0; slot < it->second.size(); ++slot)
  {
    if (static_cast<int>(slot) == picked_slot)
    {
      continue;

    }

    if (picked_slots_.count({ station_name, static_cast<int>(slot) }) > 0)
    {
      continue;

    }

    const std::string& gz_block_name = it->second[slot];
    tf2::Transform block_pose;
    if (!lookupLiveStationPose(gz_block_name, block_pose))
    {
      RCLCPP_WARN(LOGGER,
                  "No live TF for sibling block '%s' -- skipping it as an "
                  "obstacle (the arm may not avoid it correctly this cycle).",
                  gz_block_name.c_str());
      continue;
    }

    geometry_msgs::msg::Pose p;
    tf2::toMsg(block_pose, p);

    moveit_msgs::msg::CollisionObject obj;
    obj.id              = "sibling_" + gz_block_name;
    obj.header.frame_id = "base_footprint";
    obj.primitives.resize(1);
    obj.primitives[0].type       = shape_msgs::msg::SolidPrimitive::BOX;
    obj.primitives[0].dimensions = { BLOCK_DIM_X, BLOCK_DIM_Y, BLOCK_DIM_Z };
    obj.pose = p;
    objects.push_back(obj);
  }

  for (const auto& obj : objects)
  {
    current_sibling_block_ids_.push_back(obj.id);
  }
  if (!objects.empty())
  {
    psi.applyCollisionObjects(objects);
  }

  RCLCPP_INFO(LOGGER, "Registered %zu sibling block(s) as obstacles for station '%s'",
              objects.size(), station_name.c_str());
}

rclcpp_action::GoalResponse MTCTaskNode::handleGoal(
    const rclcpp_action::GoalUUID& ,
    std::shared_ptr<const ArmCommand::Goal> goal)
{
  if (busy_.load())
  {
    RCLCPP_WARN(LOGGER, "Rejecting goal for '%s' -- a pick-and-place cycle is already running.",
                goal->object_id.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(LOGGER, "Accepted goal: pick '%s' at (%.2f,%.2f,%.2f) -> place at (%.2f,%.2f,%.2f)",
              goal->object_id.c_str(),
              goal->pick_x, goal->pick_y, goal->pick_z,
              goal->place_x, goal->place_y, goal->place_z);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MTCTaskNode::handleCancel(
    const std::shared_ptr<GoalHandleArmCommand> )
{

  RCLCPP_WARN(LOGGER, "Cancel requested -- current motion will still complete.");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MTCTaskNode::handleAccepted(const std::shared_ptr<GoalHandleArmCommand> goal_handle)
{

  std::thread{ std::bind(&MTCTaskNode::execute, this, std::placeholders::_1), goal_handle }.detach();
}

void MTCTaskNode::execute(const std::shared_ptr<GoalHandleArmCommand> goal_handle)
{
  busy_.store(true);

  const auto goal   = goal_handle->get_goal();
  auto      result  = std::make_shared<ArmCommand::Result>();
  auto      feedback = std::make_shared<ArmCommand::Feedback>();

  BlockConfig block;
  block.id         = goal->object_id.empty() ? std::string("goal_block") : goal->object_id;
  block.block_slot = goal->block_slot;
  block.pick_x     = goal->pick_x;
  block.pick_y     = goal->pick_y;
  block.pick_z     = goal->pick_z;
  block.place_x    = goal->place_x;
  block.place_y    = goal->place_y;
  block.place_z    = goal->place_z;

  feedback->status = "registering station geometry";
  goal_handle->publish_feedback(feedback);
  registerStationGeometry(goal->station);

  // Pick pose for shelf blocks is no longer overridden here -- it comes
  // straight from the goal (goal->pick_x/y/z), sourced from perception on
  // the executor side. Only the base->workbench reuse below still applies:
  // once a block has been placed somewhere by an earlier goal, reuse that
  // exact stored pose rather than trusting a static guess a second time.
  if (goal->block_slot >= 0)
  {
    auto stored_it = slot_last_pose_.find(goal->block_slot);
    if (stored_it != slot_last_pose_.end())
    {
      block.pick_x     = stored_it->second.x;
      block.pick_y     = stored_it->second.y;
      block.pick_z     = stored_it->second.z - 0.12;
      block.pick_frame = stored_it->second.frame;

      RCLCPP_INFO(LOGGER,
                  "Reused stored pose for '%s' slot %d (placed there by an "
                  "earlier goal, frame '%s'): (%.3f, %.3f, %.3f)",
                  block.id.c_str(), goal->block_slot, block.pick_frame.c_str(),
                  block.pick_x, block.pick_y, block.pick_z);
    }
  }

  auto station_tpl_it = STATION_TEMPLATES.find(goal->station);
  if (station_tpl_it != STATION_TEMPLATES.end() &&
      station_tpl_it->second.kind == StationTemplate::Kind::BOX &&
      goal->block_slot >= 0)
  {
    tf2::Transform station_pose;
    if (lookupLiveStationPose(goal->station, station_pose))
    {
      const StationTemplate& tpl = station_tpl_it->second;

      static constexpr double SLOT_SPACING = 0.12;
      const double slot_x = (goal->block_slot - 1) * SLOT_SPACING;
      const double slot_z = tpl.box_dim_z / 2.0 + BLOCK_DIM_Z / 2.0 + 0.08;

      tf2::Transform local(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(slot_x, -0.2, slot_z));
      tf2::Transform world = station_pose * local;
      geometry_msgs::msg::Pose p;
      tf2::toMsg(world, p);

      block.place_x     = p.position.x;
      block.place_y     = p.position.y;
      block.place_z     = p.position.z;
      block.place_frame = "base_footprint";

      RCLCPP_INFO(LOGGER,
                  "Overrode place pose for '%s' using live station TF '%s' "
                  "(slot %d): (%.3f, %.3f, %.3f)",
                  block.id.c_str(), goal->station.c_str(), goal->block_slot,
                  block.place_x, block.place_y, block.place_z);
    }
    else
    {
      RCLCPP_WARN(LOGGER,
                  "Station '%s' is a known box-type destination but no live "
                  "TF -- using the executor-supplied place pose instead.",
                  goal->station.c_str());
    }
  }

  feedback->status = "registering sibling blocks";
  goal_handle->publish_feedback(feedback);
  registerSiblingBlocks(goal->station, goal->block_slot);

  feedback->status = "registering block in planning scene";
  goal_handle->publish_feedback(feedback);
  registerBlockCollisionObject(block);

  feedback->status = "planning";
  goal_handle->publish_feedback(feedback);

  RCLCPP_INFO(LOGGER, "--- Planning pick-and-place for block '%s' ---", block.id.c_str());
  task_ = createTask(block);

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Init failed for block '" << block.id << "': " << e);
    result->success = false;
    goal_handle->abort(result);
    busy_.store(false);
    return;
  }

  if (goal_handle->is_canceling())
  {
    result->success = false;
    goal_handle->canceled(result);
    busy_.store(false);
    return;
  }

  if (!task_.plan(1))
  {
    RCLCPP_ERROR(LOGGER, "Planning failed for block '%s'", block.id.c_str());
    result->success = false;
    goal_handle->abort(result);
    busy_.store(false);
    return;
  }

  task_.introspection().publishSolution(*task_.solutions().front());

  feedback->status = "executing";
  goal_handle->publish_feedback(feedback);

  auto exec_result = task_.execute(*task_.solutions().front());
  if (exec_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR(LOGGER, "Execution failed for block '%s'", block.id.c_str());
    result->success = false;
    goal_handle->abort(result);
    busy_.store(false);
    return;
  }

  RCLCPP_INFO(LOGGER, "Block '%s' placed successfully.", block.id.c_str());
  feedback->status = "done";
  goal_handle->publish_feedback(feedback);

  if (goal->block_slot >= 0)
  {
    picked_slots_.insert({ goal->station, goal->block_slot });
    slot_last_pose_[goal->block_slot] = { block.place_x, block.place_y,
                                           block.place_z, block.place_frame };
  }

  result->success = true;
  goal_handle->succeed(result);
  busy_.store(false);
}

mtc::Task MTCTaskNode::createTask(const BlockConfig& block)
{
  mtc::Task task;
  task.stages()->setName("pick_place_" + block.id);
  task.loadRobotModel(node_);

  const auto& arm_group_name  = "arm";
  const auto& hand_group_name = "gripper";
  const auto& hand_frame      = "tcp";

  task.setProperty("group",    arm_group_name);
  task.setProperty("eef",      "end_effector");
  task.setProperty("ik_frame", hand_frame);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage* current_state_ptr   = nullptr;
  mtc::Stage* attach_object_stage = nullptr;
#pragma GCC diagnostic pop

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "ompl");
  sampling_planner->setPlannerId("TRRT");
  sampling_planner->setTimeout(20.0);  
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.3);
  cartesian_planner->setMaxAccelerationScalingFactor(0.3);
  cartesian_planner->setStepSize(0.01);

  {
    auto stage = std::make_unique<mtc::stages::CurrentState>("current");
    current_state_ptr = stage.get();
    task.add(std::move(stage));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
    stage->setGroup(hand_group_name);
    stage->setGoal("open");
    task.add(std::move(stage));
  }

  {
    auto stage = std::make_unique<mtc::stages::Connect>(
        "move to grasp",
        mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
    stage->setTimeout(10.0);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    task.add(std::move(stage));
  }

  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick " + block.id);

    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    grasp->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

    {
      auto gen = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp");
      gen->properties().configureInitFrom(mtc::Stage::PARENT);
      // gen->setEndEffector("end_effector");   // <-- make sure this line exists
      gen->setObject(block.id);
      gen->setPreGraspPose("open");
      gen->setAngleDelta(M_PI / 2);
      gen->setMonitoredStage(current_state_ptr);

      Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
      tf.translation().z() = 0.15;
      tf.linear() = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()).toRotationMatrix();

      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("compute ik", std::move(gen));
      wrapper->setIKFrame(tf, hand_frame);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setMaxIKSolutions(5);
      wrapper->setTimeout(10.0);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      grasp->insert(std::move(wrapper));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
          "allow collision (station," + block.id + ")");
      stage->allowCollisions(block.id, { "station_shelf_row_bottom", "station_box" }, true);
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("approach", cartesian_planner);
      stage->setIKFrame(hand_frame);
      stage->setMinMaxDistance(0.0, 0.15);
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "base_link";
      vec.vector.z = -1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
          "allow collision (hand," + block.id + ")");
      stage->allowCollisions(
          block.id,
          task.getRobotModel()
              ->getJointModelGroup(hand_group_name)
              ->getLinkModelNamesWithCollisionGeometry(),
          true);
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("close");
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach " + block.id);
      stage->attachObject(block.id, "tcp");
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("hold grip", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("close");
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lift " + block.id, cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.0, 0.20);
      stage->setIKFrame(hand_frame);
      stage->properties().set("link", hand_frame);

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    task.add(std::move(grasp));
  }

  {
    auto stage = std::make_unique<mtc::stages::Connect>(
        "move to place",
        mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
    stage->setTimeout(10.0);
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage));
  }

  {
    auto place = std::make_unique<mtc::SerialContainer>("place " + block.id);

    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
    place->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

    {
      auto gen = std::make_unique<mtc::stages::GeneratePlacePose>("generate place position");
      gen->properties().configureInitFrom(mtc::Stage::PARENT);
      gen->setObject(block.id);
      gen->setMonitoredStage(attach_object_stage);

      geometry_msgs::msg::PoseStamped p;
      p.header.frame_id    = block.place_frame;
      p.pose.position.x    = block.place_x;
      p.pose.position.y    = block.place_y;
      p.pose.position.z    = block.place_z;
      p.pose.orientation.w = 1.0;
      gen->setPose(p);

      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(gen));
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(0.5);
      wrapper->setTimeout(10.0);
      wrapper->setIKFrame(block.id);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      place->insert(std::move(wrapper));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lower to place", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.0, 0.06);
      stage->setIKFrame(hand_frame);
      stage->properties().set("link", hand_frame);

      geometry_msgs::msg::Vector3Stamped lower_vec;
      lower_vec.header.frame_id = "world";
      lower_vec.vector.z = -1.0;
      stage->setDirection(lower_vec);
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("open");
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("disable collisions");
      stage->allowCollisions(
          block.id,
          task.getRobotModel()
              ->getJointModelGroup(hand_group_name)
              ->getLinkModelNamesWithCollisionGeometry(),
          false);
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach " + block.id);
      stage->detachObject(block.id, "tcp");
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("move up", cartesian_planner);
      stage->setIKFrame(hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.00, 0.30);
      stage->setIKFrame(hand_frame);

      geometry_msgs::msg::Vector3Stamped v;
      v.header.frame_id = "base_link";
      v.vector.z = 1.0;
      stage->setDirection(v);
      place->insert(std::move(stage));
    }

    task.add(std::move(place));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", sampling_planner);
    stage->setGroup(arm_group_name);
    stage->setGoal("home");
    task.add(std::move(stage));
  }

  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  mtc_task_node->setupPlanningScene();

  executor.add_node(mtc_task_node->getNodeBaseInterface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}