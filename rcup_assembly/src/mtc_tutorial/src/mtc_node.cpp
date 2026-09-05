#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include "link_attatcher/srv/attach.hpp"
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
#include <std_msgs/msg/empty.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include <unordered_map>
#include <vector>
#include <string>
#include <algorithm>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask(const std::string& object_id,double place_x,double place_y,double place_z,const std::vector<std::string> & addingcollisions);

  // Builds the planning-scene collision objects for the currently selected object_name, using assembly_recipes.yaml.
  void setupPlanningScene();

  // Physically welds two Gazebo models together via the link_attacher plugin.
  void publishattatch(const std::string& model1_name,const std::string& link1_name,const std::string& model2_name,const std::string& link2_name);

  // Runs the full pick/place/weld/attach sequence for the currently selected object_name.
  void runAssembly();

private:
  mtc::Task createTask(const std::string& object_id,double place_x,double place_y,double place_z,const std::vector<std::string> & addingcollisions);

  // Loads (and caches) assembly_recipes.yaml.
  const YAML::Node& assemblyConfig();

  // Groups 'a' and 'b' together (union-find)
  void attachTogether(const std::string& a, const std::string& b);

  // Union-find lookup: returns the current group representative for 'id'.
  std::string groupOf(const std::string& id);

  // After 'moved_id' has just been picked and placed (old_pose -> new_pose), applies the
  // same rigid-body transform to every OTHER object currently in its group, keeping each
  // one's own independent shape/id - this is what keeps welded-together pieces visually
  // attached in RViz/the planning scene instead of one of them staying frozen behind.
  void propagateGroupMotion(const std::string& moved_id,
                             const geometry_msgs::msg::Pose& old_pose,
                             const geometry_msgs::msg::Pose& new_pose);

  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
  YAML::Node assembly_config_;
  bool assembly_config_loaded_ = false;
  std::unordered_map<std::string, std::string> group_parent_;
};

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
  if (!node_->has_parameter("object_name"))
  {
    node_->declare_parameter<std::string>("object_name", "burger");
  }
  if (!node_->has_parameter("assembly_config_path"))
  {
    node_->declare_parameter<std::string>("assembly_config_path", "");
  }
}

const YAML::Node& MTCTaskNode::assemblyConfig()
{
  if (!assembly_config_loaded_)
  {
    std::string path;
    node_->get_parameter("assembly_config_path", path);
    if (path.empty())
    {
      path = ament_index_cpp::get_package_share_directory("piper_moveit_config") +
             "/config/assembly_recipes.yaml";
    }
    RCLCPP_INFO(node_->get_logger(), "Loading assembly recipes from '%s'", path.c_str());
    assembly_config_ = YAML::LoadFile(path);
    assembly_config_loaded_ = true;
  }
  return assembly_config_;
}

std::string MTCTaskNode::groupOf(const std::string& id)
{
  auto it = group_parent_.find(id);
  if (it == group_parent_.end())
  {
    group_parent_[id] = id;
    return id;
  }
  if (it->second != id)
  {
    it->second = groupOf(it->second);  // path compression
  }
  return it->second;
}

void MTCTaskNode::attachTogether(const std::string& a, const std::string& b)
{
  const std::string ra = groupOf(a);
  const std::string rb = groupOf(b);
  if (ra != rb)
  {
    group_parent_[rb] = ra;
  }
}

void MTCTaskNode::propagateGroupMotion(const std::string& moved_id,
                                        const geometry_msgs::msg::Pose& old_pose,
                                        const geometry_msgs::msg::Pose& new_pose)
{
  const std::string root = groupOf(moved_id);

  std::vector<std::string> members;
  // Collect every id we've seen so far that belongs to the same group as moved_id,
  // excluding moved_id itself (its own pose was already updated by the place stage).
  for (const auto& kv : group_parent_)
  {
    if (kv.first != moved_id && groupOf(kv.first) == root)
    {
      members.push_back(kv.first);
    }
  }
  if (members.empty())
  {
    return;
  }

  Eigen::Isometry3d old_tf, new_tf;
  tf2::fromMsg(old_pose, old_tf);
  tf2::fromMsg(new_pose, new_tf);
  const Eigen::Isometry3d delta = new_tf * old_tf.inverse();

  moveit::planning_interface::PlanningSceneInterface psi;
  auto objs = psi.getObjects(members);

  std::vector<moveit_msgs::msg::CollisionObject> updates;
  for (const auto& id : members)
  {
    auto it = objs.find(id);
    if (it == objs.end())
    {
      continue;  // not yet in the scene (e.g. never staged) - nothing to move
    }
    moveit_msgs::msg::CollisionObject obj = it->second;

    Eigen::Isometry3d cur_tf;
    tf2::fromMsg(obj.pose, cur_tf);
    const Eigen::Isometry3d moved_tf = delta * cur_tf;

    obj.pose = tf2::toMsg(moved_tf);
    obj.operation = moveit_msgs::msg::CollisionObject::MOVE;
    updates.push_back(obj);
  }

  if (!updates.empty())
  {
    psi.applyCollisionObjects(updates);
    RCLCPP_INFO(node_->get_logger(), "Dragged %zu attached piece(s) along with '%s'",
                updates.size(), moved_id.c_str());
  }
}

void MTCTaskNode::setupPlanningScene()
{
  std::string object_name;
  node_->get_parameter("object_name", object_name);

  const YAML::Node& config = assemblyConfig();
  YAML::Node objects_node = config["objects"];
  if (!objects_node[object_name])
  {
    RCLCPP_ERROR(node_->get_logger(),
                 "No assembly recipe found for object_name='%s' in assembly_recipes.yaml. "
                 "Check the 'objects' map in that file for the available names.",
                 object_name.c_str());
    return;
  }

  YAML::Node blocks = objects_node[object_name]["blocks"];
  std::vector<moveit_msgs::msg::CollisionObject> objects;

  for (const auto& entry : blocks)
  {
    const std::string id = entry.first.as<std::string>();
    const YAML::Node& b = entry.second;
    const auto dims = b["dims"].as<std::vector<double>>();

    moveit_msgs::msg::CollisionObject object;
    object.id = id;
    object.header.frame_id = "base_link";
    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    object.primitives[0].dimensions = { dims.at(0), dims.at(1), dims.at(2) };

    geometry_msgs::msg::Pose pose;
    pose.position.x = b["x"].as<double>();
    pose.position.y = b["y"].as<double>();
    pose.position.z = b["z"].as<double>();
    pose.orientation.w = 1.0;
    object.pose = pose;

    objects.push_back(object);
  }

  RCLCPP_INFO(node_->get_logger(), "Staging %zu block(s) in the planning scene for object_name='%s'",
              objects.size(), object_name.c_str());

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObjects(objects);
}

void MTCTaskNode::publishattatch(const std::string& model1_name, const std::string& link1_name,
                                  const std::string& model2_name, const std::string& link2_name)
{
  rclcpp::Client<link_attatcher::srv::Attach>::SharedPtr client =
      node_->create_client<link_attatcher::srv::Attach>("/link_attacher/attach");

  if (!client->wait_for_service(std::chrono::seconds(2)))
  {
    RCLCPP_ERROR(node_->get_logger(), "Attach service not available");
    return;
  }

  auto request = std::make_shared<link_attatcher::srv::Attach::Request>();
  request->model1_name = model1_name;
  request->model2_name = model2_name;
  request->link1_name  = link1_name;
  request->link2_name  = link2_name;

  auto future = client->async_send_request(request);
  auto status = future.wait_for(std::chrono::seconds(5));

  if (status == std::future_status::ready)
  {
    auto result = future.get();
    RCLCPP_INFO(
      node_->get_logger(),
      "Attach response: success=%s, message=%s",
      result->success ? "true" : "false",
      result->message.c_str());
  }
  else
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to call attach service (timeout)");
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

void MTCTaskNode::doTask(const std::string& object_id,double place_x,double place_y,
    double place_z,const std::vector<std::string> & addingcollisions)
{
  task_ = createTask(object_id,place_x,place_y,place_z,addingcollisions);


  try
  {
    task_.init();
  }

  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }
  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }

  return;
}




mtc::Task MTCTaskNode::createTask(const std::string& object_id,double place_x,double place_y,double place_z, const std::vector<std::string> & addingcollisions){
  mtc::Task task;
  task.stages()->setName("demo task");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "manipulator";
  const auto& hand_group_name = "gripper";
  const auto& hand_frame = "tcp";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", "eff");
  task.setProperty("ik_frame", hand_frame);

// Disable warnings for this line, as it's a variable that's set but not used in this example
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
#pragma GCC diagnostic pop

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.6);
  cartesian_planner->setMaxAccelerationScalingFactor(0.6);
  cartesian_planner->setStepSize(.01);

  auto stage_open_hand =
      std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open");
  task.add(std::move(stage_open_hand));
  
  {
    auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>("move to pick",mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
    stage_move_to_pick->setTimeout(5.0);
    stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_pick));
  }

  mtc::Stage* attach_object_stage =
    nullptr;  // Forward attach_object_stage to place pose generator


  {

  auto grasp = std::make_unique<mtc::SerialContainer>("Pick Object");
  task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
  grasp->properties().configureInitFrom(mtc::Stage::PARENT,{ "eef", "group", "ik_frame" });
  {

    auto stage=std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    stage->properties().set("marker_ns", "grasp_pose");
    stage->setObject(object_id);
    stage->setPreGraspPose("open");
    stage->setAngleDelta(M_PI/2);
    stage->setMonitoredStage(current_state_ptr);

    Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
    grasp_frame_transform.translation().z() = 0.05;
    grasp_frame_transform.linear()=Eigen::AngleAxisd(M_PI,Eigen::Vector3d::UnitX()).toRotationMatrix();

  auto wrapper =
      std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
  wrapper->setMaxIKSolutions(8);
  wrapper->setMinSolutionDistance(1.0);
  wrapper->setIKFrame(grasp_frame_transform, hand_frame);
  wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
  wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
  grasp->insert(std::move(wrapper));



  }

  {
    auto stage=std::make_unique<mtc::stages::ModifyPlanningScene>("allow collisions");
    stage->allowCollisions(object_id,task.getRobotModel()->getJointModelGroup(hand_group_name)->getLinkModelNamesWithCollisionGeometry(),true);
    grasp->insert(std::move(stage));

  }

  {
  auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
  stage->setGroup(hand_group_name);
  stage->setGoal("closed");
  grasp->insert(std::move(stage));
}

  {
  auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
  stage->attachObject(object_id, hand_frame);
  attach_object_stage = stage.get();
  grasp->insert(std::move(stage));
  }
  

  {
  auto stage =
      std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
  stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage->setMinMaxDistance(0.05, 0.3);
  stage->setIKFrame(hand_frame);
  stage->properties().set("markerpublishatt_ns", "lift_object");

  geometry_msgs::msg::Vector3Stamped vec;
  vec.header.frame_id = "base_link";
  vec.vector.z = 1.0;
  stage->setDirection(vec);
  grasp->insert(std::move(stage));
}
  task.add(std::move(grasp));
  }

  {
  auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
      "move to place",
      mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner }
                                                 });
  stage_move_to_place->setTimeout(5.0);
  stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_place));
}
{
  auto place = std::make_unique<mtc::SerialContainer>("place object");
  task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
  place->properties().configureInitFrom(mtc::Stage::PARENT,
                                        { "eef", "group", "ik_frame" });


  {
    auto stage=std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision between objects");
    for (const auto& other : addingcollisions )
    {
      stage->allowCollisions(object_id,other,true);

    } 
    place->insert(std::move(stage));
  }

  


  {
  auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
  stage->properties().configureInitFrom(mtc::Stage::PARENT);
  stage->properties().set("marker_ns", "place_pose");
  stage->setObject(object_id);

  geometry_msgs::msg::PoseStamped target_pose_msg;
  target_pose_msg.header.frame_id = "base_link";
  target_pose_msg.pose.position.x=place_x;
  target_pose_msg.pose.position.y = place_y;
  target_pose_msg.pose.position.z = place_z; 
  target_pose_msg.pose.orientation.w = 1.0;
  stage->setPose(target_pose_msg);
  stage->setMonitoredStage(attach_object_stage); 

  auto wrapper =
      std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
  wrapper->setMaxIKSolutions(8);
  wrapper->setMinSolutionDistance(1.0);
  wrapper->setIKFrame(object_id);
  wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
  wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
  place->insert(std::move(wrapper));
}
{
  auto stage=std::make_unique<mtc::stages::MoveRelative>("Approach down",cartesian_planner);
  stage->properties().configureInitFrom(mtc::Stage::PARENT,  { "group" });
  stage->setMinMaxDistance(0.00,0.002);
  
  geometry_msgs::msg::Vector3Stamped vec;
  vec.header.frame_id = "base_link";
  vec.vector.z = -1.0;
  stage->setDirection(vec);
  place->insert(std::move(stage));
  
}


{
  auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage->setGroup(hand_group_name);
  stage->setGoal("open");
  place->insert(std::move(stage));
}

{
  auto stage =
      std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
  stage->allowCollisions(object_id,
                        task.getRobotModel()
                            ->getJointModelGroup(hand_group_name)
                            ->getLinkModelNamesWithCollisionGeometry(),
                        false);
  place->insert(std::move(stage));
}

{
  auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
  stage->detachObject(object_id, hand_frame);
  place->insert(std::move(stage));
}
{
  auto stage=std::make_unique<mtc::stages::ModifyPlanningScene>("disallow collisions");
  for (const auto & other : addingcollisions )
  {
    stage->allowCollisions(other,object_id,false);
    stage->allowCollisions(other,
      task.getRobotModel()->getJointModelGroup(hand_group_name)->getLinkModelNamesWithCollisionGeometry(),
      true);
  }
  place->insert(std::move(stage));
}

{
  auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
  stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage->setMinMaxDistance(0.01, 0.3);
  stage->setIKFrame(hand_frame);
  stage->properties().set("marker_ns", "retreat");

  // Set retreat direction
  geometry_msgs::msg::Vector3Stamped vec;
  vec.header.frame_id = "base_link";
  vec.vector.z = 0.1;
  stage->setDirection(vec);
  place->insert(std::move(stage));
}

  task.add(std::move(place));
}

{
  auto stage = std::make_unique<mtc::stages::MoveTo>("return home", sampling_planner);
  stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage->setGoal("forward");
  task.add(std::move(stage));
}
{
  auto stage_open_hand =
      std::make_unique<mtc::stages::MoveTo>("close", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("closed");
  task.add(std::move(stage_open_hand));
}
  return task;
}

void MTCTaskNode::runAssembly()
{
  std::string object_name;
  node_->get_parameter("object_name", object_name);

  const YAML::Node& config = assemblyConfig();
  YAML::Node objects_node = config["objects"];
  if (!objects_node[object_name])
  {
    RCLCPP_ERROR(node_->get_logger(),
                 "No assembly recipe found for object_name='%s', nothing to run.",
                 object_name.c_str());
    return;
  }

  group_parent_.clear();
  YAML::Node steps = objects_node[object_name]["steps"];

  RCLCPP_INFO(node_->get_logger(), "Running assembly '%s' (%zu step(s))",
              object_name.c_str(), steps.size());

  moveit::planning_interface::PlanningSceneInterface psi;

  for (const auto& step : steps)
  {
    const std::string type = step["type"].as<std::string>();

    if (type == "pick_place")
    {
      const std::string object_id = step["object_id"].as<std::string>();
      const auto place = step["place"].as<std::vector<double>>();

      std::vector<std::string> collisions;
      for (const auto& c : step["allow_collision_with"])
      {
        const std::string other = c.as<std::string>();
        if (other != object_id &&
            std::find(collisions.begin(), collisions.end(), other) == collisions.end())
        {
          collisions.push_back(other);
        }
      }

      // Remember where this piece was before moving it, so any pieces already welded to
      // it (from an earlier 'attach' step) can be dragged along by the same transform.
      geometry_msgs::msg::Pose old_pose;
      bool had_old_pose = false;
      {
        auto before = psi.getObjects({ object_id });
        auto it = before.find(object_id);
        if (it != before.end())
        {
          old_pose = it->second.pose;
          had_old_pose = true;
        }
      }

      doTask(object_id, place.at(0), place.at(1), place.at(2), collisions);

      if (had_old_pose)
      {
        auto after = psi.getObjects({ object_id });
        auto it = after.find(object_id);
        if (it != after.end())
        {
          propagateGroupMotion(object_id, old_pose, it->second.pose);
        }
      }
    }
    else if (type == "weld")
    {
      publishattatch(step["model1"].as<std::string>(), step["link1"].as<std::string>(),
                      step["model2"].as<std::string>(), step["link2"].as<std::string>());
    }
    else if (type == "attach")
    {
      attachTogether(step["into"].as<std::string>(), step["from"].as<std::string>());
    }
    else
    {
      RCLCPP_WARN(node_->get_logger(), "Unknown assembly step type '%s', skipping", type.c_str());
    }
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  mtc_task_node->setupPlanningScene();
  mtc_task_node->runAssembly();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}