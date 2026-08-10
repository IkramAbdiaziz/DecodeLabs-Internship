#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// TRAC-IK & KDL includes
#include <trac_ik/trac_ik.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>

// MoveIt 2 Core includes (built on top of FCL)
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>

#include <mutex>
#include <vector>
#include <string>
#include <cmath>
#include <memory>
#include <algorithm>

class KinematicsPlannerNode : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  KinematicsPlannerNode()
  : Node("kinematics_planner_node"),
    has_joint_state_(false),
    is_moving_(false)
  {
    // Declare parameters
    this->declare_parameter<std::string>("base_link", "base_link");
    this->declare_parameter<std::string>("end_effector", "end_effector");
    this->declare_parameter<double>("solve_timeout", 0.01); // 10ms for IK
    this->declare_parameter<double>("solve_epsilon", 1e-5);
    this->declare_parameter<double>("trajectory_frequency", 20.0); // Hz of splines (dt = 0.05s)
    this->declare_parameter<std::string>("robot_description", "");
    this->declare_parameter<std::string>("robot_description_semantic", "");

    // Retrieve parameters
    this->get_parameter("base_link", base_link_);
    this->get_parameter("end_effector", end_effector_);
    this->get_parameter("solve_timeout", solve_timeout_);
    this->get_parameter("solve_epsilon", solve_epsilon_);
    this->get_parameter("trajectory_frequency", trajectory_frequency_);

    RCLCPP_INFO(this->get_logger(), "Initializing Kinematics Planner Node (Base: %s, End-Effector: %s)",
      base_link_.c_str(), end_effector_.c_str());

    // TF2 Setup
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Subscriptions
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&KinematicsPlannerNode::joint_state_callback, this, std::placeholders::_1));

    target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/target_pose", 10,
      std::bind(&KinematicsPlannerNode::target_pose_callback, this, std::placeholders::_1));

    collision_object_sub_ = this->create_subscription<moveit_msgs::msg::CollisionObject>(
      "/collision_object", 10,
      std::bind(&KinematicsPlannerNode::collision_object_callback, this, std::placeholders::_1));

    // Action Client
    action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, "arm_controller/follow_joint_trajectory");

    // Timer for environment safety checking (runs at 20 Hz)
    safety_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&KinematicsPlannerNode::safety_monitor_callback, this));

    // Initialize MoveIt Model & Planning Scene (FCL)
    // We delay the actual initialization of TRAC-IK and MoveIt until the first /robot_description is available
    model_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&KinematicsPlannerNode::initialize_kinematics_and_scene, this));
  }

private:
  void initialize_kinematics_and_scene()
  {
    if (tracik_solver_) {
      model_timer_->cancel();
      return;
    }

    std::string urdf_xml;
    if (!this->has_parameter("robot_description")) {
      // If parameter not defined yet, wait
      return;
    }

    this->get_parameter("robot_description", urdf_xml);
    if (urdf_xml.empty()) {
      RCLCPP_WARN_ONCE(this->get_logger(), "robot_description parameter is empty. Waiting...");
      return;
    }

    try {
      // 1. Initialize TRAC-IK solver
      // In ROS 2, the TRAC-IK constructor takes the node pointer, base link, tip link, URDF parameter name, timeout, epsilon, and type.
      tracik_solver_ = std::make_unique<TRAC_IK::TRAC_IK>(
        shared_from_this(),
        base_link_,
        end_effector_,
        "robot_description",
        solve_timeout_,
        solve_epsilon_,
        TRAC_IK::Distance
      );

      // Verify solver configuration and extract joint names from KDL chain
      if (!tracik_solver_->getKDLChain(kdl_chain_)) {
        RCLCPP_ERROR(this->get_logger(), "TRAC-IK could not construct KDL chain from description.");
        tracik_solver_.reset();
        return;
      }
      
      num_joints_ = 0;
      joint_names_.clear();
      for (size_t i = 0; i < kdl_chain_.getNrOfSegments(); ++i) {
        auto joint = kdl_chain_.getSegment(i).getJoint();
        if (joint.getType() != KDL::Joint::None) {
          joint_names_.push_back(joint.getName());
          num_joints_++;
        }
      }

      RCLCPP_INFO(this->get_logger(), "TRAC-IK Solver initialized successfully with %u joints:", num_joints_);
      for (size_t i = 0; i < num_joints_; ++i) {
        RCLCPP_INFO(this->get_logger(), "  - Joint %zu: %s", i + 1, joint_names_[i].c_str());
      }

      // 2. Initialize MoveIt RobotModel Loader & FCL Planning Scene
      robot_model_loader::RobotModelLoader::Options opt;
      opt.robot_description_ = "robot_description";
      robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(
        shared_from_this(), opt);

      robot_model_ptr_ = robot_model_loader_->getModel();
      if (!robot_model_ptr_) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load RobotModel from description.");
        tracik_solver_.reset();
        return;
      }

      planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_ptr_);
      RCLCPP_INFO(this->get_logger(), "MoveIt Planning Scene (FCL) initialized successfully.");

      model_timer_->cancel();
    }
    catch (const std::exception &ex) {
      RCLCPP_ERROR(this->get_logger(), "Exception initializing solver: %s", ex.what());
    }
  }

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(joint_mutex_);
    
    // Extract joints in order of the KDL chain
    if (joint_names_.empty()) {
      return; // Solver not initialized yet
    }

    current_joints_.resize(num_joints_, 0.0);
    size_t found_count = 0;
    
    for (size_t i = 0; i < num_joints_; ++i) {
      auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
      if (it != msg->name.end()) {
        size_t idx = std::distance(msg->name.begin(), it);
        current_joints_[i] = msg->position[idx];
        found_count++;
      }
    }

    if (found_count == num_joints_) {
      has_joint_state_ = true;
    }
  }

  void collision_object_callback(const moveit_msgs::msg::CollisionObject::SharedPtr msg)
  {
    if (!planning_scene_) {
      return;
    }
    std::lock_guard<std::mutex> lock(scene_mutex_);
    planning_scene_->processCollisionObjectMsg(*msg);
    RCLCPP_INFO(this->get_logger(), "Processed collision object: %s (Action: %d)", msg->id.c_str(), msg->operation);
  }

  void target_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // Guard initialization
    if (!tracik_solver_ || !planning_scene_) {
      RCLCPP_ERROR(this->get_logger(), "Planner nodes (TRAC-IK / Planning Scene) not initialized yet.");
      return;
    }

    // Interdependent Trajectory Resource Lock
    if (is_moving_) {
      RCLCPP_WARN(this->get_logger(), "Resource Locked: Arm is currently executing a path. Target pose command rejected.");
      return;
    }

    // Get Current Joint Configuration
    std::vector<double> start_joints;
    {
      std::lock_guard<std::mutex> lock(joint_mutex_);
      if (!has_joint_state_) {
        RCLCPP_ERROR(this->get_logger(), "No joint state received yet. Cannot plan.");
        return;
      }
      start_joints = current_joints_;
    }

    RCLCPP_INFO(this->get_logger(), "Planning request received for target frame: %s", msg->header.frame_id.c_str());

    // 1. Transform Target Pose to Base Link Frame (if needed)
    geometry_msgs::msg::PoseStamped target_pose_base;
    if (msg->header.frame_id != base_link_) {
      try {
        // Look up transform with temporal buffer safeguarding
        geometry_msgs::msg::TransformStamped tf_stamped = tf_buffer_->lookupTransform(
          base_link_, msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.2));
        tf2::doTransform(*msg, target_pose_base, tf_stamped);
      } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "TF transformation to base_link failed: %s. Using default frame.", ex.what());
        target_pose_base = *msg;
      }
    } else {
      target_pose_base = *msg;
    }

    // 2. Solve Inverse Kinematics via TRAC-IK
    KDL::JntArray nominal_joints(num_joints_);
    for (size_t i = 0; i < num_joints_; ++i) {
      nominal_joints(i) = start_joints[i];
    }

    KDL::Frame target_frame;
    auto pos = target_pose_base.pose.position;
    auto ori = target_pose_base.pose.orientation;
    target_frame.p = KDL::Vector(pos.x, pos.y, pos.z);
    target_frame.M = KDL::Rotation::Quaternion(ori.x, ori.y, ori.z, ori.w);

    KDL::JntArray solution_joints(num_joints_);
    int ik_status = tracik_solver_->CartToJnt(nominal_joints, target_frame, solution_joints);

    if (ik_status < 0) {
      RCLCPP_ERROR(this->get_logger(), "TRAC-IK Solver failed to find solution (SOLUTION_NOT_FOUND). Status: %d", ik_status);
      return;
    }

    std::vector<double> goal_joints(num_joints_);
    RCLCPP_INFO(this->get_logger(), "TRAC-IK Solver successfully found joint coordinates:");
    for (size_t i = 0; i < num_joints_; ++i) {
      goal_joints[i] = solution_joints(i);
      RCLCPP_INFO(this->get_logger(), "  - %s: %f rad", joint_names_[i].c_str(), goal_joints[i]);
    }

    // 3. Check Start and Goal Collisions in Planning Scene (FCL)
    {
      std::lock_guard<std::mutex> lock(scene_mutex_);
      
      // Start configuration collision check
      moveit::core::RobotState start_state(robot_model_ptr_);
      for (size_t i = 0; i < num_joints_; ++i) {
        start_state.setJointPositions(joint_names_[i], &start_joints[i]);
      }
      start_state.update();

      collision_detection::CollisionRequest col_req;
      collision_detection::CollisionResult col_res_start;
      planning_scene_->checkCollision(col_req, col_res_start, start_state);
      if (col_res_start.collision) {
        RCLCPP_ERROR(this->get_logger(), "Planning aborted: Start state is in collision!");
        return;
      }

      // Goal configuration collision check
      moveit::core::RobotState goal_state(robot_model_ptr_);
      for (size_t i = 0; i < num_joints_; ++i) {
        goal_state.setJointPositions(joint_names_[i], &goal_joints[i]);
      }
      goal_state.update();

      collision_detection::CollisionResult col_res_goal;
      planning_scene_->checkCollision(col_req, col_res_goal, goal_state);
      if (col_res_goal.collision) {
        RCLCPP_ERROR(this->get_logger(), "Planning aborted: Goal state is in collision!");
        return;
      }
    }

    // 4. Generate Continuous Quintic Spline Joint Trajectory
    // Calculate total time duration based on joint limit constraints (max joint speed limit)
    double max_joint_delta = 0.0;
    for (size_t i = 0; i < num_joints_; ++i) {
      double diff = std::abs(goal_joints[i] - start_joints[i]);
      if (diff > max_joint_delta) {
        max_joint_delta = diff;
      }
    }
    
    // Assume safe average speed of 0.4 rad/s to bound acceleration/velocity profiles
    double duration = std::max(2.0, max_joint_delta / 0.4);
    
    // Formulate waypoints
    double dt = 1.0 / trajectory_frequency_;
    size_t num_points = std::ceil(duration / dt);
    
    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.joint_names = joint_names_;
    trajectory.points.reserve(num_points);

    std::vector<std::vector<double>> computed_points_joints; // To keep for collision verification

    RCLCPP_INFO(this->get_logger(), "Generating quintic spline path with %zu points over %f seconds", num_points, duration);

    for (size_t step = 0; step <= num_points; ++step) {
      double t = step * dt;
      if (t > duration) t = duration;

      double s, ds, dds;
      compute_quintic_scaling(t, duration, s, ds, dds);

      trajectory_msgs::msg::JointTrajectoryPoint pt;
      std::vector<double> step_joints(num_joints_);
      for (size_t i = 0; i < num_joints_; ++i) {
        double qi = start_joints[i] + s * (goal_joints[i] - start_joints[i]);
        double dqi = ds * (goal_joints[i] - start_joints[i]);
        double ddqi = dds * (goal_joints[i] - start_joints[i]);

        pt.positions.push_back(qi);
        pt.velocities.push_back(dqi);
        pt.accelerations.push_back(ddqi);
        step_joints[i] = qi;
      }
      pt.time_from_start = rclcpp::Duration::from_seconds(t);
      trajectory.points.push_back(pt);
      computed_points_joints.push_back(step_joints);
    }

    // 5. Pre-verify Entire Trajectory for Collisions
    {
      std::lock_guard<std::mutex> lock(scene_mutex_);
      moveit::core::RobotState check_state(robot_model_ptr_);
      collision_detection::CollisionRequest col_req;
      collision_detection::CollisionResult col_res;

      for (size_t step = 0; step < computed_points_joints.size(); ++step) {
        for (size_t i = 0; i < num_joints_; ++i) {
          check_state.setJointPositions(joint_names_[i], &computed_points_joints[step][i]);
        }
        check_state.update();
        col_res.clear();
        planning_scene_->checkCollision(col_req, col_res, check_state);
        
        if (col_res.collision) {
          RCLCPP_ERROR(this->get_logger(), "Trajectory generation aborted: collision detected at step %zu!", step);
          return;
        }
      }
    }

    // 6. Dispatch Action Request
    if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available! Aborting execution.");
      return;
    }

    auto goal_msg = FollowJointTrajectory::Goal();
    goal_msg.trajectory = trajectory;

    auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&KinematicsPlannerNode::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
      std::bind(&KinematicsPlannerNode::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
      std::bind(&KinematicsPlannerNode::result_callback, this, std::placeholders::_1);

    // Save active trajectory data for runtime collision checking
    {
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      active_trajectory_ = trajectory;
      trajectory_start_time_ = this->now();
      trajectory_duration_ = duration;
      is_moving_ = true;
    }

    RCLCPP_INFO(this->get_logger(), "Dispatching FollowJointTrajectory action goal.");
    action_client_->async_send_goal(goal_msg, send_goal_options);
  }

  // Quintic Scaling Parameterization
  void compute_quintic_scaling(double t, double T, double &s, double &ds, double &dds)
  {
    double t_ratio = t / T;
    double t_ratio2 = t_ratio * t_ratio;
    double t_ratio3 = t_ratio2 * t_ratio;
    double t_ratio4 = t_ratio3 * t_ratio;
    double t_ratio5 = t_ratio4 * t_ratio;

    s = 10.0 * t_ratio3 - 15.0 * t_ratio4 + 6.0 * t_ratio5;
    ds = (1.0 / T) * (30.0 * t_ratio2 - 60.0 * t_ratio3 + 30.0 * t_ratio4);
    dds = (1.0 / (T * T)) * (60.0 * t_ratio - 180.0 * t_ratio2 + 120.0 * t_ratio3);
  }

  // Runtime Safety Monitor: Checks future trajectory waypoints against obstacles at 20 Hz
  void safety_monitor_callback()
  {
    if (!is_moving_ || !planning_scene_) {
      return;
    }

    std::lock_guard<std::mutex> lock_traj(trajectory_mutex_);
    std::lock_guard<std::mutex> lock_scene(scene_mutex_);

    rclcpp::Time now = this->now();
    double elapsed = (now - trajectory_start_time_).seconds();

    if (elapsed >= trajectory_duration_) {
      return; // Finished executing
    }

    // Determine current step index in the active trajectory
    double dt = 1.0 / trajectory_frequency_;
    size_t current_step = std::floor(elapsed / dt);

    moveit::core::RobotState check_state(robot_model_ptr_);
    collision_detection::CollisionRequest col_req;
    collision_detection::CollisionResult col_res;

    // Check all future points for potential collision (dynamic obstacle appearance)
    for (size_t step = current_step; step < active_trajectory_.points.size(); ++step) {
      std::vector<double> joint_positions = active_trajectory_.points[step].positions;
      for (size_t i = 0; i < num_joints_; ++i) {
        check_state.setJointPositions(joint_names_[i], &joint_positions[i]);
      }
      check_state.update();
      col_res.clear();
      planning_scene_->checkCollision(col_req, col_res, check_state);

      if (col_res.collision) {
        RCLCPP_WARN(this->get_logger(), "DYNAMIC INTERCEPTION: Collision detected along remaining trajectory! ABORTING MOTION!");
        
        // Cancel the active goal asynchronously
        if (active_goal_handle_) {
          action_client_->async_cancel_goal(active_goal_handle_);
        }
        is_moving_ = false;
        break;
      }
    }
  }

  // Action Goal Response Callback
  void goal_response_callback(const GoalHandleFollowJointTrajectory::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Action Goal was rejected by the controller server.");
      is_moving_ = false;
    } else {
      RCLCPP_INFO(this->get_logger(), "Action Goal accepted by the controller server. Starting motion execution.");
      active_goal_handle_ = goal_handle;
    }
  }

  // Action Feedback Callback
  void feedback_callback(
    GoalHandleFollowJointTrajectory::SharedPtr,
    const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback)
  {
    // Joint trajectory feedback monitoring (can be used to track progress or debug lag)
    (void)feedback;
  }

  // Action Result Callback
  void result_callback(const GoalHandleFollowJointTrajectory::WrappedResult & result)
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    is_moving_ = false;
    active_goal_handle_.reset();

    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Trajectory execution SUCCEEDED! Destination target state achieved.");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Trajectory execution was ABORTED by controller server!");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(this->get_logger(), "Trajectory execution was CANCELED.");
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Trajectory execution ended with unknown result code.");
        break;
    }
  }

  // Member Variables
  std::string base_link_;
  std::string end_effector_;
  double solve_timeout_;
  double solve_epsilon_;
  double trajectory_frequency_;

  // TF2
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
  rclcpp::Subscription<moveit_msgs::msg::CollisionObject>::SharedPtr collision_object_sub_;

  // Action Client
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
  GoalHandleFollowJointTrajectory::SharedPtr active_goal_handle_;

  // Timers
  rclcpp::TimerBase::SharedPtr safety_timer_;
  rclcpp::TimerBase::SharedPtr model_timer_;

  // TRAC-IK objects
  std::unique_ptr<TRAC_IK::TRAC_IK> tracik_solver_;
  KDL::Chain kdl_chain_;
  unsigned int num_joints_;
  std::vector<std::string> joint_names_;
  KDL::JntArray nominal_joints_;
  KDL::ChainFkSolverPos_recursive* kdl_solver_;

  // MoveIt FCL Scene Objects
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  moveit::core::RobotModelPtr robot_model_ptr_;
  planning_scene::PlanningScenePtr planning_scene_;

  // Mutexes & State
  std::mutex joint_mutex_;
  std::mutex scene_mutex_;
  std::mutex trajectory_mutex_;

  std::vector<double> current_joints_;
  bool has_joint_state_;
  bool is_moving_;

  // Active Trajectory Tracking for Dynamic Collision Monitoring
  trajectory_msgs::msg::JointTrajectory active_trajectory_;
  rclcpp::Time trajectory_start_time_;
  double trajectory_duration_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KinematicsPlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
