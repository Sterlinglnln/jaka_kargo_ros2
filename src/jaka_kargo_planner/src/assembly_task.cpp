// SDK-free MoveIt assembly client. All robot motion uses plan + execute.
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_state_validity.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <moveit/collision_detection/collision_matrix.h>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <Eigen/Geometry>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;
using Group = moveit::planning_interface::MoveGroupInterface;
using Clock = std::chrono::steady_clock;

class AssemblyTask
{
public:
    explicit AssemblyTask(const rclcpp::Node::SharedPtr& node) : node_(node)
    {
        raw_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
            "/isaac_joint_states", rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                raw_ = *msg;
                raw_time_ = Clock::now();
            });
        part_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/assembly/part_pose", 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                part_ = *msg;
                part_time_ = Clock::now();
            });
        tcp_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/assembly/tcp_pose", 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                tcp_ = *msg;
                tcp_time_ = Clock::now();
            });
        ready_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            "/assembly/ready", rclcpp::QoS(1).transient_local(), [this](const std_msgs::msg::Bool::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                ready_ = msg->data;
            });
        grasp_ = node_->create_client<std_srvs::srv::SetBool>("/assembly/grasp");
        scene_ = node_->create_client<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene");
        get_scene_ = node_->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
        validity_ = node_->create_client<moveit_msgs::srv::GetStateValidity>("/check_state_validity");
        ik_ = node_->create_client<moveit_msgs::srv::GetPositionIK>("/compute_ik");
        controllers_ = node_->create_client<controller_manager_msgs::srv::ListControllers>("/controller_manager/list_controllers");
    }

    void run()
    {
        require(node_->get_parameter("use_sim_time").as_bool(), "Task requires use_sim_time=true");
        wait_ready();
        check_control_sources(true);
        full_ = std::make_unique<Group>(node_, "jaka_kargo_full_robot");
        dual_ = std::make_unique<Group>(node_, "jaka_kargo_dual_arms");
        left_ = std::make_unique<Group>(node_, "jaka_kargo_arm_l");
        right_ = std::make_unique<Group>(node_, "jaka_kargo_arm_r");
        for (const auto& name : {"velocity_scaling", "acceleration_scaling",
                                "precision_velocity_scaling", "precision_acceleration_scaling"}) {
            const auto value = parameter<double>(name);
            require(std::isfinite(value) && value > 0.0 && value <= 1.0,
                    std::string(name) + " must be finite and in (0, 1]");
        }
        for (auto* group : {dual_.get(), left_.get(), right_.get()}) {
            group->setPlanningTime(15.0);
            group->setNumPlanningAttempts(5);
            group->setPoseReferenceFrame("world");
            group->setGoalPositionTolerance(0.005);
            group->setGoalOrientationTolerance(0.05);
            group->setMaxVelocityScalingFactor(parameter<double>("velocity_scaling"));
            group->setMaxAccelerationScalingFactor(parameter<double>("acceleration_scaling"));
        }
        require(full_->getPlanningFrame() == "world", "Expected planning frame world");
        home_ = current();
        check_feedback(*home_);
        log_initial_state(*home_);
        home_->enforceBounds();
        check_attachment(false, false);
        sync_scene();
        valid(*home_);
        const auto& dual_joints = dual_->getActiveJoints();
        require(dual_joints.size() == 14 && std::all_of(dual_joints.begin(), dual_joints.end(),
                [](const auto& name) { return name.rfind("arm_", 0) == 0; }),
                "Dual-arm group must contain exactly the 14 arm joints");
        const auto mode = parameter<std::string>("mode");
        if (mode == "single" || mode == "dual") {
            smoke(mode == "dual");
        } else if (mode == "assembly") {
            assemble();
        } else {
            throw std::runtime_error("Unknown mode: " + mode);
        }
        RCLCPP_INFO(node_->get_logger(), "STATE=SUCCESS mode=%s", mode.c_str());
    }

    void stop()
    {
        if (full_) full_->stop();
        if (dual_) dual_->stop();
        if (left_) left_->stop();
        if (right_) right_->stop();
    }

private:
    template<typename T> T parameter(const std::string& name) const
    {
        return node_->get_parameter(name).get_value<T>();
    }

    static void require(bool value, const std::string& message)
    {
        if (!value) throw std::runtime_error(message);
    }

    template<typename Service>
    typename Service::Response::SharedPtr call(
        const typename rclcpp::Client<Service>::SharedPtr& client,
        const typename Service::Request::SharedPtr& request,
        std::chrono::seconds timeout = 20s)
    {
        require(client->wait_for_service(15s), "Service unavailable: " + std::string(client->get_service_name()));
        auto future = client->async_send_request(request);
        if (future.wait_for(timeout) != std::future_status::ready) {
            client->remove_pending_request(future);
            throw std::runtime_error("Service timeout; physical state may be uncertain: " + std::string(client->get_service_name()));
        }
        return future.get();
    }

    bool feedback_ready()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto now = Clock::now();
        return ready_ && now - raw_time_ < 2s && now - part_time_ < 2s && now - tcp_time_ < 2s
            && raw_.header.stamp.sec + raw_.header.stamp.nanosec * 1e-9 > 0.0;
    }

    void wait_ready()
    {
        const auto deadline = Clock::now() + 90s;
        RCLCPP_INFO(node_->get_logger(), "Waiting for live Isaac joint/part/TCP feedback and /clock");
        while (rclcpp::ok() && Clock::now() < deadline) {
            if (feedback_ready() && node_->now().seconds() > 0.0) return;
            std::this_thread::sleep_for(100ms);
        }
        throw std::runtime_error("Isaac feedback timeout: start assembly=true and check /clock/DDS");
    }

    void check_control_sources(bool wait_for_discovery = false)
    {
        const auto deadline = Clock::now() + 5s;
        for (const auto& topic : {"/isaac_joint_commands", "/isaac_joint_states", "/joint_states", "/clock"}) {
            auto count = node_->count_publishers(topic);
            while (wait_for_discovery && count == 0 && rclcpp::ok() && Clock::now() < deadline) {
                std::this_thread::sleep_for(100ms);
                count = node_->count_publishers(topic);
            }
            require(count == 1, "Expected exactly one publisher on " + std::string(topic)
                    + "; discovered " + std::to_string(count));
        }
        for (const auto& name : node_->get_node_names()) {
            const auto leaf = name.substr(name.find_last_of('/') + 1);
            require(leaf != "jaka_kargo_driver" && leaf != "kargo_moveit_server",
                    "Hardware bridge is running: " + name);
        }
        controller_manager_msgs::srv::ListControllers::Response::SharedPtr result;
        for (int attempt = 0; attempt < 3 && !result; ++attempt) {
            try {
                result = call<controller_manager_msgs::srv::ListControllers>(controllers_,
                    std::make_shared<controller_manager_msgs::srv::ListControllers::Request>(), 5s);
            } catch (const std::runtime_error&) {
                if (attempt == 2) throw;
                RCLCPP_WARN(node_->get_logger(), "Retrying read-only controller discovery query");
            }
        }
        bool full = false;
        for (const auto& controller : result->controller) {
            if (controller.state != "active") continue;
            if (controller.name == "jaka_kargo_full_robot_controller") full = true;
            else require(controller.name == "joint_state_broadcaster", "Conflicting active controller: " + controller.name);
        }
        require(full, "Full robot trajectory controller is not active");
    }

    moveit::core::RobotStatePtr current()
    {
        require(feedback_ready(), "Isaac feedback is stale; aborting task");
        auto state = full_->getCurrentState(5.0);
        require(static_cast<bool>(state), "MoveIt did not receive a complete current joint state");
        state->update();
        require(state->satisfiesBounds(full_->getRobotModel()->getJointModelGroup("jaka_kargo_full_robot"), 0.005), "Measured robot state violates joint limits");
        return state;
    }

    void check_feedback(const moveit::core::RobotState& state)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        require(raw_.name.size() == raw_.position.size() && raw_.velocity.size() == raw_.name.size(),
                "Isaac must publish measured position AND velocity for every joint");
        for (const auto& name : full_->getActiveJoints()) {
            auto it = std::find(raw_.name.begin(), raw_.name.end(), name);
            require(it != raw_.name.end(), "Missing Isaac joint: " + name);
            const auto index = std::distance(raw_.name.begin(), it);
            require(std::isfinite(raw_.position[index]) && std::isfinite(raw_.velocity[index]), "Non-finite Isaac joint: " + name);
            require(std::abs(state.getVariablePosition(name) - raw_.position[index]) < 0.035,
                    "MoveIt/Isaac joint mismatch: " + name);
        }
    }

    void log_initial_state(const moveit::core::RobotState& state)
    {
        for (const auto& name : full_->getActiveJoints()) {
            RCLCPP_INFO(node_->get_logger(), "INITIAL_JOINT %s=%.6f", name.c_str(), state.getVariablePosition(name));
        }
        const auto& flange = state.getGlobalLinkTransform("arm_lj5_link");
        const auto& p = flange.translation();
        RCLCPP_INFO(node_->get_logger(), "INITIAL_LINK arm_lj5_link xyz=[%.6f, %.6f, %.6f]",
                    p.x(), p.y(), p.z());
        const auto bench = parameter<std::vector<double>>("workbench.position");
        const auto size = parameter<std::vector<double>>("workbench.size");
        RCLCPP_INFO(node_->get_logger(), "WORKBENCH center=[%.3f, %.3f, %.3f] size=[%.3f, %.3f, %.3f]",
                    bench.at(0), bench.at(1), bench.at(2), size.at(0), size.at(1), size.at(2));
    }

    void valid(const moveit::core::RobotState& state)
    {
        auto request = std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
        moveit::core::robotStateToRobotStateMsg(state, request->robot_state);
        request->robot_state.is_diff = true;  // Retain the scene's attached collision bodies.
        request->group_name = "jaka_kargo_full_robot";
        auto response = call<moveit_msgs::srv::GetStateValidity>(validity_, request);
        for (const auto& contact : response->contacts) {
            RCLCPP_ERROR(node_->get_logger(), "Collision: %s / %s depth=%.6f at=[%.6f, %.6f, %.6f]",
                         contact.contact_body_1.c_str(), contact.contact_body_2.c_str(), contact.depth,
                         contact.position.x, contact.position.y, contact.position.z);
        }
        require(response->valid, "Robot state is in collision or violates constraints");
    }

    void stage(const std::string& name, const std::function<void()>& action)
    {
        const auto start = Clock::now();
        RCLCPP_INFO(node_->get_logger(), "STATE=%s ENTER", name.c_str());
        check_control_sources();
        require(feedback_ready(), "Isaac feedback stale on entering " + name);
        action();
        RCLCPP_INFO(node_->get_logger(), "STATE=%s OK wall_time=%.3f", name.c_str(),
                    std::chrono::duration<double>(Clock::now() - start).count());
    }

    void execute(Group& group)
    {
        require(feedback_ready(), "No live physical feedback before planning");
        sensor_msgs::msg::JointState raw_start;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            raw_start = raw_;
        }
        group.setStartStateToCurrentState();
        Group::Plan plan;
        const auto begin = Clock::now();
        const auto result = group.plan(plan);
        require(result == moveit::core::MoveItErrorCode::SUCCESS,
                "Planning failed for " + group.getName() + " code=" + std::to_string(result.val));
        require(!plan.trajectory_.joint_trajectory.points.empty(), "Planner returned an empty trajectory");
        const auto& planned_joints = plan.trajectory_.joint_trajectory.joint_names;
        require(std::all_of(planned_joints.begin(), planned_joints.end(),
                [](const auto& name) { return name.rfind("arm_", 0) == 0; }),
                "Assembly trajectories must not command body or AGV joints");
        RCLCPP_INFO(node_->get_logger(), "PLAN group=%s planner_time=%.3f wall_time=%.3f points=%zu trajectory_sim_time=%.3f",
                    group.getName().c_str(), plan.planning_time_,
                    std::chrono::duration<double>(Clock::now() - begin).count(), plan.trajectory_.joint_trajectory.points.size(),
                    rclcpp::Duration(plan.trajectory_.joint_trajectory.points.back().time_from_start).seconds());
        // A wall-clock watchdog cancels execution if physics/clock stops or feedback disappears.
        auto timed_out = std::make_shared<std::atomic_bool>(false);
        auto watchdog = node_->create_wall_timer(500ms, [this, timed_out, begin, &group]() {
            if (!feedback_ready() || Clock::now() - begin > 300s) {
                timed_out->store(true);
                group.stop();
            }
        });
        const auto execution_begin = Clock::now();
        const double sim_begin = node_->now().seconds();
        const auto executed = group.execute(plan);
        watchdog->cancel();
        require(!timed_out->load() && executed == moveit::core::MoveItErrorCode::SUCCESS,
                "Execution aborted or failed, code=" + std::to_string(executed.val));
        auto measured = current();
        check_feedback(*measured);
        valid(*measured);
        const auto& trajectory = plan.trajectory_.joint_trajectory;
        double joint_error = 0.0;
        sensor_msgs::msg::JointState raw_end;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            raw_end = raw_;
        }
        for (size_t i = 0; i < trajectory.joint_names.size(); ++i) {
            joint_error = std::max(joint_error, std::abs(measured->getVariablePosition(trajectory.joint_names[i])
                                                      - trajectory.points.back().positions[i]));
            const auto& name = trajectory.joint_names[i];
            const auto start = std::find(raw_start.name.begin(), raw_start.name.end(), name);
            const auto end = std::find(raw_end.name.begin(), raw_end.name.end(), name);
            require(start != raw_start.name.end() && end != raw_end.name.end(), "Missing measured joint: " + name);
            const double before = raw_start.position.at(std::distance(raw_start.name.begin(), start));
            const double after = raw_end.position.at(std::distance(raw_end.name.begin(), end));
            const double requested = trajectory.points.back().positions[i] - before;
            if (std::abs(requested) > 0.07) {
                RCLCPP_INFO(node_->get_logger(), "MOTION joint=%s start=%.6f goal=%.6f actual=%.6f delta=%.6f",
                            name.c_str(), before, trajectory.points.back().positions[i], after, after - before);
                require(std::abs(after - before - requested) < 0.035,
                        "Isaac measured joint did not perform requested motion: " + name);
            }
        }
        require(joint_error < 0.035, "Measured joint goal error exceeds 0.035 rad/m");
        const double wall_elapsed = std::chrono::duration<double>(Clock::now() - execution_begin).count();
        const double sim_elapsed = node_->now().seconds() - sim_begin;
        RCLCPP_INFO(node_->get_logger(), "EXECUTE wall_time=%.3f sim_time=%.3f real_time_factor=%.3f max_joint_error=%.6f collision_check=valid",
                    wall_elapsed, sim_elapsed, sim_elapsed / std::max(wall_elapsed, 1e-6), joint_error);
    }

    void joint_target(Group& group, const moveit::core::RobotState& target)
    {
        require(group.setJointValueTarget(target), "Joint target outside bounds");
        execute(group);
    }

    void smoke(bool dual)
    {
        const double delta = parameter<double>("test_delta");
        const double hold = parameter<double>("test_hold_seconds");
        require(std::isfinite(delta) && std::abs(delta) >= 0.15 && std::abs(delta) <= 0.5,
                "test_delta must have magnitude between 0.15 and 0.5 rad");
        require(std::isfinite(hold) && hold >= 0.0 && hold <= 60.0,
                "test_hold_seconds must be between 0 and 60 seconds");
        stage("INIT", [&]() { valid(*current()); });
        stage(dual ? "DUAL_JOINT_TEST" : "SINGLE_JOINT_TEST", [&]() {
            const auto start = current();
            auto target = std::make_shared<moveit::core::RobotState>(*start);
            target->setVariablePosition("arm_lj4", target->getVariablePosition("arm_lj4") + delta);
            if (dual) target->setVariablePosition("arm_rj4", target->getVariablePosition("arm_rj4") + delta);
            target->update();
            const Eigen::Vector3d before = start->getGlobalLinkTransform("arm_l_tcp").translation();
            const Eigen::Vector3d right_before = start->getGlobalLinkTransform("arm_r_tcp").translation();
            const double expected = (target->getGlobalLinkTransform("arm_l_tcp").translation() - before).norm();
            RCLCPP_INFO(node_->get_logger(), "TEST elbow_delta=%.3f rad expected_left_tcp_displacement=%.4f m", delta, expected);
            valid(*target);
            joint_target(dual ? *dual_ : *left_, *target);
            const auto actual = current();
            const double moved = (actual->getGlobalLinkTransform("arm_l_tcp").translation() - before).norm();
            RCLCPP_INFO(node_->get_logger(), "TEST measured_left_tcp_displacement=%.4f m (MoveIt FK from feedback)", moved);
            require(moved > expected * 0.7, "Measured arm displacement is too small for the smoke test");
            if (dual) {
                const double right_expected = (target->getGlobalLinkTransform("arm_r_tcp").translation() - right_before).norm();
                const double right_moved = (actual->getGlobalLinkTransform("arm_r_tcp").translation() - right_before).norm();
                RCLCPP_INFO(node_->get_logger(), "TEST measured_right_tcp_displacement=%.4f m (MoveIt FK from feedback)", right_moved);
                require(right_moved > right_expected * 0.7, "Measured right-arm displacement is too small for the smoke test");
            }
        });
        stage("HOLD_FOR_OBSERVATION", [&]() {
            RCLCPP_INFO(node_->get_logger(), "Holding elbow test pose for %.1f wall seconds before returning HOME", hold);
            const auto until = Clock::now() + std::chrono::duration<double>(hold);
            while (rclcpp::ok() && Clock::now() < until) {
                require(feedback_ready(), "Isaac feedback stopped during observation hold");
                std::this_thread::sleep_for(100ms);
            }
            require(rclcpp::ok(), "Task interrupted during observation hold");
        });
        stage("HOME", [&]() { joint_target(dual ? *dual_ : *left_, *home_); });
    }

    geometry_msgs::msg::Pose part_pose()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        require(Clock::now() - part_time_ < 2s && part_.header.frame_id == "world", "Invalid/stale physical part pose");
        return part_.pose;
    }

    geometry_msgs::msg::Pose pose(const std::vector<double>& position)
    {
        const auto rotation = parameter<std::vector<double>>("tool_orientation_xyzw");
        require(position.size() == 3 && rotation.size() == 4, "Invalid pose configuration");
        geometry_msgs::msg::Pose target;
        target.position.x = position[0]; target.position.y = position[1]; target.position.z = position[2];
        target.orientation.x = rotation[0]; target.orientation.y = rotation[1];
        target.orientation.z = rotation[2]; target.orientation.w = rotation[3];
        return target;
    }

    void pose_target(Group& group, const geometry_msgs::msg::Pose& target, const std::string& tip, bool precision = false)
    {
        group.setMaxVelocityScalingFactor(parameter<double>(precision ? "precision_velocity_scaling" : "velocity_scaling"));
        group.setMaxAccelerationScalingFactor(parameter<double>(precision ? "precision_acceleration_scaling" : "acceleration_scaling"));
        group.clearPoseTargets();
        auto seed = current();
        solve_pose(*seed, group, target, tip);
        valid(*seed);
        joint_target(group, *seed);
        const auto actual = group.getCurrentPose(tip).pose;
        const double error = std::sqrt(std::pow(actual.position.x - target.position.x, 2)
            + std::pow(actual.position.y - target.position.y, 2) + std::pow(actual.position.z - target.position.z, 2));
        Eigen::Quaterniond a(actual.orientation.w, actual.orientation.x, actual.orientation.y, actual.orientation.z);
        Eigen::Quaterniond b(target.orientation.w, target.orientation.x, target.orientation.y, target.orientation.z);
        const double angular = a.angularDistance(b);
        RCLCPP_INFO(node_->get_logger(), "TCP %s position_error=%.5f orientation_error=%.5f", tip.c_str(), error, angular);
        require(error < 0.015 && angular < 0.10, "Measured TCP did not reach target");
        if (tip == "arm_l_tcp") {
            std::lock_guard<std::mutex> lock(mutex_);
            const auto& p = tcp_.pose.position;
            const double mismatch = std::sqrt(std::pow(p.x - actual.position.x, 2)
                + std::pow(p.y - actual.position.y, 2) + std::pow(p.z - actual.position.z, 2));
            require(mismatch < 0.015, "USD TCP does not match MoveIt FK");
        }
    }

    void dual_pose(const geometry_msgs::msg::Pose& left, const geometry_msgs::msg::Pose& right)
    {
        auto target = current();
        solve_pose(*target, *left_, left, "arm_l_tcp");
        solve_pose(*target, *right_, right, "arm_r_tcp");
        target->update();
        valid(*target);
        joint_target(*dual_, *target);
    }

    void solve_pose(moveit::core::RobotState& seed, Group& group,
                    const geometry_msgs::msg::Pose& pose, const std::string& tip)
    {
        auto request = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
        auto& ik = request->ik_request;
        ik.group_name = group.getName();
        ik.ik_link_name = tip;
        ik.avoid_collisions = true;
        ik.timeout.sec = 3;
        ik.pose_stamped.header.frame_id = "world";
        ik.pose_stamped.pose = pose;
        moveit::core::robotStateToRobotStateMsg(seed, ik.robot_state);
        ik.robot_state.is_diff = true;
        const auto response = call<moveit_msgs::srv::GetPositionIK>(ik_, request);
        require(response->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS,
                "Collision-aware IK failed for " + tip + " code=" + std::to_string(response->error_code.val));
        const auto& joints = response->solution.joint_state;
        double maximum_delta = 0.0;
        for (const auto& name : group.getActiveJoints()) {
            const auto found = std::find(joints.name.begin(), joints.name.end(), name);
            require(found != joints.name.end(), "IK solution missing joint " + name);
            const double before = seed.getVariablePosition(name);
            double nearest = joints.position.at(std::distance(joints.name.begin(), found));
            const auto& bounds = seed.getRobotModel()->getVariableBounds(name);
            const double original = nearest;
            // Arm joints are revolute. Equivalent turns preserve FK and collision geometry,
            // but can differ by several seconds of needless travel inside bounded limits.
            for (int turns = -2; turns <= 2; ++turns) {
                const double candidate = original + turns * 2.0 * std::acos(-1.0);
                if (bounds.position_bounded_ && (candidate < bounds.min_position_ || candidate > bounds.max_position_)) continue;
                if (std::abs(candidate - before) < std::abs(nearest - before)) nearest = candidate;
            }
            seed.setVariablePosition(name, nearest);
            maximum_delta = std::max(maximum_delta, std::abs(nearest - before));
        }
        seed.update();
        RCLCPP_INFO(node_->get_logger(), "IK group=%s max_joint_delta=%.3f rad seeded_from_feedback=true",
                    group.getName().c_str(), maximum_delta);
    }

    moveit_msgs::msg::CollisionObject box(const std::string& name, const geometry_msgs::msg::Pose& position)
    {
        moveit_msgs::msg::CollisionObject object;
        object.header.frame_id = "world";
        object.id = name;
        object.operation = object.ADD;
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        const auto dimensions = parameter<std::vector<double>>(name + ".size");
        require(dimensions.size() == 3, "Box size must contain three dimensions");
        primitive.dimensions.assign(dimensions.begin(), dimensions.end());
        object.primitives.push_back(primitive);
        object.primitive_poses.push_back(position);
        return object;
    }

    void apply(moveit_msgs::msg::PlanningScene& scene, const std::string& operation = "scene update")
    {
        scene.is_diff = true;
        scene.robot_state.is_diff = true;
        auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
        request->scene = scene;
        require(call<moveit_msgs::srv::ApplyPlanningScene>(scene_, request)->success,
                "MoveIt rejected " + operation + "; inspect physical grasp state before restarting");
    }

    void sync_scene()
    {
        moveit_msgs::msg::PlanningScene scene;
        for (const auto& name : parameter<std::vector<std::string>>("object_names")) {
            auto position = pose(parameter<std::vector<double>>(name + ".position"));
            position.orientation = geometry_msgs::msg::Quaternion();
            position.orientation.w = 1.0;
            if (name == "part_a") position = part_pose();
            scene.world.collision_objects.push_back(box(name, position));
        }
        apply(scene);
    }

    void physical_grasp(bool attach)
    {
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = attach;
        auto response = call<std_srvs::srv::SetBool>(grasp_, request);
        require(response->success, "Physical grasp failed: " + response->message);
    }

    void support_contact(const std::string& support, bool allowed)
    {
        auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
        request->components.components = moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
        const auto response = call<moveit_msgs::srv::GetPlanningScene>(get_scene_, request);
        collision_detection::AllowedCollisionMatrix matrix(response->scene.allowed_collision_matrix);
        matrix.setEntry("part_a", support, allowed);
        moveit_msgs::msg::PlanningScene scene;
        matrix.getMessage(scene.allowed_collision_matrix);
        apply(scene);
    }

    void attach_part()
    {
        support_contact("workbench", true);
        physical_grasp(true);
        moveit_msgs::msg::PlanningScene scene;
        moveit_msgs::msg::AttachedCollisionObject object;
        object.link_name = "arm_l_tcp";
        object.touch_links = {"arm_l_tcp", "arm_lj7_link"};
        object.object = box("part_a", part_pose());
        scene.robot_state.attached_collision_objects.push_back(object);
        // MoveIt removes the world object when attaching it. An extra world REMOVE
        // in the same diff is processed afterwards and makes this update fail.
        // If synchronization fails, stop while keeping the real constraint in place.
        apply(scene, "attaching part_a (physical fixed joint remains active on failure)");
        check_attachment(true);
    }

    void check_attachment(bool attached, bool require_world = true)
    {
        auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
        request->components.components = moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS
            | moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_NAMES;
        const auto scene = call<moveit_msgs::srv::GetPlanningScene>(get_scene_, request)->scene;
        bool found_attached = false;
        for (const auto& item : scene.robot_state.attached_collision_objects) {
            if (item.object.id != "part_a") continue;
            found_attached = true;
            require(item.link_name == "arm_l_tcp", "part_a is attached to an unexpected link");
        }
        const bool found_world = std::any_of(scene.world.collision_objects.begin(), scene.world.collision_objects.end(),
            [](const auto& item) { return item.id == "part_a"; });
        require(found_attached == attached, "Unexpected part_a attachment state; reset the assembly scene before restarting");
        require(attached ? !found_world : (!require_world || found_world), "part_a has inconsistent world/attached scene membership");
        RCLCPP_INFO(node_->get_logger(), "SCENE part_a attached=%s world=%s verified", found_attached ? "true" : "false",
                    found_world ? "true" : "false");
    }

    void release_part()
    {
        physical_grasp(false);
        const auto until = node_->now() + rclcpp::Duration::from_seconds(1.0);
        const auto deadline = Clock::now() + 10s;
        while (node_->now() < until && Clock::now() < deadline && rclcpp::ok()) std::this_thread::sleep_for(20ms);
        require(node_->now() >= until && feedback_ready(), "Physics did not advance after release");
        moveit_msgs::msg::PlanningScene scene;
        moveit_msgs::msg::AttachedCollisionObject remove;
        remove.link_name = "arm_l_tcp";
        remove.object.id = "part_a";
        remove.object.operation = remove.object.REMOVE;
        scene.robot_state.attached_collision_objects.push_back(remove);
        scene.world.collision_objects.push_back(box("part_a", part_pose()));
        apply(scene, "detaching part_a and restoring its measured world pose");
        check_attachment(false);
        const auto actual = part_pose();
        const auto goal = parameter<std::vector<double>>("target_position");
        const double error = std::sqrt(std::pow(actual.position.x - goal[0], 2)
            + std::pow(actual.position.y - goal[1], 2) + std::pow(actual.position.z - goal[2], 2));
        const auto& q = actual.orientation;
        const double angular = Eigen::Quaterniond(q.w, q.x, q.y, q.z).angularDistance(Eigen::Quaterniond::Identity());
        RCLCPP_INFO(node_->get_logger(), "PART final_position_error=%.5f orientation_error=%.5f", error, angular);
        require(error < parameter<double>("target_tolerance") && angular < parameter<double>("target_orientation_tolerance"),
                "Physical part placement outside tolerance");
    }

    void assemble()
    {
        const double half = parameter<std::vector<double>>("part_a.size")[2] / 2;
        const auto actual = part_pose();
        auto grasp = pose({actual.position.x, actual.position.y, actual.position.z + half});
        auto pregrasp = grasp;
        pregrasp.position.z += parameter<double>("approach_distance");
        auto lift = grasp;
        lift.position.z += parameter<double>("lift_distance");
        auto place = pose(parameter<std::vector<double>>("target_position"));
        place.position.z += half + 0.003;  // Small clearance; gravity settles onto the base.
        auto transfer = place;
        transfer.position.z += parameter<double>("lift_distance");
        auto right_ready = pose(parameter<std::vector<double>>("right_ready_position"));
        stage("INIT", [&]() { valid(*current()); joint_target(*dual_, *home_); });
        stage("MOVE_TO_PREGRASP", [&]() { pose_target(*left_, pregrasp, "arm_l_tcp"); });
        stage("APPROACH", [&]() { pose_target(*left_, grasp, "arm_l_tcp", true); });
        stage("GRASP", [&]() { attach_part(); });
        stage("LIFT", [&]() {
            pose_target(*left_, lift, "arm_l_tcp");
            require(part_pose().position.z > actual.position.z + 0.08, "Physical part did not follow the grasp constraint");
            support_contact("workbench", false);
        });
        stage("TRANSFER", [&]() { dual_pose(transfer, right_ready); });
        stage("PLACE", [&]() {
            support_contact("assembly_base", true);
            pose_target(*left_, place, "arm_l_tcp", true);
        });
        stage("RELEASE", [&]() { release_part(); });
        stage("RETREAT", [&]() { pose_target(*left_, transfer, "arm_l_tcp"); });
        stage("HOME", [&]() { joint_target(*dual_, *home_); });
        RCLCPP_INFO(node_->get_logger(), "Task success_count=1 attempt_count=1; aggregate repeated runs externally");
    }

    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<Group> full_, dual_, left_, right_;
    moveit::core::RobotStatePtr home_;
    std::mutex mutex_;
    sensor_msgs::msg::JointState raw_;
    geometry_msgs::msg::PoseStamped part_, tcp_;
    Clock::time_point raw_time_{}, part_time_{}, tcp_time_{};
    bool ready_{false};
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr raw_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr part_sub_, tcp_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_sub_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr grasp_;
    rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr scene_;
    rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_scene_;
    rclcpp::Client<moveit_msgs::srv::GetStateValidity>::SharedPtr validity_;
    rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr ik_;
    rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr controllers_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("kargo_assembly_task",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]() { executor.spin(); });
    int result = 0;
    {
        AssemblyTask task(node);
        try {
            task.run();
        } catch (const std::exception& error) {
            task.stop();
            RCLCPP_ERROR(node->get_logger(), "STATE=FAILED %s. No further motion; inspect grasp/scene state before restart.", error.what());
            result = 1;
        }
        executor.cancel();
        spinner.join();
    }
    rclcpp::shutdown();
    return result;
}
