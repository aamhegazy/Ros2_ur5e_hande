#include <chrono>
#include <memory>
#include <mutex>
#include <random>
#include <sstream>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "ur5e_moveit_actions/action/plan_to_pose.hpp"
#include "ur5e_moveit_actions/action/execute_plan.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

using PlanToPose = ur5e_moveit_actions::action::PlanToPose;
using ExecutePlan = ur5e_moveit_actions::action::ExecutePlan;
using GoalHandlePlan = rclcpp_action::ServerGoalHandle<PlanToPose>;
using GoalHandleExecute = rclcpp_action::ServerGoalHandle<ExecutePlan>;

// Structured storage for plans with a timestamp for TTL logic
struct CachedPlan {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    rclcpp::Time created_at;
};

static std::string make_plan_id() {
    static std::random_device rd;
    static std::mt19937_64 gen(rd());
    std::uniform_int_distribution<uint64_t> dist;
    std::stringstream ss;
    ss << std::hex << dist(gen);
    return ss.str();
}

class MoveItActionServer : public rclcpp::Node {
public:
    MoveItActionServer() : Node("moveit_action_server", 
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) 
    {
        planning_group_ = this->get_parameter("planning_group").as_string();
base_frame_ = this->get_parameter("base_frame").as_string();
tcp_link_ = this->get_parameter("tcp_link").as_string();
plan_ttl_seconds_ = this->get_parameter("plan_ttl_seconds").as_double();
waypoint_stride_ = this->get_parameter("waypoint_stride").as_int();
    }

  void init() {
    // Get parameters (already declared by launch file via params-file)
    planning_group_ = this->get_parameter("planning_group").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    tcp_link_ = this->get_parameter("tcp_link").as_string();
    plan_ttl_seconds_ = this->get_parameter("plan_ttl_seconds").as_double();
    waypoint_stride_ = this->get_parameter("waypoint_stride").as_int();
    
    // MoveGroupInterface requires shared_from_this(), which is available after constructor
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), planning_group_);
    
    move_group_->setPoseReferenceFrame(base_frame_);
    move_group_->setEndEffectorLink(tcp_link_);

    plan_server_ = rclcpp_action::create_server<PlanToPose>(
        this, "plan_to_pose",
        std::bind(&MoveItActionServer::handle_plan_goal, this, _1, _2),
        std::bind(&MoveItActionServer::handle_plan_cancel, this, _1),
        std::bind(&MoveItActionServer::handle_plan_accepted, this, _1));

    exec_server_ = rclcpp_action::create_server<ExecutePlan>(
        this, "execute_plan",
        std::bind(&MoveItActionServer::handle_exec_goal, this, _1, _2),
        std::bind(&MoveItActionServer::handle_exec_cancel, this, _1),
        std::bind(&MoveItActionServer::handle_exec_accepted, this, _1));

    cleanup_timer_ = create_wall_timer(30s, std::bind(&MoveItActionServer::cleanup_stale_plans, this));
}

private:
    // --- PlanToPose Handlers ---
// Notice how 'goal' is wrapped in comments below
rclcpp_action::GoalResponse handle_plan_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const PlanToPose::Goal> /*goal*/) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

    rclcpp_action::CancelResponse handle_plan_cancel(const std::shared_ptr<GoalHandlePlan>) {
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_plan_accepted(const std::shared_ptr<GoalHandlePlan> goal_handle) {
        std::thread{std::bind(&MoveItActionServer::execute_plan_goal, this, goal_handle)}.detach();
    }

    void execute_plan_goal(const std::shared_ptr<GoalHandlePlan> goal_handle) {
        auto goal = goal_handle->get_goal();
        auto result = std::make_shared<PlanToPose::Result>();
        auto feedback = std::make_shared<PlanToPose::Feedback>();

        move_group_->setPoseTarget(goal->target_pose.pose, tcp_link_);
        
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto rc = move_group_->plan(plan);

        if (rc == moveit::core::MoveItErrorCode::SUCCESS) {
            std::string plan_id = make_plan_id();
            {
                std::lock_guard<std::mutex> lk(plans_mutex_);
                plans_[plan_id] = {plan, now()};
            }
            result->success = true;
            result->plan_id = plan_id;
            result->tcp_waypoints = extract_tcp_waypoints(plan);
            goal_handle->succeed(result);
        } else {
            result->success = false;
            goal_handle->abort(result);
        }
    }

    // --- ExecutePlan Handlers ---
    rclcpp_action::GoalResponse handle_exec_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const ExecutePlan::Goal> goal) {
        std::lock_guard<std::mutex> lk(plans_mutex_);
        if (plans_.find(goal->plan_id) == plans_.end()) return rclcpp_action::GoalResponse::REJECT;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_exec_cancel(const std::shared_ptr<GoalHandleExecute>) {
        move_group_->stop(); // Immediate stop on cancel
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_exec_accepted(const std::shared_ptr<GoalHandleExecute> goal_handle) {
        std::thread{std::bind(&MoveItActionServer::execute_exec_goal, this, goal_handle)}.detach();
    }

    void execute_exec_goal(const std::shared_ptr<GoalHandleExecute> goal_handle) {
        auto goal = goal_handle->get_goal();
        auto result = std::make_shared<ExecutePlan::Result>();
        auto feedback = std::make_shared<ExecutePlan::Feedback>();

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        {
            std::lock_guard<std::mutex> lk(plans_mutex_);
            auto it = plans_.find(goal->plan_id);
            if (it == plans_.end()) {
                result->success = false;
                goal_handle->abort(result);
                return;
            }
            plan = it->second.plan;
            plans_.erase(it); // One-shot execution
        }

        double duration = 0.0;
        if (!plan.trajectory_.joint_trajectory.points.empty()) {
            auto &last = plan.trajectory_.joint_trajectory.points.back();
            duration = last.time_from_start.sec + last.time_from_start.nanosec * 1e-9;
        }

        auto t0 = now();
        std::atomic<bool> done{false};
        std::thread progress_thread([this, goal_handle, feedback, t0, duration, &done]() {
            rclcpp::Rate r(10.0);
            while (!done.load() && rclcpp::ok()) {
                if (goal_handle->is_canceling()) break;
                double elapsed = (now() - t0).seconds();
                feedback->progress = static_cast<float>(std::min(1.0, elapsed / std::max(0.1, duration)));
                goal_handle->publish_feedback(feedback);
                r.sleep();
            }
        });

        auto rc = move_group_->execute(plan);
        done.store(true);
        if (progress_thread.joinable()) progress_thread.join();

        result->success = (rc == moveit::core::MoveItErrorCode::SUCCESS);
        result->final_pose = move_group_->getCurrentPose(tcp_link_);
        
        if (result->success) goal_handle->succeed(result);
        else goal_handle->abort(result);
    }

    // --- Utilities ---
    std::vector<geometry_msgs::msg::Point> extract_tcp_waypoints(const moveit::planning_interface::MoveGroupInterface::Plan &plan) {
        std::vector<geometry_msgs::msg::Point> out;
        const auto &traj = plan.trajectory_.joint_trajectory;
        if (traj.points.empty()) return out;

        moveit::core::RobotState state(move_group_->getRobotModel());
        
        auto process_pt = [&](const trajectory_msgs::msg::JointTrajectoryPoint &pt) {
            for (size_t j = 0; j < traj.joint_names.size() && j < pt.positions.size(); ++j)
                state.setJointPositions(traj.joint_names[j], &pt.positions[j]);
            state.update();
            auto tf = state.getGlobalLinkTransform(tcp_link_);
            geometry_msgs::msg::Point p;
            p.x = tf.translation().x(); p.y = tf.translation().y(); p.z = tf.translation().z();
            out.push_back(p);
        };

        for (size_t i = 0; i < traj.points.size(); i += std::max(1, waypoint_stride_)) process_pt(traj.points[i]);
        process_pt(traj.points.back()); // Ensure final point is included
        return out;
    }

    void cleanup_stale_plans() {
        std::lock_guard<std::mutex> lk(plans_mutex_);
        auto t = now();
        for (auto it = plans_.begin(); it != plans_.end();) {
            if ((t - it->second.created_at).seconds() > plan_ttl_seconds_) it = plans_.erase(it);
            else ++it;
        }
    }

    // Members
    std::string planning_group_, base_frame_, tcp_link_;
    double plan_ttl_seconds_;
    int waypoint_stride_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp_action::Server<PlanToPose>::SharedPtr plan_server_;
    rclcpp_action::Server<ExecutePlan>::SharedPtr exec_server_;
    std::mutex plans_mutex_;
    std::unordered_map<std::string, CachedPlan> plans_;
    rclcpp::TimerBase::SharedPtr cleanup_timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveItActionServer>();
    node->init();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}