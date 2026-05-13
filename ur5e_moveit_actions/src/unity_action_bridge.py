#!/usr/bin/env python3

# A bridge allowing Unity Topics to speak to ROS2 Actions:
# > unity publishes on plan_to_pose/goal     -> we call /plan_to_pose action
# > unity publishes on execute_plan/goal     -> we call /execute_plan action
# > unity publishes on gripper/open          -> we call /gripper_controller/gripper_cmd (open)
# > unity publishes on gripper/close         -> we call /gripper_controller/gripper_cmd (close)

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from ur5e_moveit_actions.action import PlanToPose, ExecutePlan
from control_msgs.action import GripperCommand
from std_msgs.msg import Bool

class UnityActionBridge(Node):
    def __init__(self):
        super().__init__("unity_action_bridge")

        # Action Clients
        self.plan_client_ = ActionClient(self, PlanToPose, "plan_to_pose")
        self.exec_client_ = ActionClient(self, ExecutePlan, "execute_plan")
        self.gripper_client_ = ActionClient(self, GripperCommand, "/gripper_controller/gripper_cmd")

        # Plan topics
        self.plan_goal_sub_ = self.create_subscription(
            PlanToPose.Goal, "plan_to_pose/goal", self.on_plan_goal_, 10)
        self.plan_result_pub_ = self.create_publisher(
            PlanToPose.Result, "plan_to_pose/result", 10)

        # Execute topics
        self.exec_goal_sub_ = self.create_subscription(
            ExecutePlan.Goal, "execute_plan/goal", self.on_exec_goal_, 10)
        self.exec_result_pub_ = self.create_publisher(
            ExecutePlan.Result, "execute_plan/result", 10)

        # Gripper topics (Bool: True = open, False = close)
        self.gripper_open_sub_ = self.create_subscription(
            Bool, "gripper/open", self.on_gripper_open_, 10)
        self.gripper_close_sub_ = self.create_subscription(
            Bool, "gripper/close", self.on_gripper_close_, 10)
        self.gripper_result_pub_ = self.create_publisher(
            Bool, "gripper/result", 10)

        self.get_logger().info("unity_action_bridge up ...")

    # --- PLAN ---
    def on_plan_goal_(self, msg: PlanToPose.Goal):
        self.get_logger().info("Relaying plan goal to /plan_to_pose action.")
        if not self.plan_client_.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("/plan_to_pose action server not available")
            r = PlanToPose.Result()
            r.success = False
            r.message = "Action Server /plan_to_pose unavailable"
            self.plan_result_pub_.publish(r)
            return
        future = self.plan_client_.send_goal_async(msg)
        future.add_done_callback(self.on_plan_goal_response_)

    def on_plan_goal_response_(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("Plan goal rejected by action server.")
            r = PlanToPose.Result()
            r.success = False
            r.message = "Plan goal rejected"
            self.plan_result_pub_.publish(r)
            return
        handle.get_result_async().add_done_callback(self.on_plan_result_)

    def on_plan_result_(self, future):
        wrapped = future.result()
        result = wrapped.result
        self.get_logger().info(f"plan done: success={result.success} message='{result.message}'")
        self.plan_result_pub_.publish(result)

    # --- EXECUTE ---
    def on_exec_goal_(self, msg: ExecutePlan.Goal):
        self.get_logger().info(f"Relaying execute goal for plan_id='{msg.plan_id}'")
        if not self.exec_client_.wait_for_server(timeout_sec=2.0):
            r = ExecutePlan.Result()
            r.success = False
            r.message = "Execute goal Rejected"
            self.exec_result_pub_.publish(r)
            return
        future = self.exec_client_.send_goal_async(msg)
        future.add_done_callback(self.on_exec_goal_response_)

    def on_exec_goal_response_(self, future):
        handle = future.result()
        if not handle.accepted:
            r = ExecutePlan.Result()
            r.success = False
            r.message = "Execute goal is rejected"
            self.exec_result_pub_.publish(r)
            return
        handle.get_result_async().add_done_callback(self.on_exec_result_)

    def on_exec_result_(self, future):
        wrapped = future.result()
        result = wrapped.result
        self.get_logger().info(f"Execute done: success={result.success} message='{result.message}'")
        self.exec_result_pub_.publish(result)

    # --- GRIPPER ---
    def on_gripper_open_(self, msg: Bool):
        self.get_logger().info("Gripper open requested.")
        self.send_gripper_goal_(0.025)

    def on_gripper_close_(self, msg: Bool):
        self.get_logger().info("Gripper close requested.")
        self.send_gripper_goal_(0.0)

    def send_gripper_goal_(self, position: float):
        if not self.gripper_client_.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Gripper action server not available")
            self.gripper_result_pub_.publish(Bool(data=False))
            return
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 10.0
        future = self.gripper_client_.send_goal_async(goal)
        future.add_done_callback(self.on_gripper_goal_response_)

    def on_gripper_goal_response_(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("Gripper goal rejected.")
            self.gripper_result_pub_.publish(Bool(data=False))
            return
        handle.get_result_async().add_done_callback(self.on_gripper_result_)

    def on_gripper_result_(self, future):
        wrapped = future.result()
        result = wrapped.result
        success = result.reached_goal or result.stalled
        self.get_logger().info(f"Gripper done: reached_goal={result.reached_goal} stalled={result.stalled}")
        self.gripper_result_pub_.publish(Bool(data=success))


def main():
    rclpy.init()
    node = UnityActionBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()