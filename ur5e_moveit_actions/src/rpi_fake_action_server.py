#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from ur5e_moveit_actions_pi.action import PlanToPose, ExecutePlan

class FakeActionServer(Node):
    def __init__(self):
        super().__init__('fake_action_server')
        self.plan_server_ = ActionServer(
            self, PlanToPose, 'plan_to_pose', self.on_plan_goal_)
        self.exec_server_ = ActionServer(
            self, ExecutePlan, 'execute_plan', self.on_exec_goal_)
        self.get_logger().info('Fake action server ready')

    async def on_plan_goal_(self, goal_handle):
        self.get_logger().info(
            f'Received plan goal: position=({goal_handle.request.target_pose.position.x:.2f}, '
            f'{goal_handle.request.target_pose.position.y:.2f}, '
            f'{goal_handle.request.target_pose.position.z:.2f})')
        goal_handle.succeed()
        result = PlanToPose.Result()
        result.success = True
        result.message = 'Fake plan succeeded'
        result.plan_id = 'fake_plan_001'
        return result

    async def on_exec_goal_(self, goal_handle):
        self.get_logger().info(
            f'Received execute goal: plan_id={goal_handle.request.plan_id}')
        goal_handle.succeed()
        result = ExecutePlan.Result()
        result.success = True
        result.message = 'Fake execution succeeded'
        return result

def main():
    rclpy.init()
    node = FakeActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()