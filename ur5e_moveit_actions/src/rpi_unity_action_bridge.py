#!/usr/bin/env python3

# A bridge allow Unity Topics & services to speak to ROS2 Actions, The script node bridges:
    
    # > unity publishes on/plan_to_pose/goal -> we call /plan_to_pose action 
        #  replies ->   we publish to plan to pose result 

    # > Unity Publish on /execute_plan/goal -> we call execute plan 
        # Action Server replies -> wr publish to /execute_plan/result 


import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node 
from ur5e_moveit_actions_msgs.action import PlanToPose, ExecutePlan 

class UnityActionBridge(Node):
    def __init__(self):
        super().__init__("unity_action_bridge")

        # Action Clients 
        self.plan_client_ = ActionClient(self, PlanToPose, "plan_to_pose")
        self.exec_client_ = ActionClient(self, ExecutePlan, "execute_plan")

        # Topic-based Shim/translator 
            # plan
        self.plan_goal_sub_= self.create_subscription(
            PlanToPose.Goal, "plan_to_pose/goal", self.on_plan_goal_, 10)
        self.plan_result_pub_ = self.create_publisher(
            PlanToPose.Result, "plan_to_pose/result", 10)
          
          
            # exec
        self.exec_goal_sub_ =  self.create_subscription(ExecutePlan.Goal, "execute_plan/goal", self.on_exec_goal_, 10)
        self.exec_result_pub_ = self.create_publisher(ExecutePlan.Result, "execute_plan/result", 10)

        self.get_logger().info("unity_action_bridge up ...")


# PLAN FUNC
    def on_plan_goal_(self, msg: PlanToPose.Goal):
        self.get_logger().info("Relaying plan goal to /plan_to_pse action.")
        #if it fails to connects to the server 
        if not self.plan_client_.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("/plan_to_pse action server not available")
            r = PlanToPose.Result()
            r.success = False
            r.message = "Action Server /plan_to_pose unavailable"
            self.plan_result_pub_.publish(r)
            return 
        
        #if server works, send the message as a goal to the plan_client_server 
        future = self.plan_client_.send_goal_async(msg)
        #once the Plan_client_server call_back is done , call the the function on_plan_goal_response_ and pass the result to it 
        future.add_done_callback(self.on_plan_goal_response_)

        
        # if the plan_server_goal is rejected , return
        #if goal accepted , get the result from the goal after callback, and call the function on_plan_reasult , and pass the result to it 
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
        self.get_logger().info(
            f"plan done: success={result.success} message='{result.message}'")
        self.plan_result_pub_.publish(result)


# EXECUTION FUNC

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
        handle=future.result()
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