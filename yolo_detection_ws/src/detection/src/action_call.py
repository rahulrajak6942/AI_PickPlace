#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped
from control_msgs.action import GripperCommand
import time


class PickPlaceClient(Node):
    def __init__(self):
        super().__init__('pick_place_client')
        # MoveIt action client
        self._move_client = ActionClient(self, MoveGroup, '/move_action')
        # Gripper action client
        self._gripper_client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')

        # Subscribe to detected target pose
        self.subscription = self.create_subscription(
            PoseStamped,
            '/detected_target_pose',
            self.pose_callback,
            10
        )
        self.get_logger().info("✅ PickPlaceClient ready, waiting for target poses...")

        self.done = False  # ✅ run only once

    def pose_callback(self, msg: PoseStamped):
        if self.done:
            self.get_logger().info("⏳ Sequence already executed, ignoring new poses")
            return

        self.get_logger().info(f"📩 Received target pose: {msg.pose.position}")
        self.done = True  # ✅ block future runs

        # Step 1: open gripper first
        self.open_gripper(lambda: self._delayed(2.0, lambda: self.move_to_pose(msg)))

    # ------------------ Gripper Control ------------------
    def open_gripper(self, done_cb=None):
        self.get_logger().info("🖐️ Opening gripper...")
        goal = GripperCommand.Goal()
        goal.command.position = 0.01  # open
        goal.command.max_effort = 0.0
        self._gripper_client.wait_for_server()
        future = self._gripper_client.send_goal_async(goal)
        future.add_done_callback(lambda f: self._after_gripper_goal(f, done_cb))

    def close_gripper(self, done_cb=None):
        self.get_logger().info("✊ Closing gripper...")
        goal = GripperCommand.Goal()
        goal.command.position = 0.003 # close
        goal.command.max_effort = 1.0
        self._gripper_client.wait_for_server()
        future = self._gripper_client.send_goal_async(goal)
        future.add_done_callback(lambda f: self._after_gripper_goal(f, done_cb))

    def _after_gripper_goal(self, future, done_cb):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("❌ Gripper goal rejected!")
            return
        self.get_logger().info("✅ Gripper goal accepted")
        result_future = handle.get_result_async()
        result_future.add_done_callback(lambda r: self._after_gripper_result(r, done_cb))

    def _after_gripper_result(self, future, done_cb):
        result = future.result().result
        self.get_logger().info(f"📦 Gripper result: {result}")
        if done_cb:
            done_cb()

    # ------------------ MoveIt Control ------------------
    def move_to_pose(self, target_pose: PoseStamped):
        """Step 2: move to target pose"""
        self.get_logger().info("🚀 Moving to target pose...")
        goal_msg = self._make_move_goal(target_pose)
        self._move_client.wait_for_server()
        future = self._move_client.send_goal_async(goal_msg)
        future.add_done_callback(lambda f: self._after_move_goal(f, target_pose))

    def move_down_and_grip(self, target_pose: PoseStamped):
        """Step 3: move down 7.5cm then close gripper"""
        self.get_logger().info("⬇️ Moving down 7.5 cm in Z...")
        lowered_pose = PoseStamped()
        lowered_pose.header = target_pose.header
        lowered_pose.pose = target_pose.pose
        lowered_pose.pose.position.z -= 0.06 # go 7.5 cm down

        goal_msg = self._make_move_goal(lowered_pose)
        self._move_client.wait_for_server()
        future = self._move_client.send_goal_async(goal_msg)
        future.add_done_callback(lambda f: self._after_move_goal(f, lowered_pose, final_step=True))

    def _make_move_goal(self, target_pose: PoseStamped):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "arm"
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.num_planning_attempts = 5

        pc = PositionConstraint()
        pc.header = target_pose.header
        pc.link_name = "end_effector_link"
        pc.target_point_offset.x = 0.0
        pc.target_point_offset.y = 0.0
        pc.target_point_offset.z = 0.0

        solid = SolidPrimitive()
        solid.type = SolidPrimitive.BOX
        solid.dimensions = [0.01, 0.01, 0.01]

        bv = BoundingVolume()
        bv.primitives.append(solid)
        bv.primitive_poses.append(target_pose.pose)

        pc.constraint_region = bv
        pc.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pc)
        goal_msg.request.goal_constraints.append(constraints)
        return goal_msg

    def _after_move_goal(self, future, pose, final_step=False):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("❌ MoveIt goal rejected")
            return
        self.get_logger().info("✅ MoveIt goal accepted")
        result_future = handle.get_result_async()
        result_future.add_done_callback(lambda r: self._after_move_result(r, pose, final_step))

    def _after_move_result(self, future, pose, final_step):
        result = future.result().result
        self.get_logger().info(f"📦 MoveIt result: error_code={result.error_code}")
        if result.error_code.val == 1:  # SUCCESS
            if not final_step:
                # Step 3: after reaching target pose, go down after 2 sec
                self._delayed(2.0, lambda: self.move_down_and_grip(pose))
            else:
                # Step 4: close gripper after 2 sec
                self._delayed(2.0, lambda: self.close_gripper())

    # ------------------ Helper ------------------
    def _delayed(self, delay, cb):
        """Run callback after delay seconds"""
        def wrapper():
            cb()
            timer.cancel()
        timer = self.create_timer(delay, wrapper)


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

