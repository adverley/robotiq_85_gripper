"""

 Author: Thomas Lips
 Brief:  Action server for fake Robotiq 85 gripper communication.
 Platform: Linux/ROS2
"""

from math import fabs

import rclpy
from control_msgs.action import GripperCommand
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState


class FakeGripper:
    """
    Fake Robotiq gripper state with maximal step size
    """

    def __init__(self, max_step_size):
        self.max_step_size = max_step_size
        self.desired_position = 0.0
        self.position = 0.0

        self.min_position = 0.0  # open
        self.max_position = 0.8

    def step(self):
        step = self.desired_position - self.position
        if abs(step) > self.max_step_size:
            step = step / abs(step) * self.max_step_size

        self.position += step
        self._clamp_position()

    def _clamp_position(self):
        self.position = max(min(self.position, self.max_position), self.min_position)


class Robotiq85ActionServer(Node):
    def __init__(self):
        super().__init__("fake_robotiq_85_action_server")

        self.declare_parameter("gripper_speed", 0.05)

        self._gripper_speed = self.get_parameter("gripper_speed").get_parameter_value().double_value
        self._joint_pub = self.create_publisher(JointState, "/joint_states", 10)

        self._cb_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            GripperCommand,
            "/robotiq_gripper_controller/gripper_cmd",
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            execute_callback=self._execute_callback,
            callback_group=self._cb_group,
        )

        self.gripper = FakeGripper(self._gripper_speed)

        self.create_timer(0.05, self._publish_joint)
        self.get_logger().info("Gripper server ready")

    def get_time(self):
        time_msg = self.get_clock().now().to_msg()
        return float(time_msg.sec) + (float(time_msg.nanosec) * 1e-9)

    def shutdown(self):
        self.get_logger().debug("Shutdown gripper")
        self._gripper.shutdown()

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def _publish_joint(self):
        js = JointState()
        js.header.frame_id = ""
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ["robotiq_85_left_knuckle_joint"]
        js.position = [self.gripper.position]
        self._joint_pub.publish(js)

    def _goal_callback(self, goal_request):
        self.get_logger().debug("Gripper received goal request")
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.get_logger().debug("Gripper received cancel request")
        self.gripper.desired_position = self.gripper.position
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle):
        self.get_logger().debug("Gripper executing goal...")

        # Approximately convert angular joint position at knuckle to linear distance between jaws
        self.get_logger().debug(" desired Angle: " + str(goal_handle.request.command.position))
        # Send goal to gripper
        self.gripper.desired_position = goal_handle.request.command.position

        rate = self.create_rate(20)  # simulated control frequency of 20Hz

        while rclpy.ok():
            # update fake gripper state
            self.gripper.step()

            feedback_msg = GripperCommand.Feedback()
            feedback_msg.position = self.gripper.position
            feedback_msg.stalled = False

            if fabs(self.gripper.desired_position - feedback_msg.position) < 0.01:
                feedback_msg.reached_goal = True
                self.get_logger().info("Goal achieved: %r" % feedback_msg.reached_goal)

            goal_handle.publish_feedback(feedback_msg)

            if feedback_msg.reached_goal:
                self.get_logger().debug("Reached goal, exiting loop")
                break

            # sleep, also serves as simulated control frequency.
            rate.sleep()

        result_msg = GripperCommand.Result()
        result_msg.reached_goal = feedback_msg.reached_goal
        result_msg.stalled = feedback_msg.stalled
        result_msg.position = feedback_msg.position
        result_msg.effort = feedback_msg.effort

        if result_msg.reached_goal:
            self.get_logger().debug("Setting action to succeeded")
            goal_handle.succeed()
        else:
            self.get_logger().debug("Setting action to abort")
            goal_handle.abort()

        return result_msg


def main(args=None):
    rclpy.init(args=args)

    action_server = Robotiq85ActionServer()

    executor = MultiThreadedExecutor()
    rclpy.spin(action_server, executor=executor)
    action_server.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
