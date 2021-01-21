#!/usr/bin/env python3

import os
import rclpy
import random

import xacro
from geometry_msgs.msg import Pose, Point, Twist, Vector3
from rclpy.clock import ROSClock
from rclpy.duration import Duration
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SpawnEntity, DeleteEntity, SetEntityState
from ament_index_python import get_package_share_directory
from rclpy.time_source import TimeSource


class BallManager(Node):

    def __init__(self):
        super().__init__("ball_manager")
        self.ball_id = 0
        self.balls = dict()
        self.ball_description_file = os.path.join(get_package_share_directory("tennis_court"), "urdf", "ball.urdf.xacro")

        self.clock = ROSClock()
        self.time_source = TimeSource(node=self)

        # Spawn entity client
        self.spawn_entity_client = self.create_client(SpawnEntity, "/spawn_entity")
        while not self.spawn_entity_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service '/spawn_entity' not available, waiting...")

        # Delete entity client
        self.delete_entity_client = self.create_client(DeleteEntity, "/delete_entity")
        while not self.delete_entity_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service '/delete_entity' not available, waiting...")

        # Set entity state client
        self.set_entity_state_client = self.create_client(SetEntityState, "/set_entity_state")
        while not self.set_entity_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service '/set_entity_state' not available, waiting...")

        # Model states subscriber
        self.model_states_sub = self.create_subscription(ModelStates, "/model_states", self.on_model_states, 10)

        # Ball spawner timer
        self.timer = self.create_timer(5.0, self.spawn_ball)

    def spawn_ball(self):
        self.timer.cancel()

        self.ball_id += 1
        ball = Ball(self.ball_id)
        self.balls[ball.id] = ball

        xacro_mappings = {
            "vel_x": str(ball.initial_velocity.linear.x),
            "vel_y": str(ball.initial_velocity.linear.y),
            "vel_z": str(ball.initial_velocity.linear.z)
        }
        urdf_description = xacro.process_file(self.ball_description_file, mappings=xacro_mappings).toxml()
        with open("/tmp/test.urdf", "w") as s:
            s.write(urdf_description)
        self.spawn_entity_client.call_async(SpawnEntity.Request(
            name=ball.name,
            xml=urdf_description,
            initial_pose=ball.initial_pose
        ))
        self.get_logger().info(f"Ball {ball.id} spawned")
        ball.status = Ball.STATUS_SPAWNED

        self.timer = self.create_timer(5.0, self.spawn_ball)

    def on_model_states(self, model_states):
        """
        :type model_states: ModelStates
        """
        timestamp = self.get_clock().now()
        for model_name, model_pose in zip(model_states.name, model_states.pose):
            if not model_name.startswith("ball"):
                continue
            ball_id = int(model_name[4:])
            ball = self.balls[ball_id]
            if ball.status != Ball.STATUS_SPAWNED:
                continue
            in_region = self.is_in_region(model_pose.position)
            if in_region and not ball.in_region:
                self.get_logger().info(f"Ball {ball_id} entered region")
            if not in_region and ball.in_region:
                self.get_logger().info(f"Ball {ball_id} exited region")
            region_time = ball.get_region_time(in_region, timestamp)
            if region_time > Duration(seconds=5.0):
                self.delete_ball(ball_id)

    def delete_ball(self, ball_id):
        ball = self.balls[ball_id]
        self.delete_entity_client.call_async(DeleteEntity.Request(
            name=ball.name
        ))
        ball.status = Ball.STATUS_DESTROYED
        self.get_logger().info(f"Ball {ball_id} deleted")

    def is_in_region(self, point):
        """
        :type point: Point
        """
        region_size = Point(x=2.224250, y=2.631040, z=2.0)
        region_1_center = Point(x=-6.807575, y=-13.601261, z=1.0)
        region_2_center = Point(x=6.807575, y=13.601261, z=1.0)
        if region_1_center.x - region_size.x / 2.0 <= point.x <= region_1_center.x + region_size.x / 2.0 and \
            region_1_center.y - region_size.y / 2.0 <= point.y <= region_1_center.y + region_size.y / 2.0 and \
            region_1_center.z - region_size.z / 2.0 <= point.z <= region_1_center.z + region_size.z / 2.0:
            return True
        if region_2_center.x - region_size.x / 2.0 <= point.x <= region_2_center.x + region_size.x / 2.0 and \
            region_2_center.y - region_size.y / 2.0 <= point.y <= region_2_center.y + region_size.y / 2.0 and \
            region_2_center.z - region_size.z / 2.0 <= point.z <= region_2_center.z + region_size.z / 2.0:
            return True
        return False


class Ball(object):

    STATUS_NONE = 0
    STATUS_SPAWNED = 1
    STATUS_DESTROYED = 2

    def __init__(self, ball_id):
        self.id = ball_id
        self.status = self.STATUS_NONE
        self.in_region = False
        self.enter_region_time = None
        self.initial_pose = self._get_initial_pose()
        self.initial_velocity = self._get_initial_velocity()

    def get_region_time(self, in_region, timestamp):
        self.in_region = in_region
        if self.enter_region_time is None and in_region:
            self.enter_region_time = timestamp
        if not self.in_region and self.enter_region_time is not None:
            self.enter_region_time = None
        return timestamp - self.enter_region_time if self.enter_region_time is not None else Duration()

    @property
    def name(self):
        return f"ball{self.id}"

    def _get_initial_pose(self):
        x = -5.0 + random.random() * 10.0
        y = -12.0 + random.random() * 4.0
        if random.random() > 0.5:
            y = -y
        z = 1.0 + random.random()
        return Pose(position=Point(x=x, y=y, z=z))

    def _get_initial_velocity(self):
        x = -6.0 + random.random() * 12.0
        y = -20.0 + random.random() * 10.0 if self.initial_pose.position.y > 0 else 20.0 - random.random() * 10.0
        z = random.random() * 5.0
        return Twist(linear=Vector3(x=x, y=y, z=z))


def main(args=None):
    rclpy.init(args=args)
    ball_spawner = BallManager()

    while rclpy.ok():
        rclpy.spin_once(ball_spawner)

    ball_spawner.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
