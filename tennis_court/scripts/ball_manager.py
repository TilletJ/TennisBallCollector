#!/usr/bin/env python3

import os
import rclpy
import random
import xacro
from geometry_msgs.msg import Pose, Point, Twist, Vector3
from rclpy.clock import Clock
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, HistoryPolicy, ReliabilityPolicy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SpawnEntity, DeleteEntity, SetEntityState
from ament_index_python import get_package_share_directory
from tennis_court.msg import BallManagerStats, GameStatus


class BallManager(Node):

    DELETE_BALL_DURATION = Duration(seconds=5.0)
    SPAWN_BALL_DURATION = 20.0
    TOTAL_BALL_COUNT = 10

    GAME_STARTED_DURATION = 30.0
    GAME_STARTING_DURATION = 30.0
    GAME_PAUSED_DURATION = 120.0

    MODEL_NAMES = [
        "zenith_camera",
        "tennis_court",
        "player_1",
        "player_1_collision",
        "player_2",
        "player_2_collision"
    ]

    def __init__(self):
        super().__init__("ball_manager")
        self.score = 0.0
        self.ball_id = 0
        self.balls = dict()
        self.game_status = 42
        self.next_status_stamp = None
        self.robot_pose = None
        self.robot_model_name = None
        self.ball_description_file = os.path.join(get_package_share_directory("tennis_court"), "urdf", "ball.urdf.xacro")

        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # Ball manager stats publisher
        self.stats_pub = self.create_publisher(BallManagerStats, "/ball_manager_stats", qos_profile)

        # Game status publisher
        self.game_status_pub = self.create_publisher(GameStatus, "/game_status", qos_profile)
        self.game_status_pub.publish(GameStatus(is_started=False))

        # Spawn entity client
        self.spawn_entity_client = self.create_client(SpawnEntity, "/spawn_entity")
        while not self.spawn_entity_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Service '/spawn_entity' not available, waiting...")

        # Delete entity client
        self.delete_entity_client = self.create_client(DeleteEntity, "/delete_entity")
        while not self.delete_entity_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Service '/delete_entity' not available, waiting...")

        # Set entity state client
        self.set_entity_state_client = self.create_client(SetEntityState, "/set_entity_state")
        while not self.set_entity_state_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Service '/set_entity_state' not available, waiting...")

        # Model states subscriber
        self.model_states_sub = self.create_subscription(ModelStates, "/model_states", self.on_model_states, 10)

        # Ball spawner timer
        self.spawn_ball_timer = None

        # Publish stats timer
        self.publish_stats_timer = self.create_timer(0.1, self.publish_stats)

        # Game status timer
        self.game_status_timer = None

        self.get_logger().info("Ball manager initialized")

    def publish_stats(self):
        time_left = 0.0
        if self.next_status_stamp is not None:
            time_left = max(0.0, (self.next_status_stamp - self.get_clock().now()).nanoseconds * 1e-9)
        in_safe_region = True
        if self.robot_pose is not None:
            in_safe_region = self.is_robot_in_safe_region(self.robot_pose.position)
            if self.game_status == BallManagerStats.GAME_STARTED and not in_safe_region:
                self.score -= 0.2
            elif self.game_status in [BallManagerStats.GAME_STARTED, BallManagerStats.GAME_STARTING] and in_safe_region:
                self.score += 0.1
        self.stats_pub.publish(BallManagerStats(
            score=int(round(self.score)),
            current_ball_count=self.get_ball_count(),
            total_ball_count=len(self.balls),
            game_status=self.game_status,
            status_time_left=time_left,
            robot_is_safe=in_safe_region
        ))

    def get_ball_count(self):
        return len([ball for ball in self.balls.values() if ball.status == Ball.STATUS_SPAWNED])

    def spawn_ball(self):
        self.spawn_ball_timer.cancel()

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
        ball.set_spawned(self.get_clock().now())

    def set_robot_model(self, model_name):
        """
        :type model_name: str
        """
        self.get_logger().info(f"Tracking robot {model_name}")
        self.robot_model_name = model_name
        self.game_status = BallManagerStats.GAME_STARTING
        self.next_status_stamp = self.get_clock().now() + Duration(seconds=self.GAME_STARTING_DURATION)
        self.game_status_timer = self.create_timer(self.GAME_STARTING_DURATION, self.game_status_update)
        self.game_status_pub.publish(GameStatus(is_started=True))
        if len(self.balls) < self.TOTAL_BALL_COUNT:
            self.spawn_ball_timer = self.create_timer(self.GAME_STARTING_DURATION * 0.2, self.spawn_ball)

    def game_status_update(self):
        self.game_status_timer.cancel()
        duration = 0.0
        if self.game_status == BallManagerStats.GAME_STARTING: # GAME_STARTED
            self.game_status = BallManagerStats.GAME_STARTED
            duration = self.GAME_STARTED_DURATION
            if len(self.balls) < self.TOTAL_BALL_COUNT:
                self.spawn_ball_timer = self.create_timer(duration * 0.2, self.spawn_ball)
        elif self.game_status == BallManagerStats.GAME_STARTED: # GAME_PAUSED
            self.game_status_pub.publish(GameStatus(is_started=False))
            self.game_status = BallManagerStats.GAME_PAUSED
            duration = self.GAME_PAUSED_DURATION
        elif self.game_status == BallManagerStats.GAME_PAUSED: # GAME_STARTING
            self.game_status_pub.publish(GameStatus(is_started=True))
            self.game_status = BallManagerStats.GAME_STARTING
            duration = self.GAME_STARTING_DURATION
            if len(self.balls) < self.TOTAL_BALL_COUNT:
                self.spawn_ball_timer = self.create_timer(duration * 0.2, self.spawn_ball)
        self.next_status_stamp = self.get_clock().now() + Duration(seconds=duration)
        self.game_status_timer = self.create_timer(duration, self.game_status_update)

    def on_model_states(self, model_states):
        """
        :type model_states: ModelStates
        """
        timestamp = self.get_clock().now()
        for model_name, model_pose in zip(model_states.name, model_states.pose):
            if not model_name.startswith("ball"):
                if self.robot_model_name is None and model_name not in self.MODEL_NAMES:
                    self.set_robot_model(model_name)
                if model_name == self.robot_model_name:
                    self.robot_pose = model_pose
                continue
            ball_id = int(model_name[4:])
            ball = self.balls[ball_id]
            if ball.status != Ball.STATUS_SPAWNED:
                continue
            in_region = self.is_ball_in_gathering_region(model_pose.position)
            if in_region and not ball.in_region:
                self.get_logger().info(f"Ball {ball_id} entered region")
            if not in_region and ball.in_region:
                self.get_logger().info(f"Ball {ball_id} exited region")
            region_time = ball.get_region_time(in_region, timestamp)
            if region_time > self.DELETE_BALL_DURATION:
                self.delete_ball(ball_id)

    def delete_ball(self, ball_id):
        """
        :type ball_id: int
        """
        ball = self.balls[ball_id]
        self.delete_entity_client.call_async(DeleteEntity.Request(
            name=ball.name
        ))
        ball.set_destroyed(self.get_clock().now())
        self.score += self.compute_score(ball.get_lifespan())
        self.get_logger().info(f"Ball {ball_id} deleted")
        if self.get_ball_count() == 0 and len(self.balls) == self.TOTAL_BALL_COUNT:
            self.get_logger().info(f"End of simulation, congratulations :)")
            self.destroy_node()

    def compute_score(self, lifespan):
        """
        :type lifespan: Duration
        """
        now = self.get_clock().now().nanoseconds * 1e-9
        lifespan_sec = lifespan.nanoseconds * 1e-9
        global_score = int(10000.0 / now)
        ball_score = int(20000.0 / lifespan_sec)
        self.get_logger().info(f"Took {round(now, 3)} seconds to collect ball with a lifespan of {round(lifespan_sec, 3)} seconds. " +
                               f"Got global time score of {global_score} and ball score of {ball_score}")
        return float(global_score + ball_score)

    @staticmethod
    def is_ball_in_gathering_region(point):
        """
        :type point: Point
        """
        region_size = Point(x=2.224250, y=2.631040, z=2.0)
        region_1_center = Point(x=-6.85, y=-13.65, z=1.0)
        region_2_center = Point(x=6.85, y=13.65, z=1.0)
        if region_1_center.x - region_size.x / 2.0 <= point.x <= region_1_center.x + region_size.x / 2.0 and \
            region_1_center.y - region_size.y / 2.0 <= point.y <= region_1_center.y + region_size.y / 2.0 and \
            region_1_center.z - region_size.z / 2.0 <= point.z <= region_1_center.z + region_size.z / 2.0:
            return True
        if region_2_center.x - region_size.x / 2.0 <= point.x <= region_2_center.x + region_size.x / 2.0 and \
            region_2_center.y - region_size.y / 2.0 <= point.y <= region_2_center.y + region_size.y / 2.0 and \
            region_2_center.z - region_size.z / 2.0 <= point.z <= region_2_center.z + region_size.z / 2.0:
            return True
        return False

    @staticmethod
    def is_robot_in_safe_region(point):
        """
        :type point: Point
        """
        region_size = Point(x=2.281020, y=21.2145, z=4.0)
        region_1_center = Point(x=-6.85, y=0.0, z=1.0)
        region_2_center = Point(x=6.85, y=0.0, z=1.0)
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
        """
        :type ball_id: int
        """
        self.id = ball_id
        self._status = self.STATUS_NONE
        self.spawned_time = None
        self.destroyed_time = None
        self.enter_region_time = None
        self.in_region = False
        self.initial_pose = self._get_initial_pose()
        self.initial_velocity = self._get_initial_velocity()

    def set_destroyed(self, timestamp):
        """
        :type timestamp: Clock
        """
        self._status = self.STATUS_DESTROYED
        self.destroyed_time = timestamp

    def set_spawned(self, timestamp):
        """
        :type timestamp: Clock
        """
        self._status = self.STATUS_SPAWNED
        self.spawned_time = timestamp

    def get_lifespan(self):
        if self.spawned_time is None or self.destroyed_time is None:
            return None
        return self.destroyed_time - self.spawned_time

    def get_region_time(self, in_region, timestamp):
        """
        :type in_region: bool
        :type timestamp: Clock
        """
        self.in_region = in_region
        if self.enter_region_time is None and in_region:
            self.enter_region_time = timestamp
        if not self.in_region and self.enter_region_time is not None:
            self.enter_region_time = None
        return timestamp - self.enter_region_time if self.enter_region_time is not None else Duration()

    @property
    def status(self):
        return self._status

    @property
    def name(self):
        return f"ball{self.id}"

    def _get_initial_pose(self):
        x = -4.0 + random.random() * 8.0
        y = -12.0 + random.random() * 4.0
        if random.random() > 0.5:
            y = -y
        z = 1.0 + random.random()
        return Pose(position=Point(x=x, y=y, z=z))

    def _get_initial_velocity(self):
        x = - random.random() * 5.0 - 2.0 if self.initial_pose.position.y < 0 else random.random() * 5.0 + 2.0
        y = -18.0 + random.random() * 10.0 if self.initial_pose.position.y > 0 else 18.0 - random.random() * 10.0
        z = 1.0 + random.random() * 5.0
        return Twist(linear=Vector3(x=x, y=y, z=z))


def main(args=None):
    random.seed(22)

    rclpy.init(args=args)
    ball_manager = BallManager()
    rclpy.spin(ball_manager)
    ball_manager.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
