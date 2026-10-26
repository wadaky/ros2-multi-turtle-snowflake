#!/usr/bin/env python3

import math
import random
import threading
import time

import rclpy

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from turtlesim.srv import (
    Spawn,
    TeleportAbsolute,
    SetPen
)


class TurtleAgent:

    def __init__(self, node, name):

        self.node = node
        self.name = name

        self.pose = None
        self.pose_lock = threading.Lock()

        self.pub = node.create_publisher(
            Twist,
            f"/{name}/cmd_vel",
            10
        )

        node.create_subscription(
            Pose,
            f"/{name}/pose",
            self.pose_callback,
            10
        )

        self.tp_client = node.create_client(
            TeleportAbsolute,
            f"/{name}/teleport_absolute"
        )

        self.pen_client = node.create_client(
            SetPen,
            f"/{name}/set_pen"
        )

    def pose_callback(self, msg):

        with self.pose_lock:
            self.pose = msg

    def wait_pose(self):

        while rclpy.ok():

            with self.pose_lock:
                if self.pose is not None:
                    return

            time.sleep(0.05)

    def normalize(self, angle):

        while angle > math.pi:
            angle -= 2 * math.pi

        while angle < -math.pi:
            angle += 2 * math.pi

        return angle

    def set_pen(self, width):

        while not self.pen_client.wait_for_service(
            timeout_sec=1.0
        ):
            pass

        req = SetPen.Request()

        req.r = 255
        req.g = 255
        req.b = 255
        req.width = width
        req.off = 0

        future = self.pen_client.call_async(req)

        while not future.done():
            time.sleep(0.01)

    def teleport(self, x, y):

        while not self.tp_client.wait_for_service(
            timeout_sec=1.0
        ):
            pass

        req = TeleportAbsolute.Request()

        req.x = float(x)
        req.y = float(y)
        req.theta = 0.0

        future = self.tp_client.call_async(req)

        while not future.done():
            time.sleep(0.01)

    def stop(self):

        self.pub.publish(
            Twist()
        )

    def goto(self, tx, ty):

        while rclpy.ok():

            with self.pose_lock:
                pose = self.pose

            if pose is None:
                time.sleep(0.01)
                continue

            dx = tx - pose.x
            dy = ty - pose.y

            dist = math.hypot(dx, dy)

            if dist < 0.05:
                break

            target = math.atan2(
                dy,
                dx
            )

            err = self.normalize(
                target - pose.theta
            )

            cmd = Twist()

            cmd.angular.z = err * 6.0

            if abs(err) < 0.3:
                cmd.linear.x = min(
                    dist * 2.0,
                    3.0
                )

            self.pub.publish(cmd)

            time.sleep(0.03)

        self.stop()

    def draw_line(self, start, end):

        self.teleport(
            start[0],
            start[1]
        )

        self.goto(
            end[0],
            end[1]
        )


class MultiTurtleCrystal(Node):

    def __init__(self):

        super().__init__(
            "multi_turtle_crystal"
        )

        self.turtles = {}

    def spawn_turtles(self):

        spawn = self.create_client(
            Spawn,
            "/spawn"
        )

        while not spawn.wait_for_service(
            timeout_sec=1.0
        ):
            self.get_logger().info(
                "waiting spawn service..."
            )

        self.turtles["turtle1"] = (
            TurtleAgent(
                self,
                "turtle1"
            )
        )

        for i in range(2, 7):

            req = Spawn.Request()

            req.x = 5.5
            req.y = 5.5
            req.theta = 0.0
            req.name = f"turtle{i}"

            future = spawn.call_async(req)

            while not future.done():
                time.sleep(0.01)

            self.turtles[
                f"turtle{i}"
            ] = TurtleAgent(
                self,
                f"turtle{i}"
            )

        self.get_logger().info(
            "all turtles spawned"
        )

        for turtle in self.turtles.values():

            turtle.wait_pose()
            turtle.set_pen(2)

    def genome(self):

        return {

            "main":
                random.uniform(
                    2.0,
                    2.8
                ),

            "branch1":
                random.uniform(
                    0.8,
                    1.4
                ),

            "branch2":
                random.uniform(
                    0.4,
                    0.8
                ),

            "branch3":
                random.uniform(
                    0.2,
                    0.4
                ),

            "angle1":
                random.uniform(
                    math.radians(20),
                    math.radians(40)
                ),

            "angle2":
                random.uniform(
                    math.radians(10),
                    math.radians(25)
                )
        }

    def draw_arm(
        self,
        turtle,
        angle,
        g,
        final
    ):

        cx = 5.5
        cy = 5.5

        x1 = cx + (
            g["main"] *
            math.cos(angle)
        )

        y1 = cy + (
            g["main"] *
            math.sin(angle)
        )

        turtle.draw_line(
            (cx, cy),
            (x1, y1)
        )

        for sign in (+1, -1):

            a1 = angle + sign * g["angle1"]

            bx = x1 - (
                g["branch1"] *
                math.cos(a1)
            )

            by = y1 - (
                g["branch1"] *
                math.sin(a1)
            )

            turtle.draw_line(
                (x1, y1),
                (bx, by)
            )

            a2 = a1 + sign * g["angle2"]

            tx = bx - (
                g["branch2"] *
                math.cos(a2)
            )

            ty = by - (
                g["branch2"] *
                math.sin(a2)
            )

            turtle.draw_line(
                (bx, by),
                (tx, ty)
            )

            if final:

                a3 = a2 + sign * 0.4

                ex = tx - (
                    g["branch3"] *
                    math.cos(a3)
                )

                ey = ty - (
                    g["branch3"] *
                    math.sin(a3)
                )

                turtle.draw_line(
                    (tx, ty),
                    (ex, ey)
                )

    def draw_crystal(
        self,
        genome,
        final=False
    ):

        workers = []

        for arm in range(6):

            turtle = self.turtles[
                f"turtle{arm+1}"
            ]

            angle = (
                arm *
                math.pi /
                3
            )

            worker = threading.Thread(
                target=self.draw_arm,
                args=(
                    turtle,
                    angle,
                    genome,
                    final
                )
            )

            worker.start()

            workers.append(
                worker
            )

        for worker in workers:
            worker.join()

    def run(self):

        self.spawn_turtles()

        best = self.genome()

        for generation in range(5):

            self.get_logger().info(
                f"Generation={generation}"
            )

            self.draw_crystal(
                best,
                generation == 4
            )


def main(args=None):

    rclpy.init(args=args)

    node = MultiTurtleCrystal()

    executor = MultiThreadedExecutor(
        num_threads=16
    )

    executor.add_node(node)

    spin_thread = threading.Thread(
        target=executor.spin,
        daemon=True
    )

    spin_thread.start()

    try:

        node.run()

    finally:

        executor.shutdown()

        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":
    main()
