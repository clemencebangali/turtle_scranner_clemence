#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from turtle_interfaces.srv import ResetMission
from turtlesim.srv import Spawn, Kill
import math
import random


class TurtleScannerNode(Node):

    def __init__(self):
        super().__init__('turtle_scanner_node')

        # === Attributs de position ===
        self.pose_scanner = None
        self.pose_target = None

        # === Paramètres du balayage serpentin ===
        self.nb_lignes = 5
        self.y_start = 1.0
        self.y_step = 2.0
        self.x_min = 1.0
        self.x_max = 10.0
        self.linear_speed_max = 2.0
        self.angular_speed_max = 1.5
        self.waypoint_tolerance = 0.3
        self.detection_radius = 1.5

        # Gains (à ajuster)
        self.declare_parameter('Kp_ang', 3.0)
        self.declare_parameter('Kp_lin', 1.0)
        self.Kp_ang = self.get_parameter('Kp_ang').value
        self.Kp_lin = self.get_parameter('Kp_lin').value

        # Liste des waypoints + index courant
        self.waypoints = self.generate_serpent_waypoints()
        self.current_waypoint_idx = 0
        self.is_scanning = True
        self.target_detected = False

        # === Subscribers ===
        self.create_subscription(Pose, '/turtle1/pose', self.pose_scanner_callback, 10)
        self.create_subscription(Pose, '/turtle_target/pose', self.pose_target_callback, 10)

        # === Publishers ===
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.detected_pub = self.create_publisher(Bool, '/target_detected', 10)

        # === Timer principal (≈ 20 Hz) ===
        self.create_timer(0.05, self.scan_step)

        # === Service Reset Mission ===
        self.reset_service = self.create_service(ResetMission, '/reset_mission', self.reset_mission_callback)

        # Clients pour kill et spawn
        self.kill_client = self.create_client(Kill, '/kill')
        self.spawn_client = self.create_client(Spawn, '/spawn')

        self.get_logger().info('Turtle Scanner Node démarré - Balayage serpentin en cours...')

    def generate_serpent_waypoints(self):
        """Génère la liste des waypoints en motif serpentin (lignes horizontales alternées)"""
        waypoints = []
        y = self.y_start
        for i in range(self.nb_lignes):
            if i % 2 == 0:
                waypoints.append([self.x_max, y])   # ligne paire → de gauche à droite
            else:
                waypoints.append([self.x_min, y])   # ligne impaire → de droite à gauche
            y += self.y_step
        return waypoints

    def compute_angle(self, A, B):
        """Angle désiré entre deux points A et B"""
        return math.atan2(B[1] - A[1], B[0] - A[0])

    def compute_distance(self, A, B):
        """Distance euclidienne"""
        return math.sqrt((B[0] - A[0])**2 + (B[1] - A[1])**2)

    def pose_scanner_callback(self, msg):
        self.pose_scanner = msg

    def pose_target_callback(self, msg):
        self.pose_target = msg

    def scan_step(self):
        if self.pose_scanner is None or not self.is_scanning:
            self.publish_stop()
            return

        # Détection de la cible
        if self.pose_target is not None:
            dist_to_target = self.compute_distance(
                [self.pose_scanner.x, self.pose_scanner.y],
                [self.pose_target.x, self.pose_target.y]
            )
            if dist_to_target < self.detection_radius:
                self.target_detected = True
                self.is_scanning = False
                self.publish_stop()
                self.detected_pub.publish(Bool(data=True))
                self.get_logger().info(f'Cible détectée à ({self.pose_target.x:.2f}, {self.pose_target.y:.2f}) !')
                return
            else:
                self.target_detected = False
                self.detected_pub.publish(Bool(data=False))

        # Balayage normal
        if self.current_waypoint_idx >= len(self.waypoints):
            self.get_logger().info('Balayage terminé - Tous les waypoints parcourus.')
            self.publish_stop()
            return

        current_wp = self.waypoints[self.current_waypoint_idx]
        scanner_pos = [self.pose_scanner.x, self.pose_scanner.y]

        distance = self.compute_distance(scanner_pos, current_wp)

        if distance < self.waypoint_tolerance:
            self.current_waypoint_idx += 1
            self.get_logger().info(f'Waypoint {self.current_waypoint_idx} atteint.')
            return

        # Calcul des erreurs et commandes
        theta_desired = self.compute_angle(scanner_pos, current_wp)
        heading_error = math.atan(math.tan((theta_desired - self.pose_scanner.theta) / 2))

        msg = Twist()
        msg.angular.z = self.Kp_ang * heading_error
        msg.linear.x = min(self.Kp_lin * distance, self.linear_speed_max)

        self.cmd_vel_pub.publish(msg)

    def publish_stop(self):
        msg = Twist()
        self.cmd_vel_pub.publish(msg)

    def reset_mission_callback(self, request, response):
        """Service de réinitialisation"""
        try:
            # Kill l'ancienne turtle_target
            kill_req = Kill.Request()
            kill_req.name = 'turtle_target'
            self.kill_client.call_async(kill_req)

            # Spawn nouvelle cible
            spawn_req = Spawn.Request()
            if request.random_target:
                spawn_req.x = random.uniform(1.0, 10.0)
                spawn_req.y = random.uniform(1.0, 10.0)
            else:
                spawn_req.x = request.target_x
                spawn_req.y = request.target_y
            spawn_req.theta = 0.0
            spawn_req.name = 'turtle_target'

            self.spawn_client.call_async(spawn_req)

            # Réinitialiser le balayage
            self.waypoints = self.generate_serpent_waypoints()
            self.current_waypoint_idx = 0
            self.is_scanning = True
            self.target_detected = False

            response.success = True
            response.message = f'Mission réinitialisée - Nouvelle cible spawnée à ({spawn_req.x:.2f}, {spawn_req.y:.2f})'
            self.get_logger().info(response.message)

        except Exception as e:
            response.success = False
            response.message = f'Erreur reset: {str(e)}'
            self.get_logger().error(response.message)

        return response


def main(args=None):
    rclpy.init(args=args)
    node = TurtleScannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
