
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose


class TurtleScannerNode(Node):

    def __init__(self):
        super().__init__('turtle_scanner_node')

        # Attributs pour stocker les positions (initialisés à None)
        self.pose_scanner = None   # Position de la tortue scanner (turtle1)
        self.pose_target = None    # Position de la tortue cible (turtle_target)

        # Subscriber pour la tortue scanner (/turtle1/pose)
        self.sub_scanner = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.callback_scanner,
            10
        )

        # Subscriber pour la tortue target (/turtle_target/pose)
        self.sub_target = self.create_subscription(
            Pose,
            '/turtle_target/pose',
            self.callback_target,
            10
        )

        self.get_logger().info('Turtle Scanner Node démarré - Attente des positions des deux tortues...')

    def callback_scanner(self, msg: Pose):
        """Mise à jour de la position de la tortue scanner (turtle1)"""
        self.pose_scanner = msg
        # Optionnel : afficher de temps en temps pour debug
        # self.get_logger().info(f'Pose Scanner → x={msg.x:.2f}, y={msg.y:.2f}, theta={msg.theta:.2f}')

    def callback_target(self, msg: Pose):
        """Mise à jour de la position de la tortue target"""
        self.pose_target = msg
        # self.get_logger().info(f'Pose Target  → x={msg.x:.2f}, y={msg.y:.2f}, theta={msg.theta:.2f}')


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

