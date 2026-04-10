python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
import random


class SpawnTargetNode(Node):

    def __init__(self):
        super().__init__('spawn_target')

        # Client du service /spawn
        self.spawn_client = self.create_client(Spawn, '/spawn')

        # Attendre que le service soit disponible
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /spawn non disponible, attente...')

        self.get_logger().info('Service /spawn disponible. Spawning de turtle_target...')

        # Générer des coordonnées aléatoires (x et y entre 1.0 et 10.0)
        x = random.uniform(1.0, 10.0)
        y = random.uniform(1.0, 10.0)
        theta = random.uniform(-3.14, 3.14)   # angle aléatoire (optionnel)

        # Créer la requête
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = 'turtle_target'        # Nom exact demandé

        # Appel asynchrone du service
        future = self.spawn_client.call_async(request)
        future.add_done_callback(self.spawn_callback)

    def spawn_callback(self, future):
        try:
            response = future.result()
            if response.name:
                self.get_logger().info(f'Turtle cible spawnée avec succès !')
                self.get_logger().info(f'Position → x = {response.x:.2f}, y = {response.y:.2f}, theta = {response.theta:.2f}')
                self.get_logger().info(f'Nom de la tortue : {response.name}')
            else:
                self.get_logger().error('Échec du spawn de la tortue cible.')
        except Exception as e:
            self.get_logger().error(f'Erreur lors du spawn : {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = SpawnTargetNode()

    try:
        rclpy.spin_once(node)   # On spin une seule fois car le nœud fait juste un spawn
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

