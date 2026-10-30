#!/usr/bin/python3

import os

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker


class MapMeshPublisher(Node):
    """Pubblica in RViz la stessa mesh del mondo caricata in Gazebo."""

    def __init__(self):
        super().__init__('map_mesh_publisher')

        default_mesh = os.path.join(
            get_package_share_directory('autosweep_gazebo'),
            'models', 'coverage1_map', 'meshes', 'visual.obj',
        )
        self.declare_parameter('mesh_path', default_mesh)
        self.declare_parameter('frame_id', 'map')

        mesh_path = str(self.get_parameter('mesh_path').value)
        if not os.path.isfile(mesh_path):
            self.get_logger().error(f'Mesh not found: {mesh_path}')

        # Transient local: RViz riceve il marker anche se si avvia dopo.
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._publisher = self.create_publisher(Marker, '/map_mesh', qos)

        marker = Marker()
        marker.header.frame_id = str(self.get_parameter('frame_id').value)
        marker.ns = 'map_mesh'
        marker.id = 0
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.mesh_resource = f'file://{mesh_path}'
        # Usa la texture referenziata dal file .mtl, come fa Gazebo.
        marker.mesh_use_embedded_materials = True
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        # Colore nullo: con materiali embedded RViz usa quelli della mesh.
        marker.color.a = 0.0
        marker.frame_locked = True

        self._publisher.publish(marker)
        self.get_logger().info(f'Published map mesh {mesh_path}')


def main(args=None):
    rclpy.init(args=args)
    node = MapMeshPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
