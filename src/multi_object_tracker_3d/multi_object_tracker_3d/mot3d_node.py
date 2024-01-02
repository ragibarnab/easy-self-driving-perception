import rclpy
from rclpy.node import Node


class MultiObjectTracker3DNode(Node):

    def __init__(self):
        super().__init__('multi_object_tracker_3d_node')

        


def main(args=None):
    rclpy.init(args=args)

    mot3d_node = MultiObjectTracker3DNode()
    rclpy.spin(mot3d_node)

    mot3d_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
