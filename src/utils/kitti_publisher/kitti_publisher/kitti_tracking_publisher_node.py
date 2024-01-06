import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
import ros2_numpy as rnp
import numpy as np
from numpy.lib.recfunctions import unstructured_to_structured
from .kitti_dataset import KittiTrackingDataset
import time


class KittiTrackingPublisherNode(Node):
    def __init__(self):
        super().__init__('kitti_tracking_publisher_node')

        # declare parameters
        self.declare_parameter("tracking_path", "/home/rae384/data/kitti/tracking/training/")
        self.declare_parameter("sequence_id", 0)

        # get parameters
        tracking_path = self.get_parameter("tracking_path").get_parameter_value().string_value
        seq_id = self.get_parameter("sequence_id").get_parameter_value().integer_value

        self.dataset = KittiTrackingDataset(root_path=tracking_path, seq_id=seq_id, load_points=True, load_image=True) 
        self.timestep = 0
        time.sleep(5)   # sleep 5 seconds until all nodes are launched and set up
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.lidar_publisher = self.create_publisher(PointCloud2, 'lidar', 10)
        self.image_publisher = self.create_publisher(Image, 'image', 10)

    
    def timer_callback(self):
        P2, V2C, points, image, objects, det_scores, pose = self.dataset[self.timestep]

        dtypes = np.dtype([('x', np.float32), ('y', np.float32), ('z', np.float32), ('r', np.float32)])
        point_cloud_numpy = unstructured_to_structured(arr=points, dtype=dtypes)
        
        lidar_msg: PointCloud2 = rnp.msgify(PointCloud2, point_cloud_numpy)
        image_msg: Image = rnp.msgify(Image, image, encoding='bgr8')
        
        current_stamp = self.get_clock().now().to_msg()

        lidar_msg.header.stamp = current_stamp
        lidar_msg.header.frame_id = 'velodyne'
        image_msg.header.stamp = current_stamp
        image_msg.header.frame_id = 'camera'
        
        self.lidar_publisher.publish(lidar_msg)
        self.image_publisher.publish(image_msg)
        
        self.timestep += 1
        if self.timestep == len(self.dataset):  # recycle the sequence
            self.timestep = 0


def main(args=None):
    rclpy.init(args=args)

    kitti_tracking_publisher_node = KittiTrackingPublisherNode()
    rclpy.spin(kitti_tracking_publisher_node)

    kitti_tracking_publisher_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()