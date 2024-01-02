import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import ros2_numpy as rnp
import numpy as np
from mmdet3d.apis import init_model, inference_detector


class LidarObjectDetector3DNode(Node):

    def __init__(self):
        super().__init__('lidar_object_detector_3d_node')

        # declare parameters
        self.declare_parameter("config_path", "")
        self.declare_parameter("checkpoint_path", "")
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("translate_height", 0.0)

        # get parameters
        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        checkpoint_path = self.get_parameter("checkpoint_path").get_parameter_value().string_value
        device = self.get_parameter("device").get_parameter_value().string_value


        self.model = init_model(config=config_path, checkpoint=checkpoint_path, device=device)

        self.lidar_subscription = self.create_subscription(
            msg_type = PointCloud2,
            topic = 'lidar',
            callback = self.lidar_callback, 
            qos_profile = 10
        )

        # self.objects_publisher = self.create_publisher(
        #     Object3DArray, 
        #     'objdet3d_raw', 
        #     10
        # )

    def lidar_callback(self, lidar_msg: PointCloud2):
        pcd = rnp.numpify(lidar_msg)
        pcd = np.array([ pcd['x'].flatten(), pcd['y'].flatten(), pcd['z'].flatten(), pcd['reflectivity'].flatten()]).T
        pcd[:, 3] /= 255.0 

        results, _ = inference_detector(model=self.model, pcds=pcd)
        bbox_corners = results.pred_instances_3d.bboxes_3d.corners
        labels = results.pred_instances_3d.labels_3d
        scores = results.pred_instances_3d.scores_3d
        return


def main(args=None):
    rclpy.init(args=args)

    lidar_objdet3d_node = LidarObjectDetector3DNode()
    rclpy.spin(lidar_objdet3d_node)

    lidar_objdet3d_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()