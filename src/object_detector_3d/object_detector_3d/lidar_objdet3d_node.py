import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import ros2_numpy as rnp
import numpy as np
from mmdet3d.apis import init_model, inference_detector
from self_driving_msgs.msg import Object3d, Object3dArray
from geometry_msgs.msg import Point


CLASS2LABEL = {
    'Pedestrian': 0, 
    'Cyclist': 1, 
    'Car': 2        
}


class LidarObjectDetector3DNode(Node):

    def __init__(self):
        super().__init__('lidar_object_detector_3d_node')

        # declare parameters
        self.declare_parameter("config_path", "/home/rae384/workdir/easy-self-driving-perception/data/configs/pointpillars_hv_secfpn_8xb6-160e_kitti-3d-3class.py")
        self.declare_parameter("checkpoint_path", "/home/rae384/workdir/easy-self-driving-perception/data/checkpoints/hv_pointpillars_secfpn_6x8_160e_kitti-3d-3class_20220301_150306-37dc2420.pth")
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

        self.objects_publisher = self.create_publisher(
            Object3dArray, 
            'objdet3d_raw', 
            10
        ) 

    def lidar_callback(self, lidar_msg: PointCloud2):
        pcd = rnp.numpify(lidar_msg)
        pcd = np.array([ pcd['x'].flatten(), pcd['y'].flatten(), pcd['z'].flatten(), pcd['r'].flatten()]).T

        results, _ = inference_detector(model=self.model, pcds=pcd)
        bounding_boxes = results.pred_instances_3d.bboxes_3d
        bbox_corners = results.pred_instances_3d.bboxes_3d.corners.cpu().numpy()
        labels = results.pred_instances_3d.labels_3d
        scores = results.pred_instances_3d.scores_3d

        object_instance_array = Object3dArray()
        for bounding_box, corners, label, score in zip(bounding_boxes, bbox_corners, labels, scores):
            object_instance = Object3d()

            x, y, z, x_size, y_size, z_size, yaw = bounding_box
            object_instance.bounding_box.coordinates[0] = float(x)
            object_instance.bounding_box.coordinates[1] = float(y)
            object_instance.bounding_box.coordinates[2] = float(z)
            object_instance.bounding_box.coordinates[3] = float(x_size)
            object_instance.bounding_box.coordinates[4] = float(y_size)
            object_instance.bounding_box.coordinates[5] = float(z_size)
            object_instance.bounding_box.coordinates[6] = float(yaw)

            x0y0z0, x0y0z1, x0y1z1, x0y1z0, x1y0z0, x1y0z1, x1y1z1, x1y1z0 = corners
            object_instance.bounding_box.corners[3] = Point(x=float(x0y0z0[0]), y=float(x0y0z0[1]), z=float(x0y0z0[2]))
            object_instance.bounding_box.corners[2] = Point(x=float(x0y0z1[0]), y=float(x0y0z1[1]), z=float(x0y0z1[2]))
            object_instance.bounding_box.corners[1] = Point(x=float(x0y1z1[0]), y=float(x0y1z1[1]), z=float(x0y1z1[2]))
            object_instance.bounding_box.corners[0] = Point(x=float(x0y1z0[0]), y=float(x0y1z0[1]), z=float(x0y1z0[2]))
            object_instance.bounding_box.corners[7] = Point(x=float(x1y0z0[0]), y=float(x1y0z0[1]), z=float(x1y0z0[2]))
            object_instance.bounding_box.corners[6] = Point(x=float(x1y0z1[0]), y=float(x1y0z1[1]), z=float(x1y0z1[2]))
            object_instance.bounding_box.corners[5] = Point(x=float(x1y1z1[0]), y=float(x1y1z1[1]), z=float(x1y1z1[2]))
            object_instance.bounding_box.corners[4] = Point(x=float(x1y1z0[0]), y=float(x1y1z0[1]), z=float(x1y1z0[2]))

            object_instance.label = CLASS2LABEL[self.model.dataset_meta['classes'][label]]
            object_instance.confidence_score = float(score)
            object_instance_array.objects.append(object_instance)

        object_instance_array.header.stamp = lidar_msg.header.stamp
        object_instance_array.header.frame_id = lidar_msg.header.frame_id
        self.objects_publisher.publish(object_instance_array)   
    

def main(args=None):
    rclpy.init(args=args)

    lidar_objdet3d_node = LidarObjectDetector3DNode()
    rclpy.spin(lidar_objdet3d_node)

    lidar_objdet3d_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()