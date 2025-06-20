import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
import numpy as np

class TimeFieldAdder(Node):
    def __init__(self):
        super().__init__('add_time_field_node')
        self.sub = self.create_subscription(
            PointCloud2,
            '/points_raw',  # your input topic
            self.callback,
            10
        )
        self.pub = self.create_publisher(
            PointCloud2,
            '/points',  # output topic  
            10
        )
        self.scan_period = 0.1  # adjust based on your lidar's scan rate (e.g., 10Hz)

    def callback(self, msg: PointCloud2):
        points = list(pc2.read_points(msg, field_names=["x", "y", "z", "intensity", "ring"], skip_nans=True))
        num_points = len(points)

        fields = msg.fields.copy()
        fields.append(PointField(
            name="time",
            offset=msg.point_step,
            datatype=PointField.FLOAT32,
            count=1 
        ))

        point_step = msg.point_step + 4
        data = []

        for i, point in enumerate(points):
            relative_time = float(i) / num_points * self.scan_period
            data.append((*point, relative_time))

        new_msg = pc2.create_cloud(
            msg.header,
            fields,
            data
        )
        self.pub.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TimeFieldAdder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
