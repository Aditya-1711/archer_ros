import numpy as np
import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from std_msgs.msg import Header

class DepthToPointCloudNode(Node):
    def __init__(self):
        super().__init__('depth_to_pointcloud')
        self.get_logger().info("Depth to PointCloud converter node starting...")
        
        self.image_sub = self.create_subscription(
            Image,
            '/archer/camera/depth',
            self.depth_callback,
            10
        )
        self.info_sub = self.create_subscription(
            CameraInfo,
            '/archer/camera/camera_info',
            self.info_callback,
            10
        )
        self.pc_pub = self.create_publisher(
            PointCloud2,
            '/archer/camera/depth/points',
            10
        )
        
        self.K = None
        self.width = None
        self.height = None
        self.u_grid = None
        self.v_grid = None
        self.frame_counter = 0

    def info_callback(self, msg):
        if self.K is not None:
            return  # already initialized intrinsics
        
        self.width = msg.width
        self.height = msg.height
        self.K = msg.k  # flat 9-element array: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        
        fx = self.K[0]
        fy = self.K[4]
        cx = self.K[2]
        cy = self.K[5]
        
        self.get_logger().info(f"Initialized intrinsics: {self.width}x{self.height}, fx={fx}, fy={fy}, cx={cx}, cy={cy}")
        
        # Precompute coordinate grids
        u = np.arange(self.width)
        v = np.arange(self.height)
        uu, vv = np.meshgrid(u, v)
        
        self.u_grid = uu.astype(np.float32)
        self.v_grid = vv.astype(np.float32)

    def depth_callback(self, msg):
        if self.K is None:
            return
            
        self.frame_counter += 1
        if self.frame_counter % 3 != 0:
            return
            
        # Convert depth data to numpy array
        # In Gazebo, float depth image has encoding '32FC1' (4 bytes per pixel float)
        if msg.encoding != '32FC1':
            return
            
        depth_data_full = np.frombuffer(msg.data, dtype=np.float32).reshape((self.height, self.width))
        
        # Downsample the grids by taking every 4th pixel horizontally and vertically.
        # This reduces the number of points by 16x, drastically saving CPU processing
        # and making OctoMap 3D mapping extremely fast and lightweight.
        STEP = 4
        depth_data = depth_data_full[::STEP, ::STEP]
        u_grid = self.u_grid[::STEP, ::STEP]
        v_grid = self.v_grid[::STEP, ::STEP]
        
        # Filter invalid depth values
        # Invalid values can be NaN, Inf, or out of range (like <= 0.1 or > 10.0)
        valid_mask = np.isfinite(depth_data) & (depth_data > 0.1) & (depth_data < 10.0)
        
        if not np.any(valid_mask):
            return
            
        # Extract parameters
        fx = self.K[0]
        fy = self.K[4]
        cx = self.K[2]
        cy = self.K[5]
        
        z_opt = depth_data[valid_mask]
        x_opt = (u_grid[valid_mask] - cx) * z_opt / fx
        y_opt = (v_grid[valid_mask] - cy) * z_opt / fy
        
        # Convert from Camera Optical Frame (x_opt=right, y_opt=down, z_opt=forward)
        # to standard ROS Coordinate Frame (x=forward, y=left, z=up) for 'Head' frame_id
        x_std = z_opt
        y_std = -x_opt
        z_std = -y_opt
        
        # Stack coordinates to form N x 3 array in standard ROS frame
        points = np.stack((x_std, y_std, z_std), axis=-1).astype(np.float32)
        
        # Create PointCloud2 message
        pc_msg = PointCloud2()
        pc_msg.header = msg.header
        pc_msg.height = 1
        pc_msg.width = len(points)
        pc_msg.is_dense = True
        pc_msg.is_bigendian = False
        pc_msg.point_step = 12
        pc_msg.row_step = pc_msg.point_step * pc_msg.width
        
        # Define fields (x, y, z as float32)
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        pc_msg.fields = fields
        
        # Pack data using memoryview / tobytes() for high-speed conversion
        pc_msg.data = points.tobytes()
        
        self.pc_pub.publish(pc_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DepthToPointCloudNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
