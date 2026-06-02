import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
import time

def main():
    rclpy.init()
    node = Node('tf_inspector')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    
    print("Spinning to collect TF frames...")
    start = time.time()
    while time.time() - start < 3.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        
    print("TF Frames (YAML format):")
    try:
        print(buffer.all_frames_as_yaml())
    except Exception as e:
        print("Error getting frames:", e)
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
