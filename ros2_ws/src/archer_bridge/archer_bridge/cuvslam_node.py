#!/usr/bin/env python3
"""
cuvslam_node.py — NVIDIA cuVSLAM Visual Odometry as a ROS2 node
================================================================
Uses PyCuVSLAM (pip-installable wheel, no Isaac ROS apt required).

Install:
    Go to https://github.com/nvidia-isaac/cuVSLAM/releases
    Download the wheel for: cu12, cp312, linux_x86_64
    pip install cuvslam-*.whl

Subscribed topics:
    /archer/camera/left/image_raw   [sensor_msgs/Image]
    /archer/camera/left/camera_info [sensor_msgs/CameraInfo]
    /archer/camera/right/image_raw  [sensor_msgs/Image]
    /archer/camera/right/camera_info[sensor_msgs/CameraInfo]
    /imu                            [sensor_msgs/Imu]

Published topics:
    /visual_slam/tracking/odometry  [nav_msgs/Odometry]

Published TF:
    odom → base_link  (visual odometry, high-frequency)
"""

import math
import threading
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import Image, CameraInfo, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
from message_filters import ApproximateTimeSynchronizer, Subscriber

# ── PyCuVSLAM import guard ─────────────────────────────────────────────────────
try:
    import cuvslam
    CUVSLAM_AVAILABLE = True
except ImportError:
    CUVSLAM_AVAILABLE = False


# ── Helpers ────────────────────────────────────────────────────────────────────

def _ros_img_to_gray(msg: Image) -> np.ndarray:
    """Convert a ROS2 Image message to a uint8 grayscale numpy array."""
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    h, w = msg.height, msg.width

    enc = msg.encoding.lower()
    if enc in ('mono8', '8uc1'):
        return raw.reshape((h, w))
    elif enc in ('rgb8',):
        rgb = raw.reshape((h, w, 3))
        # ITU-R BT.601 luma
        return (0.299 * rgb[:, :, 0] + 0.587 * rgb[:, :, 1] + 0.114 * rgb[:, :, 2]).astype(np.uint8)
    elif enc in ('bgr8',):
        bgr = raw.reshape((h, w, 3))
        return (0.114 * bgr[:, :, 0] + 0.587 * bgr[:, :, 1] + 0.299 * bgr[:, :, 2]).astype(np.uint8)
    else:
        raise ValueError(f'Unsupported image encoding: {msg.encoding}')


def _rotation_matrix_to_quaternion(R: np.ndarray):
    """Convert 3×3 rotation matrix to (x, y, z, w) quaternion."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


def _build_camera(info: CameraInfo, rig_from_camera: 'cuvslam.Pose') -> 'cuvslam.Camera':
    """Build a cuvslam.Camera from a ROS CameraInfo message."""
    K = np.array(info.k).reshape(3, 3)
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]

    # Distortion — Gazebo cameras are ideal (zero distortion), use Pinhole
    d = info.d
    if len(d) >= 4 and any(abs(v) > 1e-9 for v in d[:4]):
        distortion = cuvslam.Distortion(
            model=cuvslam.Distortion.Brown,
            parameters=list(d[:5]) + [0.0] * max(0, 5 - len(d)),
        )
    else:
        distortion = cuvslam.Distortion(model=cuvslam.Distortion.Pinhole, parameters=[])

    return cuvslam.Camera(
        focal=(float(fx), float(fy)),
        principal=(float(cx), float(cy)),
        distortion=distortion,
        size=(int(info.width), int(info.height)),
        rig_from_camera=rig_from_camera,
    )


# ── Main Node ──────────────────────────────────────────────────────────────────

class CuVSLAMNode(Node):
    """ROS2 node wrapping PyCuVSLAM for stereo visual odometry."""

    def __init__(self):
        super().__init__('cuvslam_node')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('odom_frame',      'odom')
        self.declare_parameter('base_frame',      'base_link')
        self.declare_parameter('map_frame',       'map')
        self.declare_parameter('warmup_frames',   30)
        self.declare_parameter('jitter_ms',       34.0)

        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._warmup     = self.get_parameter('warmup_frames').value
        self._jitter_ms  = self.get_parameter('jitter_ms').value

        self._last_ts = -1

        if not CUVSLAM_AVAILABLE:
            self.get_logger().error(
                'PyCuVSLAM not installed! '
                'Download the wheel from https://github.com/nvidia-isaac/cuVSLAM/releases '
                'and run: pip install cuvslam-*.whl'
            )
            return

        # ── State ───────────────────────────────────────────────────────────
        self._tracker    = None
        self._rig        = None
        self._frame_idx  = 0
        self._imu_buf: deque = deque(maxlen=200)
        self._lock       = threading.Lock()

        # Camera intrinsics (populated on first CameraInfo message)
        self._left_info:  CameraInfo | None = None
        self._right_info: CameraInfo | None = None

        # ── Publishers ──────────────────────────────────────────────────────
        self._odom_pub = self.create_publisher(Odometry, '/visual_slam/tracking/odometry', 10)
        self._tf_broadcaster = TransformBroadcaster(self)

        # ── Camera info subscribers (use VOLATILE to match ros_gz_bridge) ──────
        # ros_gz_bridge publishes camera_info as VOLATILE; using TRANSIENT_LOCAL
        # here causes a QoS incompatibility and no messages arrive.
        cam_info_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._left_info_sub = self.create_subscription(
            CameraInfo, '/archer/camera/left/camera_info',
            self._left_info_cb, cam_info_qos,
        )
        self._right_info_sub = self.create_subscription(
            CameraInfo, '/archer/camera/right/camera_info',
            self._right_info_cb, cam_info_qos,
        )

        # ── IMU subscriber ───────────────────────────────────────────────────
        self._imu_sub = self.create_subscription(
            Imu, '/imu', self._imu_cb, 100,
        )

        # ── Stereo image synchroniser ────────────────────────────────────────
        sync_qos = QoSProfile(depth=10,
                              reliability=ReliabilityPolicy.BEST_EFFORT,
                              durability=DurabilityPolicy.VOLATILE)
        self._left_sub  = Subscriber(self, Image, '/archer/camera/left/image_raw',  qos_profile=sync_qos)
        self._right_sub = Subscriber(self, Image, '/archer/camera/right/image_raw', qos_profile=sync_qos)

        # Allow up to jitter_ms tolerance between left/right frames
        self._sync = ApproximateTimeSynchronizer(
            [self._left_sub, self._right_sub],
            queue_size=10,
            slop=self._jitter_ms / 1000.0,
        )
        self._sync.registerCallback(self._stereo_cb)

        self.get_logger().info(
            'cuVSLAM node started — waiting for stereo camera_info...'
        )

    # ── Camera info callbacks ──────────────────────────────────────────────────

    def _left_info_cb(self, msg: CameraInfo):
        if self._left_info is None:
            self._left_info = msg
            self.get_logger().info(f'Left camera info received: {msg.width}×{msg.height}')
            self._try_init_tracker()

    def _right_info_cb(self, msg: CameraInfo):
        if self._right_info is None:
            self._right_info = msg
            self.get_logger().info(f'Right camera info received: {msg.width}×{msg.height}')
            self._try_init_tracker()

    def _try_init_tracker(self):
        """Initialise the cuVSLAM tracker once both camera infos are available."""
        if self._tracker is not None:
            return
        if self._left_info is None or self._right_info is None:
            return

        # ── Build camera rig ─────────────────────────────────────────────────
        # Left camera: identity pose in rig frame (reference camera)
        left_pose = cuvslam.Pose(rotation=[0.0, 0.0, 0.0, 1.0], translation=[0.0, 0.0, 0.0])

        # Right camera: 12 cm to the right (-Y in camera frame convention)
        # baseline = 0.12 m  (matches URDF: right_link xyz="0 -0.06 0" relative to left_link xyz="0 0.06 0")
        right_pose = cuvslam.Pose(rotation=[0.0, 0.0, 0.0, 1.0], translation=[0.0, -0.12, 0.0])

        left_cam  = _build_camera(self._left_info,  left_pose)
        right_cam = _build_camera(self._right_info, right_pose)

        imu_calib = cuvslam.ImuCalibration(
            rig_from_imu=cuvslam.Pose(rotation=[0.0, 0.0, 0.0, 1.0], translation=[0.0, 0.0, 0.0]),
            gyroscope_noise_density=2.0e-4,
            gyroscope_random_walk=3.0e-6,
            accelerometer_noise_density=2.0e-3,
            accelerometer_random_walk=1.0e-4,
            frequency=100.0,
        )

        self._rig = cuvslam.Rig(cameras=[left_cam, right_cam], imus=[imu_calib])

        cfg = cuvslam.Tracker.OdometryConfig(
            # Stereo multicamera mode — fastest on GTX 1650
            odometry_mode=cuvslam.Tracker.OdometryMode.Inertial,
            multicam_mode=cuvslam.Tracker.MulticameraMode.Performance,
        )

        self._tracker = cuvslam.Tracker(rig=self._rig, odom_config=cfg)
        self.get_logger().info('cuVSLAM tracker initialised ✓  (stereo, CUDA)')

    # ── IMU callback ───────────────────────────────────────────────────────────

    def _imu_cb(self, msg: Imu):
        """Buffer IMU measurements for feeding into cuVSLAM."""
        ts_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        meas = cuvslam.ImuMeasurement(
            timestamp_ns=ts_ns,
            linear_accelerations=(
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            ),
            angular_velocities=(
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
            ),
        )
        with self._lock:
            self._imu_buf.append(meas)

    # ── Stereo image callback ──────────────────────────────────────────────────

    def _stereo_cb(self, left_msg: Image, right_msg: Image):
        """Synchronised stereo callback — runs cuVSLAM tracking."""
        if self._tracker is None:
            return

        try:
            left_gray  = _ros_img_to_gray(left_msg)
            right_gray = _ros_img_to_gray(right_msg)
        except ValueError as e:
            self.get_logger().warn(f'Image conversion error: {e}')
            return

        # Timestamp from left image header (nanoseconds)
        ts_ns = (left_msg.header.stamp.sec * 1_000_000_000
                 + left_msg.header.stamp.nanosec)

        # Drain buffered IMU measurements up to this timestamp
        with self._lock:
            imu_batch = [m for m in self._imu_buf if m.timestamp_ns <= ts_ns]
            self._imu_buf = deque(
                (m for m in self._imu_buf if m.timestamp_ns > ts_ns),
                maxlen=200,
            )

        # Sort and enforce strict global monotonicity for IMU
        imu_batch.sort(key=lambda m: m.timestamp_ns)
        
        valid_imu_batch = []
        for meas in imu_batch:
            if meas.timestamp_ns <= self._last_ts:
                continue
            
            # Prevent double-precision truncation in PyCuVSLAM (requires ~240ns separation)
            ts = meas.timestamp_ns
            if ts < self._last_ts + 1000:
                ts = self._last_ts + 1000
            
            # Recreate with safe timestamp
            safe_meas = cuvslam.ImuMeasurement(
                timestamp_ns=ts,
                linear_accelerations=meas.linear_accelerations,
                angular_velocities=meas.angular_velocities
            )
            valid_imu_batch.append(safe_meas)
            self._last_ts = ts

        # Feed IMU measurements before this frame
        for meas in valid_imu_batch:
            try:
                self._tracker.register_imu_measurement(0, meas)
            except Exception as e:
                self.get_logger().error(f'IMU Non-monotonic crash! ts={meas.timestamp_ns}, valid_batch={[m.timestamp_ns for m in valid_imu_batch]}: {e}')
                raise

        # Enforce strict global monotonicity for Camera
        if ts_ns < self._last_ts + 1000:
            ts_ns = self._last_ts + 1000
        self._last_ts = ts_ns

        # Track
        try:
            status = self._tracker.track(
                timestamp=ts_ns,
                images=[left_gray, right_gray],
            )
            estimate: cuvslam.PoseEstimate = status[0]
        except Exception as e:
            self.get_logger().warn(f'cuVSLAM tracking error: {e}')
            return

        self._frame_idx += 1
        if self._frame_idx <= self._warmup:
            # Silently skip warmup frames
            return

        self._publish_pose(estimate, left_msg.header.stamp)

    # ── Pose publishing ────────────────────────────────────────────────────────

    def _publish_pose(self, estimate: 'cuvslam.PoseEstimate', stamp):
        pose = getattr(estimate.world_from_rig, 'pose', estimate.world_from_rig)

        t = pose.translation
        qx, qy, qz, qw = pose.rotation

        # ── Publish nav_msgs/Odometry ───────────────────────────────────────
        odom = Odometry()
        odom.header.stamp    = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id  = self._base_frame

        odom.pose.pose.position.x = float(t[0])
        odom.pose.pose.position.y = float(t[1])
        odom.pose.pose.position.z = float(t[2])
        odom.pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        self._odom_pub.publish(odom)

        # ── Broadcast TF: odom → base_link ─────────────────────────────────
        tf_msg = TransformStamped()
        tf_msg.header.stamp    = stamp
        tf_msg.header.frame_id = self._odom_frame
        tf_msg.child_frame_id  = self._base_frame

        tf_msg.transform.translation.x = float(t[0])
        tf_msg.transform.translation.y = float(t[1])
        tf_msg.transform.translation.z = float(t[2])
        tf_msg.transform.rotation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        # self._tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CuVSLAMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
