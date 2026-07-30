import sys
from sensor_msgs.msg import CameraInfo
import cuvslam
import numpy as np

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

    print(f"DEBUG _build_camera: fx_type={type(fx)}, cx_type={type(cx)}, rig_type={type(rig_from_camera)}")
    try:
        return cuvslam.Camera(
            focal=(float(fx), float(fy)),
            principal=(float(cx), float(cy)),
            distortion=distortion,
            size=(int(info.width), int(info.height)),
            rig_from_camera=rig_from_camera,
        )
    except Exception as e:
        print(f"DEBUG _build_camera EXCEPTION: {e}")
        raise

info = CameraInfo()
info.width = 640
info.height = 480
info.k = [100.0, 0.0, 320.0, 0.0, 100.0, 240.0, 0.0, 0.0, 1.0]
info.d = [0.0, 0.0, 0.0, 0.0, 0.0]

pose = cuvslam.Pose(rotation=[0.0,0.0,0.0,1.0], translation=[0.0,0.0,0.0])
print('Testing _build_camera')
cam = _build_camera(info, pose)
print('Success')
