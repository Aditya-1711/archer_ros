import cuvslam
import numpy as np
fx, fy = np.float64(100.0), np.float64(100.0)
cx, cy = np.float64(320.0), np.float64(240.0)
pose = cuvslam.Pose(rotation=[0.0,0.0,0.0,1.0], translation=[0.0,0.0,0.0])
dist = cuvslam.Distortion(model=cuvslam.Distortion.Pinhole, parameters=[])
try:
    c = cuvslam.Camera(focal=(fx, fy), principal=(cx, cy), distortion=dist, size=(640, 480), rig_from_camera=pose)
    print("Success with np.float64")
except Exception as e:
    print("Failed with np.float64:")
    print(e)

try:
    c = cuvslam.Camera(focal=(float(fx), float(fy)), principal=(float(cx), float(cy)), distortion=dist, size=(640, 480), rig_from_camera=pose)
    print("Success with built-in float")
except Exception as e:
    print("Failed with built-in float:")
    print(e)
