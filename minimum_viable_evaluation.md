# Minimum Viable Evaluation

## 1. Feature Evaluation Summary

| Feature / Subsystem | Evaluation Metric | Result / Status | Notes |
| :--- | :--- | :--- | :--- |
| **Autonomous Navigation (Nav2)** | Goal reach success rate | Pass (80%+) | Robot successfully navigates to random waypoints while avoiding obstacles. |
| **Exploration Algorithm** | State-space coverage | Pass | Memory-based frontier exploration prevents infinite loops in visited regions. |
| **Object Detection (YOLOv8)** | Inference accuracy | Pass | Accurately identifies standard household objects in the Gazebo environment. |
| **Voice Command Processing**| Intent recognition rate | Pass | Successfully parses natural language into actionable ROS 2 coordinate goals. |
| **Semantic Mapping** | Landmark persistence | Pass | Detected objects are correctly anchored to the global costmap coordinates. |
| **System Architecture** | Real-time orchestration | Pass | Bridge node reliably coordinates asynchronous ROS 2 and Gazebo subsystems. |

## 2. Quantitative Results

* **Successful Navigation Trials:** 12 out of 15 trials (80% success rate) completed without manual teleoperation or recovery timeouts.
* **Mean Completion Time:** 42.5 seconds per navigation waypoint (for an average traversal distance of 4.5 meters).
* **Recognised Voice Commands:** 28 out of 30 voice intents correctly parsed and translated into ROS 2 actions (93% accuracy).
* **Detection Count Accuracy:** 92% classification accuracy on a small test set of 50 simulated object instances (using YOLOv8 nano).
* **End-to-End Latency Estimate:** ~850 ms total delay from raw voice command input to initial wheel actuation (`cmd_vel` generation).
