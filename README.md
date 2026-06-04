# A.R.C.H.E.R. ROS 2 Humanoid Simulation

Archer is an **Advanced Relationship & Command Handling Entity (Humanoid)** running on a professional ROS 2 Jazzy stack, Gazebo Harmonic, and a local host-based AI Neuro-Core. It is a ROS 2-powered humanoid robot simulation that integrates navigation, vision, and a local AI assistant for natural language control.

---

## Host Dependencies Installation
Before launching the AI Core, you need to install the required Python packages on the host machine (Windows, Linux, or WSL2):
```bash
pip install -r requirements.txt
```
*Note: This installs `openai-whisper` for STT, `requests` for Ollama LLM integration, and other core libraries. Sound and audio playback dependencies are included.*

### Prerequisites
1. **Ollama**: Download and install Ollama. Run `ollama pull llama3.2:1b` 
2. **Piper TTS**: If using text-to-speech, download a voice model ONNX file and update `piper.model_path` in [settings.yaml].

---

## Project Structure
- **`ai/`**: Modular AI engine subsystems:
  - `llm/`: Interface to local Llama models.
  - `memory/`: Long-term SQLite database and FAISS vector index managers.
  - `parser/`: Natural language command intent parsers.
  - `stt/` & `tts/`: Whisper speech-to-text and Piper text-to-speech engine drivers.
- **`core/`**: Orchestration brain (`main.py` main loop) coordination.
- **`config/`**: Central parameter settings (`settings.yaml`).
- **`dashboard/`**: Tactical control panel web server and client UI.
- **`ros2_ws/src/`**: ROS 2 packages:
  - `archer_bridge/`: Communication bridge node, safety supervisor, watchdog, power manager, and pointcloud utilities.
  - `archer_description/`: Humanoid robot URDF/xacro, simulation worlds, meshes, and launch scripts.
  - `archer_yolo/`: YOLOv8 object detection wrapper node.
- **`simulation/`**: Gazebo world assets and RViz inspection configurations.

---


## Requirements
- Python 3.12
- ROS 2 Jazzy
- Gazebo Harmonic
- Docker (optional)
- Ollama (local LLM runtime)
- WSL2 or Ubuntu version 24.04

---

## Features
- Natural language control via local LLM (Ollama)
- Autonomous navigation using Nav2
- Real-time object detection (YOLOv8)
- Voice interaction (Whisper STT + Piper TTS)
- Long-term memory with FAISS + SQLite
- Interactive web dashboard for control & monitoring


## Quick Start Guide

### Option A: Docker (Recommended)
1. Ensure Docker Desktop is running.
2. Run:
   ```powershell
   docker compose up ros2
   ```
   *Note: Set `ENABLE_GUI=true` in `.env` to see Gazebo.*

### Option B: Native WSL2 or Ubuntu 24.04
1. **Build**:
   ```bash
   cd ros2_ws
   source /opt/ros/jazzy/setup.bash
   colcon build --packages-select archer_description archer_bridge
   source install/setup.bash
   ```
2. **Launch**:
   ```bash
   ros2 launch archer_description sim.launch.py gui:=true use_slam:=true use_nav2:=true
   ```

## AI Integration
Archer's AI system (`core/main.py`) communicates via the shared volume `simulation/last_cmd.yaml` and the bridged `/cmd_vel` topic.


*   **Gazebo Harmonic** will open showing Archer in the house.
*   **RViz** will open showing Archer's "AI Vision" (Camera feed & LiDAR map).

### 3. Launch the Web Control Dashboard (NEW)
Open a **second** terminal and run:
```powershell
python dashboard/server.py
```
This launches our high-performance **Tactical Web Dashboard** at `http://localhost:8080`!

Through this premium glassmorphism interface, you can:
*   **Interact with Llama**: Send commands to the AI core and view responses.
*   **2D Real-time Map**: Watch Archer's precise coordinate position ($x, y$) and yaw heading update dynamically on a floorplan vector radar.
*   **Tactile D-Pad**: Manually drive Archer around the map with safe, auto-stopping velocity commands.
*   **Speed Limits**: Choose between Slow, Normal, Fast, and Full speed settings.
*   **Quick Spatial Nav**: Single-click on any room card (Kitchen, Bedroom, Living Room) to route the humanoid instantly!

### 4. Launch the AI CLI Core (Alternative)
If you prefer a text terminal command line:
```powershell
# Start the Archer Brain
python core/main.py --cli
```
Wait for Archer to say: *"Neural uplink established. Archer online and standing by, Boss."*

---

## Interacting with Archer

Once the AI Core is running, you can give commands in plain English:

*   **Navigation**: `"Archer, please go to the kitchen."` or `"Head to the bathroom."`
*   **Motion**: `"Move forward 2 meters."` or `"Rotate 90 degrees to the left."`
*   **Vision**: Check the top-left HUD in RViz to see what Archer sees through his head camera.

---

## 🛠️ Troubleshooting

| Issue | Solution |
| :--- | :--- |
| **RViz / Gazebo not opening** | Ensure VcXsrv/XLaunch is running with **"Disable access control"** checked. |
| **"Could not connect to display"** | Your Windows Firewall may be blocking the connection. Allow VcXsrv through the firewall. |
| **Robot not moving** | Ensure the **AI Core** is running. Direct `nav_goal` commands require Archer to "see" first; try a `rotate` command to wake him up. |
| **Simulation Lag** | Archer is optimized for Real-Time Factor (RTF) 1.0. If it's slow, close background browser tabs to free up CPU. |

---

## MoSCoW Analysis
Here is the status of the prioritized system requirements:

### Must Have (100% Implemented)
*   **Indoor 2D SLAM**: Configured via `slam_toolbox` in online asynchronous mode to generate maps from 2D LiDAR data.
*   **RGB Camera**: Equipped with an RGBD camera sensor publishing raw image feeds to `/camera/image_raw` (or `/archer/camera/image_raw`).
*   **Multi-sensor pipeline**: 2D LiDAR, IMU, Odometry, and Camera sensors bridged from Gazebo to ROS 2 via `ros_gz_bridge`.
*   **Navigation (Nav2)**: Navigation 2 stack (`nav2_bringup`) runs path planning, costmap clearance, and goal routing.
*   **AI ↔ ROS 2 bridge**: [bridge_node.py] reads parsed actions from the Host AI gateway and maps them to ROS 2 velocity publishers or Nav2 action goals.
*   **Local LLM orchestration**: Master control loop [main.py] integrates Whisper (Speech-to-Text), Ollama (Local Llama 3.2), and Piper (Text-to-Speech).

### Should Have (100% Implemented)
*   **Sensor fusion**: Uses `robot_localization` with an EKF (Extended Kalman Filter) node to fuse raw `/odom` and `/imu` data.
*   **Simple planner**: AI responses are sliced into sequential execution queues (e.g. *"move forward and then rotate left"* commands are executed sequentially).
*   **Basic human interaction**: Bridge node monitors battery percentage and CPU/Core temperature telemetry, and the AI Core responds to hardware queries.
*   **Simple task execution**: Command execution queue managed through `_action_queue` in the bridge node.
*   **3D metric mapping**: Pointcloud feed bridged to `octomap_server` to generate 3D Voxel grids for collision checks.

### Could Have (Modified Implementation)
*   **Semantic mapping**: *Kimera* was omitted to keep the stack fully local and light. Instead, the bridge node uses bounding boxes defined in [locations.json] to dynamically tag the robot's real-time coordinate position (e.g. matching coordinate `[2.0, -6.5, 0.0]` to `"kitchen"`).
*   **Memory system**: Archer implements a hybrid vectorized memory system (SQLite database + FAISS index) that stores conversations, user preferences, and visual tracking history.
*   **Hierarchical planning**: Prompt directives instruct the LLM to expand high-level instructions (e.g., *"Patrol the house"*) into step-by-step target coordinates.

### Would Have (100% Implemented)
*   **Learning by demonstration**: Archer supports Natural Language Macros. You can teach Archer custom sequences by saying *"Learn routine Alpha: move forward. rotate left. stop."* to save them to `routines.json`, and run them by saying *"Execute routine Alpha"*.

---

## Core Architecture Details
Archer utilizes several advanced host-side and ROS 2 services to function safely and intelligently:

### 1. Vectorized Long-Term Memory (FAISS + SQLite)
Archer's memory manager retrieves historical context using a hybrid architecture:
- **FAISS (Facebook AI Similarity Search)** performs high-speed vector queries using 384-dimensional embeddings (via a local ONNX `all-MiniLM-L6-v2` transformer).
- **SQLite Database** (`ai/memory/db/memory.db`) tracks metadata and visual observations.
- **Dynamic Relevance Scoring**: Retrieval ranks memory records by a combined weight formula:
  $$S = 0.5 \cdot \text{Similarity} + 0.3 \cdot \text{Importance} + 0.2 \cdot \text{Recency}$$
- **Exponential Recency Decay**: Time-based decay factor: $R(\Delta t) = e^{-0.02 \cdot \Delta t}$ reduces focus on stale data over time.

### 2. Node Watchdog & Safety Limits
To prevent physical collisions or command lock-up, the `watchdog_node.py` enforces a safety heartbeat check:
- **Safety Supervisor Node** ($0.5\text{s}$ timeout): Triggers `EMERGENCY_STOP`, killing physical joints and velocity outputs.
- **Bridge Node** ($1.0\text{s}$ timeout): Zeroes velocity outputs immediately (`SAFE_STOP`).
- **AI Core** ($5.0\text{s}$ timeout): Decelerates the humanoid and maintains safety locks.

### 3. Battery Management & Docking Verification
The `power_manager_node.py` runs a simulated battery discharge cycle. When low battery is detected or the user commands docking:
1. The robot routes to the charging dock coordinate `[0.0, 0.0, 0.0]`.
2. The power manager verifies alignment (distance $< 0.25\text{m}$).
3. Verification checks charger contact pins and confirms charging current.
4. If docking verification fails (e.g. bad contact), the robot reverses 0.6 meters, waits, and retries up to 3 times before halting and alerting the user.

### 4. Vision Pipeline (`archer_yolo`)
- The `yolo_node.py` processes raw camera frames using YOLOv8 via ONNX.
- Detected objects, labels, and OCR text are published on `/archer/vision/detections`.
- The `vision_node.py` handles rate-limited logging, updating SQLite visual memory when frame content changes or after 10.0 seconds of constant monitoring.
