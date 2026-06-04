# A.R.C.H.E.R. ROS 2 Humanoid Simulation

#### Option A: Docker (Portable)
Archer is an Advanced Relationship & Command Handling Entity (Humanoid) running on a professional ROS 2 Jazzy stack and Gazebo Harmonic.

## Project Structure (Lab 2 Refactor)
- **`core/`**: AI Core (Persona, Voice, MCP Tools).
- **`ros2_ws/src/archer_description/`**: Unified description package.
  - `urdf/`: Xacro robot models.
  - `launch/`: Simulation and inspection launch files.
  - `config/`: Bridge and world configurations.
  - `meshes/`: 3D assets.
- **`ros2_ws/src/archer_bridge/`**: Communication layer between AI and ROS.

## Dependencies
- **OS**: Windows 11 (with WSL2) or Ubuntu 24.04
- **ROS 2**: Jazzy Jalisco
- **Simulation**: Gazebo Harmonic
- **AI Core**: Python 3.12, `openai-whisper` (Local STT), `ollama` (Local LLM), `piper-tts` (Local TTS)
- **Computer Vision**: `ultralytics` (YOLOv8)
- **Other Python Packages**: `rclpy`, `sounddevice`, `numpy`, `PyYAML`, `requests`

## MoSCoW Requirements Checklist
- [x] **Must Have**: Autonomous ROS 2 Nav2 navigation, Local LLM integration, Speech-to-Text capability.
- [x] **Should Have**: Gazebo Harmonic simulation, Web-based Tactical Dashboard, YOLOv8 Object Detection.
- [x] **Could Have**: Memory-based Frontier Exploration, Semantic Mapping.
- [ ] **Won't Have**: Physical hardware deployment, Cloud-based LLM APIs (due to GDPR constraints).

## Quick Start Guide

### Option A: Docker (Recommended)
1. Ensure Docker Desktop is running.
2. Run:
   ```powershell
   docker compose up ros2
   ```
   *Note: Set `ENABLE_GUI=true` in `.env` to see Gazebo.*

### Option B: Native WSL2
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

## 🗣️ Interacting with Archer

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

## 📂 Project Structure
*   `core/`: Archer's LLM, TTS, and Command Processing logic.
*   `docker/`: Orchestration and ROS 2 environment.
*   `ros2_ws/`: Archer's physical humanoid definition (URDF) and Bridge Node.
*   `simulation/`: Gazebo worlds, RViz configs, and shared command volumes.
