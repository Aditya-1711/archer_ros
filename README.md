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

### 3. Launch the AI Core
Open a **second** PowerShell terminal and run:
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
