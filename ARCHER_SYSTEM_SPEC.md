# ARCHER: Advanced Relationship & Command Handling Entity
## Integrated ROS2 System Specification (v2.1.1 - Refinement Update)

Archer is a fully local, high-performance robotic AI assistant designed for humanoid simulation, autonomous navigation, conversational control, vision processing, long-term memory retrieval, safety-critical supervisors, and intelligent power management.

---

## 1. System Architecture
The system follows a decoupled "Brain-to-Body" architecture, bridging a high-level Python AI Core on the Windows host with a containerized ROS2 Jazzy Robotics Stack in WSL2/Docker.

```mermaid
graph TD
    User((User Input)) -->|CLI/Voice| AI_Core[AI Neuro-Core]
    AI_Core -->|REST API| Ollama[Llama 3.2:1b]
    AI_Core -->|Subprocess| Piper[Piper TTS]
    
    %% Host-Side Memory System (FAISS & SQLite)
    AI_Core <-->|Query/Write| Mem_Mgr[Memory Manager]
    Mem_Mgr <-->|SQL Client| SQL_DB[(SQLite DB: memory.db)]
    Mem_Mgr <-->|FAISS Client| FAISS[(FAISS Index: memory.index)]
    Mem_Mgr -->|Context Injection| Prompt_Inj[Memory Injection Layer]
    Prompt_Inj -->|Enriched Prompt| AI_Core
    
    %% Host-Side Heartbeat Daemon
    AI_Core -->|Periodic Write| HostHB[ai_heartbeat.json]
    AI_Core -->|Frictionless Command Gateway| PubScript[pub_cmd.py / last_cmd.yaml]
    
    subgraph ROS2_Jazzy_Environment
        Bridge[Archer Bridge Node] -->|Reads cmd| PubScript
        Bridge -->|Reads heartbeat| HostHB
        
        %% Velocity routing via safety
        Bridge -->|Raw Velocity: /archer/cmd_vel_raw| SafetyNode[Safety Supervisor Node]
        SafetyNode -->|Governed Velocity: /cmd_vel| Gazebo[Gazebo Harmonic]
        Bridge -->|Action: /nav2| Nav2[Navigation Stack]
        
        %% Vision Pipeline & Storage
        Cam[Head Camera] -->|Topic: /camera/image_raw| VisionNode[Archer Vision Node]
        VisionNode -->|YOLOv8 ONNX / OpenCV| VisionNode
        VisionNode -->|Topic: /archer/vision/detections| Bridge
        VisionNode -->|OCR & Tracking| VisionNode
        VisionNode -->|Direct SQL Write| SQL_DB
        
        %% Safety & Lidar
        Lidar[Lidar Sensor] -->|Topic: /scan| SafetyNode
        Odom[Odometry] -->|Topic: /odom| SafetyNode
        TF[Transforms] -->|Topic: /tf| SafetyNode
        
        %% Power Subsystem & Diagnostics
        PowerNode[Power Manager Node] -->|Discharge Model| Gazebo
        PowerNode -->|Service: /archer/power/dock| Nav2
        Odom -->|Verification Alignment| PowerNode
        
        %% System Watchdog Supervisor
        Watchdog[System Watchdog Node] -->|Emergency Twist| Gazebo
        Watchdog -->|Estop Service Call| SafetyNode
        
        %% Heartbeats Published to Watchdog
        Bridge -->|Topic: /archer/heartbeat/bridge| Watchdog
        Bridge -->|Topic: /archer/heartbeat/ai_core| Watchdog
        VisionNode -->|Topic: /archer/heartbeat/vision| Watchdog
        SafetyNode -->|Topic: /archer/heartbeat/safety| Watchdog
        PowerNode -->|Topic: /archer/heartbeat/power| Watchdog
        
        %% Health telemetry outputs
        Bridge -->|Active Goals| Status[robot_status.json]
        Watchdog -->|Health state| Status
        PowerNode -->|System Metrics| Diag[diagnostics.json]
    end
    
    Status -->|Prompt Injection| AI_Core
    Diag -->|Prompt Injection| AI_Core
```

---

## 2. Core AI Specifications
The AI layer runs on the Windows host to ensure direct access to microphone, speakers, and local model servers.

| Subsystem | Component | Detail |
| :--- | :--- | :--- |
| **LLM (Brain)** | Ollama | Model: `llama3.2:1b` (optimized for local latency) |
| **STT (Ears)** | Whisper | Model: `base` (openai-whisper) |
| **TTS (Voice)** | Piper | Engine: ONNX Neural Voice (`en_US-lessac-medium`) |
| **Parser** | Custom Regex | Structured JSON intent extraction with safety clamping |
| **Memory Manager** | SQLite + FAISS | Local persistent memory database combined with FAISS dense indexing |
| **Embedding Engine** | Local ONNX | Sentence-transformer (`all-MiniLM-L6-v2`) for query vectorization |
| **Persona** | Archer | Dry, precise, Stark Industries "Friday" persona |

---

## 3. Robotics & Simulation (WSL2/Docker)
The robotics layer is containerized in **Ubuntu 24.04 (Jazzy)**.

*   **Middleware**: ROS2 Jazzy Jalisco
*   **Simulator**: Gazebo Harmonic (GZ Sim)
*   **Robot Model**: Archer Humanoid V2 (Modular Xacro)
*   **Slam**: `slam_toolbox` (Online Asynchronous)
*   **Navigation**: `nav2_bringup` / Navigation2 Stack
*   **Safety Supervisor**: Standalone C++ node implementing real-time obstacle avoidance and sensor validation.
*   **System Watchdog**: Active heartbeat supervisor enforcing emergency stop triggers.
*   **Vision Subsystem**: ONNX Runtime-powered YOLOv8 node running on the camera stream.

---

## 4. Networking & Topic Manifest
Standardized communication channels using `ros_gz_bridge`.

| Topic / Action / Service | Type | Direction | Purpose |
| :--- | :--- | :--- | :--- |
| `/archer/command` | `std_msgs/String` | AI -> Bridge | JSON command packets |
| `/archer/cmd_vel_raw` | `geometry_msgs/Twist` | Bridge -> Safety | Raw velocity before safety checks |
| `/cmd_vel` | `geometry_msgs/Twist` | Safety/WD -> Sim | Governed velocity control |
| `/goal_pose` | `geometry_msgs/PoseStamped` | Bridge/Power -> Nav2 | Target navigation goals |
| `/odom` | `nav_msgs/Odometry` | Sim -> Bridge/Safety/Power | Filtered EKF position data |
| `/scan` | `sensor_msgs/LaserScan` | Sim -> Nav2/Safety | Obstacle range detection |
| `/tf` | `tf2_msgs/TFMessage` | Sim -> ROS | Coordinate transforms |
| `/camera/image_raw` | `sensor_msgs/Image` | Sim -> Vision | Head camera video stream |
| `/archer/vision/detections`| `vision_msgs/Detection2DArray` | Vision -> Bridge | Detected objects, labels, and coordinates |
| `/archer/vision/ocr` | `std_msgs/String` | Vision -> Bridge | Extracted text from labels or documents |
| `/sensor/battery` | `sensor_msgs/BatteryState` | Battery -> Power | Simulated battery status metrics |
| `/archer/safety/state` | `archer_msgs/SafetyState` | Safety -> User/AI | Current active safety state |
| `/archer/heartbeat/*` | `std_msgs/Header` | Node -> Watchdog | Periodic heartbeat signals |
| `/archer/safety/estop` [Srv] | `std_srvs/Trigger` | UI/WD -> Safety | Emergency stop activation |
| `/archer/safety/reset` [Srv] | `std_srvs/Trigger` | UI/AI -> Safety | Reset from Emergency Stop |
| `/archer/power/dock` [Srv] | `std_srvs/Trigger` | AI/Power -> Nav2 | Auto-return to charging dock |

---

## 5. Location Semantic Map
Archer uses a real-to-virtual coordinate mapping for semantic awareness.

| Friendly Name | Coordinates [X, Y, Z] | Type |
| :--- | :--- | :--- |
| **origin / dock** | `[0.0, 0.0, 0.0]` | Spawn Point & Charger |
| **living_room** | `[0.0, 1.0, 0.0]` | Goal Node |
| **kitchen** | `[2.0, -6.5, 0.0]` | Goal Node |
| **bedroom** | `[7.0, 9.0, 0.0]` | Goal Node |
| **garage** | `[-7.5, 5.0, 0.0]` | Goal Node |

---

## 6. Directory Structure
```text
archer_ros/
├── ai/                # LLM, STT, and TTS engines
│   ├── llm/           # Ollama client interfaces
│   ├── memory/        # Long-Term Memory manager and retrieval
│   │   ├── db/        # SQLite database (memory.db) and FAISS index
│   │   └── memory_manager.py
│   ├── parser/        # Natural language command parsers
│   ├── stt/           # Whisper speech-to-text configurations
│   └── tts/           # Piper text-to-speech configurations
├── core/              # Main control loop and orchestration
│   └── main.py        # Central execution logic and gateway writes
├── config/            # Settings and personality definitions
├── docker/            # Dockerfiles and Jazzy orchestration
├── ros2_ws/           # ROS2 Jazzy Workspace
│   └── src/
│       ├── archer_bridge/        # Communication bridge
│       │   ├── archer_bridge/
│       │   │   ├── bridge_node.py
│       │   │   ├── vision_node.py
│       │   │   ├── safety_supervisor_node.py
│       │   │   ├── power_manager_node.py
│       │   │   └── watchdog_node.py
│       │   ├── setup.py
│       │   └── package.xml
│       ├── archer_description/   # Robot URDF models and meshes
│       │   └── launch/
│       │       ├── sim.launch.py
│       │       └── sim_v2.1.launch.py
│       └── archer_msgs/          # Custom Msg & Srv definitions
├── simulation/        # Launch files and Gazebo Harmonic worlds
└── ARCHER_SYSTEM_SPEC.md (This File)
```

---

## 7. Operational Modes
1.  **Direct Mode**: "Move forward" -> Immediate velocity burst.
2.  **Autonomous Mode**: "Go to the kitchen" -> Pathfinding with Nav2.
3.  **Exploration Mode**: "Explore the area" -> Mapping via Slam Toolbox.
4.  **Safe Recovery Mode**: Automatic backing up and costmap clearance upon navigation blockages.

---

## 8. Memory Scalability & FAISS Integration

To scale vector retrieval under resource constraints, Archer shifts vector-based retrieval from direct SQLite cosine iterations to a dedicated **FAISS (Facebook AI Similarity Search)** indexing pipeline while maintaining SQLite for metadata structures.

### A. Core Architecture
*   **FAISS Index**: Stores dense, 384-dimensional floating-point vectors corresponding to text data. Uses an inner product index (`IndexFlatIP`) optimized for cosine similarity.
*   **SQLite Mapper**: A dedicated `faiss_mapping` table maintains the alignment between sequential FAISS index positions and unique relational SQLite row IDs.
*   **Synchronous Additions**: Database inserts automatically compute text embeddings, append them to the FAISS index, write the index to `ai/memory/db/memory.index`, and insert the index map row.
*   **Robust Fallback**: If FAISS imports fail or the index file is corrupted, the system falls back to exact SQLite keyword LIKE checks and Python-based cosine iterations, preventing conversational deadlocks.

```mermaid
flowchart LR
    Query[Text Query] --> Embed[ONNX Embedding Engine]
    Embed -->|Query Vector| FAISS[FAISS Index Search]
    FAISS -->|Matched FAISS IDs| Map[faiss_mapping Table]
    Map -->|SQLite IDs| SQLite[(SQLite DB)]
    SQLite -->|Metadata Content| Context[Context Prompt Formatting]
    
    %% Fallback path
    Embed -.->|Import Failure Fallback| SQLiteKeywords[SQLite LIKE Matches]
    SQLiteKeywords --> Context
```

---

## 9. Memory Importance Scoring

To prevent prompt inflation and maintain focus on critical constraints, Archer uses a dynamic **Memory Importance Engine** combining raw semantic similarity, inherent context importance, and exponential recency decay.

### A. Scoring Equation
The final retrieval rank score $S$ for each candidate memory is defined by:
$$S = w_s \cdot \text{Similarity} + w_i \cdot \text{Importance} + w_r \cdot \text{Recency}$$
Where weight parameters are calibrated to: $w_s = 0.5$, $w_i = 0.3$, $w_r = 0.2$.

### B. Inherent Importance Matrix ($w_i$)
*   **Identity Information** (e.g., user names, entity identifiers): `0.95`
*   **User Preferences / Macros** (e.g. routine configurations, room nicknames): `0.90`
*   **Learned Locations**: `0.80`
*   **Mission logs / E-Stop Events**: `0.95` (Critical event history)
*   **Routine System telemetry**: `0.20`

### C. Recency Decay & Promotion
*   **Recency Decay**: Varies exponentially based on hours elapsed $\Delta t$ since the record was last retrieved:
    $$R(\Delta t) = e^{-\lambda \cdot \Delta t}$$
    where the decay constant $\lambda$ is set to `0.02` (approx. $50\%$ decay in 35 hours).
*   **Retrieval Promotion**: When a memory is selected in the top $K$ context window, the database increments its `access_count` and updates `last_retrieved` to the current system time, resetting its decay factor.
*   **Pruning**: During background consolidation, items with a total score $S < 0.2$ and zero access counts are automatically purged from the SQLite index.

---

## 10. Persistent Visual Memory

Visual detections are recorded as permanent, long-term spatial knowledge, allowing Archer to recall where objects were seen or labels were read.

### A. SQLite visual_memory Schema
```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "title": "VisualMemorySchema",
  "type": "object",
  "properties": {
    "id": { "type": "integer" },
    "timestamp": { "type": "string", "format": "date-time" },
    "object_name": { "type": "string" },
    "location": { "type": "string" },
    "coordinates": { "type": "string" },
    "ocr_text": { "type": "string" },
    "scene_description": { "type": "string" },
    "importance": { "type": "number" },
    "last_retrieved": { "type": "string" },
    "access_count": { "type": "integer" }
  },
  "required": ["id", "timestamp", "object_name", "location", "scene_description"]
}
```

### B. Rate-Limited Logging
To prevent database flooding, the vision node checks frame detections against a set tracker. A database write is only triggered if:
1.  The set of detected objects changes (e.g. a new item is detected or an item leaves the frame).
2.  More than 10.0 seconds have elapsed since the last observation log.

---

## 11. Safety Supervisor & Watchdog Node

A dedicated ROS2 supervisor node, `archer_watchdog_node`, monitors node heartbeats to prevent safety system freezes or AI core locks.

### A. Supervision Matrix
```mermaid
graph TD
    %% Nodes
    Bridge[Bridge Node] -->|/archer/heartbeat/bridge| WD[Watchdog Node]
    AICore[Host AI Core] -->|/archer/heartbeat/ai_core| WD
    Vision[Vision Node] -->|/archer/heartbeat/vision| WD
    Safety[Safety Supervisor] -->|/archer/heartbeat/safety| WD
    Power[Power Node] -->|/archer/heartbeat/power| WD
    
    %% Watchdog Decisions
    WD -->|Timeout safety > 0.5s| EStop[EMERGENCY_STOP: Disable CmdVel]
    WD -->|Timeout bridge > 1.0s| SafeStop[SAFE_STOP: Zero Velocity]
    WD -->|Timeout AI > 5.0s| Halt[Halt: Await uplinks]
```

### B. Timeout Thresholds & Safety Transitions

| Source Node | Timeout Limit | Escalation Action | System Effect |
| :--- | :--- | :--- | :--- |
| **safety_supervisor** | $0.5\text{ seconds}$ | `EMERGENCY_STOP` | Hard lock velocity to 0, kill joint commands, request manual reboot. |
| **bridge_node** | $1.0\text{ seconds}$ | `SAFE_STOP` | Output immediate zero velocity to `/cmd_vel` to prevent runaway robot. |
| **ai_core** | $5.0\text{ seconds}$ | `SAFE_STOP` | Slow to stop, maintain pose, wait for uplink recovery. |
| **vision_node** | $3.0\text{ seconds}$ | `WARNING` | Lock visual memory updates, alert user of visual blindness. |
| **power_manager** | $3.0\text{ seconds}$ | `WARNING` | Lock battery state queries, disable auto-docking checks. |

---

## 12. Battery & Dock Verification

To ensure reliable autonomous recharges, the power manager verifies the physical docking process before marking missions complete.

### A. Verification Steps

```mermaid
flowchart TD
    Start[Dock Command] --> Align[Check Position Alignment]
    Align -->|Distance < 0.25m| PinCheck[Verify Charger Pin Contact]
    Align -->|Distance >= 0.25m| ReAlign[Re-Align Goal]
    
    PinCheck -->|Pins Engaged| CurrentCheck[Confirm Charging Current Flow]
    PinCheck -->|Timeout / Fail| ReAlign
    
    CurrentCheck -->|Current Draw Negative| ChargeCheck[Verify Battery Level Increase]
    CurrentCheck -->|Current Draw Positive| ReAlign
    
    ChargeCheck -->|Percent Increases| Success[Mark Docking Successful]
    ChargeCheck -->|Percent Flat/Decays| ReAlign
```

### B. Retry & Back-Up Recovery
If verification fails at any stage (contact pin open, positive current draw, or flat charge rate), the node triggers a retry sequence:
1.  Command back-up movement (reverse to `[0.0, -0.6]`).
2.  Wait 5.0 seconds for pose clearance.
3.  Re-dispatch return-to-dock target coordinate command via Nav2.
4.  Limit retry loops to **3 attempts**. If all fail, halt motion, declare `failed` status, lock joints, and alert the user.

---

## 13. Unified Diagnostics Layer

The System Diagnostics Manager aggregates performance metrics and component statuses into a single, queryable diagnostic schema, allowing the AI to answer system status questions.

### A. `diagnostics.json` Schema
```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "title": "UnifiedDiagnosticsSchema",
  "type": "object",
  "properties": {
    "cpu_usage_pct": { "type": "number" },
    "ram_usage_mb": { "type": "number" },
    "vision_fps": { "type": "number" },
    "battery_percent": { "type": "number" },
    "safety_state": { "type": "string" },
    "active_navigation_goal": { "type": ["string", "null"] },
    "memory_db_health": { "type": "string", "enum": ["nominal", "error"] },
    "faiss_index_health": { "type": "string", "enum": ["active", "rebuilding", "disabled"] },
    "watchdog_status": { "type": "string" },
    "current_operational_mode": { "type": "string" }
  },
  "required": ["cpu_usage_pct", "ram_usage_mb", "vision_fps", "battery_percent", "safety_state", "memory_db_health", "faiss_index_health", "watchdog_status"]
}
```

---

## 14. Implementation Roadmap

```mermaid
gantt
    title ARCHER v2.1.1 Refinement Implementation Schedule
    dateFormat  YYYY-MM-DD
    section Phase 1: FAISS & Importance Engine
    SQLite Alter & Importance Schema    :active, p1_1, 2026-06-01, 2d
    Implement FAISS Index Managers      :p1_2, after p1_1, 3d
    section Phase 2: Watchdog & Safety Check
    Develop Watchdog Heartbeat Logic    :p2_1, 2026-06-03, 3d
    Integrate Estop Escalate Methods    :p2_2, after p2_1, 2d
    section Phase 3: Dock Verification & Telemetry
    Write Dock Validation Sequences     :p3_1, 2026-06-06, 3d
    Build Diagnostics aggregator        :p3_2, after p3_1, 2d
    section Phase 4: Integration
    Verify colcon workspace compiles    :p4_1, 2026-06-09, 2d
    Verification & Calibration          :p4_2, after p4_1, 3d
```

---
**Prepared by**: Antigravity Assistant
**Status**: Specification Extended & Refined (v2.1.1 Update)
**Date**: June 1, 2026
