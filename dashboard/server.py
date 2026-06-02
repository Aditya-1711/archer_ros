#!/usr/bin/env python3
"""
Archer Web Dashboard Server
=====================================
Serves the Tactical Control Core dashboard interface and bridges REST API
requests directly to the Llama AI core, Piper TTS, and the ROS 2 gateway.

Author: Antigravity
"""

import http.server
import socketserver
import json
import os
import sys
import yaml
import time
from pathlib import Path
from urllib.parse import urlparse

# Ensure project root is on sys.path so core imports work
PROJECT_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

# Import core modules
from core.main import _get_llm, _get_parser, _get_tts, _get_openclaw, run_pipeline_step, send_to_ros2

PORT = 8080
DIRECTORY = os.path.dirname(os.path.abspath(__file__))

# Lazy initialize AI components
print("[Dashboard] Initialising AI components...")
llm = _get_llm()
parser = _get_parser()
tts = _get_tts()
openclaw = _get_openclaw()
print("[Dashboard] AI neural core online.")

class DashboardHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=DIRECTORY, **kwargs)

    def log_message(self, format, *args):
        # Suppress spammy log outputs for status polling
        if "GET /api/status" in args[0]:
            return
        super().log_message(format, *args)

    def do_GET(self):
        parsed_path = urlparse(self.path)
        
        # 1. API Status Endpoint
        if parsed_path.path == "/api/status":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            
            # Default fallback state
            battery = 100.0
            cpu_temp = 45.0
            status_state = "nominal"
            location = "dock"
            coords = [0.0, 0.0, 0.0]  # x, y, yaw

            # Read diagnostics.json
            diag_file = PROJECT_ROOT / "simulation" / "diagnostics.json"
            if diag_file.exists():
                try:
                    with open(diag_file, "r") as f:
                        diag = json.load(f)
                        battery = float(diag.get("battery_percent", 100.0))
                        cpu_temp = float(diag.get("cpu_temp_c", 45.0))
                        status_state = diag.get("status", "nominal")
                except:
                    pass

            # Read robot_status.json
            status_file = PROJECT_ROOT / "simulation" / "robot_status.json"
            if status_file.exists():
                try:
                    with open(status_file, "r") as f:
                        status = json.load(f)
                        location = status.get("location", "unknown")
                        coords = [
                            float(status.get("x", 0.0)),
                            float(status.get("y", 0.0)),
                            float(status.get("yaw", 0.0))
                        ]
                except:
                    pass

            payload = {
                "battery": battery,
                "cpu_temp": cpu_temp,
                "status": status_state,
                "location": location,
                "coordinates": coords
            }
            self.wfile.write(json.dumps(payload).encode("utf-8"))
            return
            
        # Default file server
        return super().do_GET()

    def do_POST(self):
        parsed_path = urlparse(self.path)
        content_length = int(self.headers['Content-Length'])
        post_data = self.rfile.read(content_length)
        
        try:
            data = json.loads(post_data.decode('utf-8'))
        except Exception as e:
            self.send_error(400, f"Malformed JSON: {e}")
            return

        # 2. AI Command Pipeline Entrypoint
        if parsed_path.path == "/api/command":
            command = data.get("command", "")
            print(f"[Dashboard] Received AI Command: '{command}'")
            
            # Execute one pipeline step
            try:
                result = run_pipeline_step(command, llm, parser, tts, openclaw)
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps(result).encode("utf-8"))
            except Exception as e:
                print(f"[Dashboard] Pipeline Error: {e}")
                self.send_error(500, f"AI Core execution failed: {e}")
            return

        # 3. Manual Override Endpoint (D-Pad and Semantic nav buttons)
        elif parsed_path.path == "/api/manual":
            action_type = data.get("action", "")
            
            if action_type == "cmd_vel":
                direction = data.get("direction", "stop")
                speed_lvl = data.get("speed", "moderate")
                
                # Speed mapping
                speeds = {
                    "slow": (0.4, 0.4),
                    "moderate": (0.9, 0.8),
                    "fast": (1.4, 1.2),
                    "full": (1.9, 1.6)
                }
                lin_scale, ang_scale = speeds.get(speed_lvl, (0.9, 0.8))
                
                linear = 0.0
                angular = 0.0
                duration = 2.0 # Safe default burst
                
                if direction == "forward":
                    linear = lin_scale
                elif direction == "backward":
                    linear = -lin_scale
                elif direction == "left":
                    angular = 0.785
                    duration = 1.0
                elif direction == "right":
                    angular = -0.785
                    duration = 1.0
                elif direction == "stop":
                    linear = 0.0
                    angular = 0.0
                    duration = 0.0
                
                payload = {
                    "speech": "Manual drive override activated.",
                    "actions": [{
                        "type": "cmd_vel",
                        "linear": linear,
                        "angular": angular,
                        "duration": duration
                    }]
                }
                success = send_to_ros2(payload)
                
            elif action_type == "nav_goal":
                target = data.get("target", "dock")
                coords = data.get("coordinates", [0.0, 0.0, 0.0])
                
                payload = {
                    "speech": f"Direct uplink routing active. Heading to the {target}.",
                    "actions": [{
                        "type": "nav_goal",
                        "target": target,
                        "coordinates": coords
                    }]
                }
                success = send_to_ros2(payload)
                
            else:
                self.send_error(400, "Unknown manual action type")
                return

            if success:
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps({"status": "success"}).encode("utf-8"))
            else:
                self.send_error(500, "Failed to write command to ROS2 bridge")
            return

        self.send_error(404, "Endpoint not found")

def run():
    socketserver.TCPServer.allow_reuse_address = True
    with socketserver.TCPServer(("", PORT), DashboardHandler) as httpd:
        print(f"\n======================================================================")
        print(f"  A.R.C.H.E.R. // WEB TACTICAL DASHBOARD CONNECTED")
        print(f"  Access local panel at: http://localhost:{PORT}")
        print(f"  (Keep this server terminal open, close with Ctrl+C)")
        print(f"======================================================================\n")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nDashboard server shutting down. Standby.")

if __name__ == "__main__":
    run()
