import json
import logging
from typing import List, Dict, Any

logger = logging.getLogger("archer.htn_planner")

class HTNPlanner:
    """
    Hierarchical Task Network (HTN) Planner for A.R.C.H.E.R.
    Decomposes high-level abstract goals into primitive ROS actions.
    """
    def __init__(self, locations: Dict[str, Any]):
        self.locations = locations
        self.methods = {
            "patrol": self._method_patrol,
            "sweep": self._method_patrol, # Alias
            "check perimeter": self._method_patrol,
            "secure house": self._method_patrol,
            "tour": self._method_patrol
        }

    def is_abstract_goal(self, command: str) -> bool:
        """Check if a natural language command matches an HTN abstract method."""
        cmd_lower = command.lower()
        for key in self.methods.keys():
            if key in cmd_lower:
                return True
        return False

    def decompose(self, command: str) -> List[Dict[str, Any]]:
        """Decompose an abstract command into primitive actions."""
        cmd_lower = command.lower()
        
        for key, method in self.methods.items():
            if key in cmd_lower:
                logger.info(f"[HTN] Decomposing abstract goal '{key}'")
                return method()
                
        return []

    def _method_patrol(self) -> List[Dict[str, Any]]:
        """
        Decomposes a 'patrol' goal into visiting every known location.
        """
        plan = []
        # Visit all registered semantic locations except origin if possible
        rooms = [loc for loc in self.locations.keys() if loc != "origin"]
        if not rooms:
            rooms = ["origin"]
            
        for room in rooms:
            plan.append({
                "type": "nav_goal",
                "target": room,
                "coordinates": self.locations[room]
            })
            # After reaching a room, rotate to scan
            plan.append({
                "type": "cmd_vel",
                "linear": 0.0,
                "angular": 0.5,
                "duration": 6.0 # Complete a ~180 degree scan
            })
            plan.append({
                "type": "stop"
            })
            
        return plan
