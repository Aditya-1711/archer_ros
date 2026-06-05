import threading
import logging
from typing import Callable

logger = logging.getLogger("archer.timer_manager")

class TimerManager:
    def __init__(self):
        self.timers = []
        
    def start_timer(self, duration_sec: int, callback: Callable[[str], None], location: str = "nottingham UK"):
        """Starts a background timer. When done, invokes the callback."""
        def timer_done():
            logger.info(f"[Timer] Timer for {duration_sec}s completed.")
            # Speak the message
            callback(f"Boss, your timer is complete. Location: {location}")
            
        t = threading.Timer(duration_sec, timer_done)
        t.daemon = True
        t.start()
        self.timers.append(t)
        logger.info(f"[Timer] Started timer for {duration_sec} seconds.")

