"""
State Publisher - Publishes MissionState to Zenoh for C++ blackboard sync.

This module continuously publishes the current mission state to Zenoh topics
that the C++ BlackboardSync class subscribes to.
"""

import json
import threading
import time
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    import zenoh
    from ..state import MissionState


class StatePublisher:
    """
    Publishes MissionState to Zenoh for C++ behavior tree blackboard.

    Publishes to:
        robot/{id}/bt/state/target - Selected target info
        robot/{id}/bt/state/inventory - Object inventory summary
        robot/{id}/bt/state/mission - Mission running state

    Format: JSON payloads
    """

    def __init__(
        self,
        session: 'zenoh.Session',
        state: 'MissionState',
        robot_id: str = "drone",
        publish_rate_hz: float = 10.0
    ):
        """
        Initialize StatePublisher.

        Args:
            session: Zenoh session for publishing
            state: MissionState to publish
            robot_id: Robot identifier for topic namespacing
            publish_rate_hz: Publishing rate in Hz (default: 10)
        """
        self.session = session
        self.state = state
        self.robot_id = robot_id
        self.publish_interval = 1.0 / publish_rate_hz

        self.target_topic = f"robot/{robot_id}/bt/state/target"
        self.inventory_topic = f"robot/{robot_id}/bt/state/inventory"
        self.mission_topic = f"robot/{robot_id}/bt/state/mission"

        self._running = False
        self._thread = None

    def start(self) -> None:
        """Start publishing state."""
        if self._running:
            return

        self._running = True
        self._thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._thread.start()

        print(f"[StatePublisher] Started publishing at {1.0/self.publish_interval:.1f} Hz")
        print(f"[StatePublisher] Topics: {self.target_topic}, {self.inventory_topic}, {self.mission_topic}")

    def stop(self) -> None:
        """Stop publishing state."""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

        print("[StatePublisher] Stopped")

    def _publish_loop(self) -> None:
        """Background thread that publishes state periodically."""
        while self._running:
            try:
                self._publish_target_state()
                self._publish_inventory_state()
                self._publish_mission_state()
            except Exception as e:
                print(f"[StatePublisher] Error: {e}")

            time.sleep(self.publish_interval)

    def _publish_target_state(self) -> None:
        """Publish selected target information."""
        target = self.state.selected_target

        if target is not None:
            data = {
                "selected": True,
                "x": target.world_x,
                "y": target.world_y,
                "class": target.class_name,
                "confidence": target.confidence,
                "camera": target.camera_id
            }
        else:
            data = {
                "selected": False,
                "x": 0.0,
                "y": 0.0,
                "class": "",
                "confidence": 0.0,
                "camera": ""
            }

        payload = json.dumps(data).encode('utf-8')
        self.session.put(self.target_topic, payload)

    def _publish_inventory_state(self) -> None:
        """Publish object inventory summary."""
        inventory = self.state.inventory

        # Get target class from params if available
        target_class = self.state.params.get("target_class", "")

        data = {
            "count": inventory.count() if inventory else 0,
            "target_count": inventory.count(target_class) if inventory and target_class else 0
        }

        payload = json.dumps(data).encode('utf-8')
        self.session.put(self.inventory_topic, payload)

    def _publish_mission_state(self) -> None:
        """Publish mission running state."""
        data = {
            "running": self.state.running
        }

        payload = json.dumps(data).encode('utf-8')
        self.session.put(self.mission_topic, payload)
