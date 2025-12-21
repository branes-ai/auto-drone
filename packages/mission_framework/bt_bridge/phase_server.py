"""
Phase Server - Executes mission phases on demand from C++ behavior tree.

Subscribes to phase requests via Zenoh, executes the phase using the
existing mission framework, and publishes the response.
"""

import asyncio
import json
import time
import traceback
from typing import Optional, Dict, Any, TYPE_CHECKING

if TYPE_CHECKING:
    import zenoh
    from ..state import MissionState
    from ..services.drone import DroneService
    from ..services.detection import DetectionService


class PhaseServer:
    """
    Zenoh-based RPC server for executing mission phases.

    Listens for phase requests from the C++ behavior tree runner,
    executes the requested phase, and returns the result.

    Zenoh Topics:
        Subscribe: robot/{id}/bt/phase/request/{phase_name}
        Publish: robot/{id}/bt/phase/response/{phase_name}
        Subscribe: robot/{id}/bt/phase/cancel/{phase_name}

    Request format: {"params": {...}}
    Response format: {"status": str, "message": str, "data": {...}, "timestamp": float}
    """

    def __init__(
        self,
        session: 'zenoh.Session',
        state: 'MissionState',
        drone: 'DroneService',
        detection: 'DetectionService',
        robot_id: str = "drone"
    ):
        """
        Initialize PhaseServer.

        Args:
            session: Zenoh session for pub/sub
            state: Shared mission state
            drone: Drone control service
            detection: Detection service
            robot_id: Robot identifier for topic namespacing
        """
        self.session = session
        self.state = state
        self.drone = drone
        self.detection = detection
        self.robot_id = robot_id

        self.request_topic = f"robot/{robot_id}/bt/phase/request/*"
        self.cancel_topic = f"robot/{robot_id}/bt/phase/cancel/*"
        self.response_base = f"robot/{robot_id}/bt/phase/response"

        self._request_subscriber = None
        self._cancel_subscriber = None
        self._running = False
        self._current_phase = None
        self._current_task: Optional[asyncio.Task] = None

    async def start(self) -> None:
        """Start listening for phase requests."""
        self._running = True

        # Subscribe to phase requests
        self._request_subscriber = self.session.declare_subscriber(
            self.request_topic,
            self._on_request
        )

        # Subscribe to cancel requests
        self._cancel_subscriber = self.session.declare_subscriber(
            self.cancel_topic,
            self._on_cancel
        )

        print(f"[PhaseServer] Listening on {self.request_topic}")
        print(f"[PhaseServer] Cancel topic: {self.cancel_topic}")

    async def stop(self) -> None:
        """Stop the server and cancel any running phase."""
        self._running = False

        # Cancel current phase if running
        if self._current_phase is not None:
            self._current_phase.cancel()

        if self._current_task is not None:
            self._current_task.cancel()
            try:
                await self._current_task
            except asyncio.CancelledError:
                pass

        # Undeclare subscribers
        if self._request_subscriber is not None:
            self._request_subscriber.undeclare()
        if self._cancel_subscriber is not None:
            self._cancel_subscriber.undeclare()

        print("[PhaseServer] Stopped")

    def _on_request(self, sample) -> None:
        """Handle incoming phase request (callback from Zenoh)."""
        # Extract phase name from key: robot/drone/bt/phase/request/{phase_name}
        key_str = str(sample.key_expr)
        key_parts = key_str.split("/")
        if len(key_parts) < 1:
            return
        phase_name = key_parts[-1]

        # Parse request JSON
        try:
            payload_bytes = bytes(sample.payload)
            request = json.loads(payload_bytes.decode('utf-8'))
        except Exception as e:
            print(f"[PhaseServer] Failed to parse request: {e}")
            request = {}

        params = request.get("params", {})

        # Handle string params (may be JSON encoded)
        if isinstance(params, str):
            try:
                params = json.loads(params)
            except:
                params = {}

        print(f"[PhaseServer] Received request for phase '{phase_name}' with params: {params}")

        # Execute in background task
        loop = asyncio.get_event_loop()
        self._current_task = loop.create_task(self._execute_phase(phase_name, params))

    def _on_cancel(self, sample) -> None:
        """Handle cancel request."""
        key_str = str(sample.key_expr)
        key_parts = key_str.split("/")
        phase_name = key_parts[-1] if key_parts else ""

        print(f"[PhaseServer] Cancel request for phase '{phase_name}'")

        if self._current_phase is not None:
            self._current_phase.cancel()

    async def _execute_phase(self, phase_name: str, params: Dict[str, Any]) -> None:
        """Execute a phase and publish response."""
        response_topic = f"{self.response_base}/{phase_name}"
        start_time = time.time()

        # Import phase registry
        try:
            from ..phases import get_phase_class, PhaseStatus
        except ImportError as e:
            self._publish_response(response_topic, "FAILED", f"Import error: {e}", {})
            return

        # Get phase class
        try:
            phase_cls = get_phase_class(phase_name)
        except KeyError as e:
            self._publish_response(
                response_topic,
                "FAILED",
                f"Unknown phase: {phase_name}",
                {}
            )
            return

        # Create phase instance
        try:
            phase = phase_cls(
                name=phase_name,
                state=self.state,
                drone=self.drone,
                detection=self.detection,
                config=params
            )
        except Exception as e:
            self._publish_response(
                response_topic,
                "FAILED",
                f"Failed to create phase: {e}",
                {}
            )
            return

        self._current_phase = phase

        # Execute phase
        try:
            result = await phase.run()
            elapsed = time.time() - start_time

            if result.status == PhaseStatus.COMPLETED:
                status = "COMPLETED"
            elif result.status == PhaseStatus.SKIPPED:
                status = "SKIPPED"
            else:
                status = "FAILED"

            self._publish_response(
                response_topic,
                status,
                result.message,
                result.data or {},
                elapsed
            )

        except asyncio.CancelledError:
            elapsed = time.time() - start_time
            self._publish_response(
                response_topic,
                "CANCELLED",
                "Phase was cancelled",
                {},
                elapsed
            )

        except Exception as e:
            elapsed = time.time() - start_time
            traceback.print_exc()
            self._publish_response(
                response_topic,
                "FAILED",
                f"Exception: {e}",
                {},
                elapsed
            )

        finally:
            self._current_phase = None

    def _publish_response(
        self,
        topic: str,
        status: str,
        message: str,
        data: Dict[str, Any],
        elapsed: float = 0.0
    ) -> None:
        """Publish phase execution response."""
        response = {
            "status": status,
            "message": message,
            "data": data,
            "elapsed": elapsed,
            "timestamp": time.time()
        }

        payload = json.dumps(response).encode('utf-8')
        self.session.put(topic, payload)

        print(f"[PhaseServer] Response: {status} - {message}")
