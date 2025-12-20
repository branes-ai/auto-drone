"""
Base Phase Classes

Abstract base class for mission phases and supporting types.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Any, Dict, Optional, TYPE_CHECKING
import time

if TYPE_CHECKING:
    from ..state import MissionState
    from ..services.drone import DroneService
    from ..services.detection import DetectionService


class PhaseStatus(Enum):
    """Phase execution status."""
    PENDING = auto()     # Not yet started
    RUNNING = auto()     # Currently executing
    COMPLETED = auto()   # Finished successfully
    FAILED = auto()      # Finished with error
    SKIPPED = auto()     # Skipped (disabled or precondition not met)


@dataclass
class PhaseResult:
    """Result of a phase execution."""
    status: PhaseStatus
    message: str = ""
    data: Optional[Dict[str, Any]] = None  # Phase-specific output data
    elapsed_time: float = 0.0


class MissionPhase(ABC):
    """
    Abstract base class for mission phases.

    Phases are async and run within the mission runner's event loop.
    They access shared state and services through injected dependencies.

    Lifecycle:
        1. on_enter() - Called when phase starts
        2. execute() - Main phase logic (must be implemented)
        3. on_exit() - Called when phase ends

    Example:
        @register_phase("ascend")
        class AscendPhase(MissionPhase):
            async def execute(self) -> PhaseResult:
                target_alt = self.config.get('altitude', 25.0)
                while self.state.running and not self.is_cancelled:
                    if self.state.altitude >= target_alt - 0.5:
                        return PhaseResult(PhaseStatus.COMPLETED, "Reached altitude")
                    self.drone.send_velocity(0, 0, -3.0, 0)
                    await asyncio.sleep(0.1)
                return PhaseResult(PhaseStatus.FAILED, "Cancelled")
    """

    def __init__(
        self,
        name: str,
        state: 'MissionState',
        drone: 'DroneService',
        detection: 'DetectionService',
        config: Dict[str, Any]
    ):
        """
        Initialize phase.

        Args:
            name: Display name for this phase
            state: Shared mission state
            drone: Drone control service
            detection: Detection service
            config: Phase-specific configuration parameters
        """
        self.name = name
        self.state = state
        self.drone = drone
        self.detection = detection
        self.config = config
        self.status = PhaseStatus.PENDING
        self._cancelled = False
        self._start_time = 0.0

    @abstractmethod
    async def execute(self) -> PhaseResult:
        """
        Execute the phase.

        Must be implemented by subclasses.
        Should check self.state.running and self.is_cancelled periodically.

        Returns:
            PhaseResult indicating success/failure and any output data.
        """
        pass

    async def on_enter(self) -> None:
        """
        Called when phase starts.

        Override for custom setup. Default prints phase start message.
        """
        self.status = PhaseStatus.RUNNING
        self._start_time = time.time()
        print(f"\n[{self.name.upper()}] Starting...")

    async def on_exit(self, result: PhaseResult) -> None:
        """
        Called when phase ends.

        Override for custom cleanup. Default prints completion message.
        """
        self.status = result.status
        status_str = "COMPLETED" if result.status == PhaseStatus.COMPLETED else "FAILED"
        elapsed = result.elapsed_time
        print(f"[{self.name.upper()}] {status_str} ({elapsed:.1f}s): {result.message}")

    def cancel(self) -> None:
        """Request phase cancellation."""
        self._cancelled = True

    @property
    def is_cancelled(self) -> bool:
        """Check if phase cancellation was requested."""
        return self._cancelled

    @property
    def elapsed_time(self) -> float:
        """Get time since phase started."""
        if self._start_time > 0:
            return time.time() - self._start_time
        return 0.0

    async def run(self) -> PhaseResult:
        """
        Run phase with enter/exit lifecycle.

        This is the main entry point called by MissionRunner.
        Handles the lifecycle and timing.
        """
        start_time = time.time()

        await self.on_enter()
        try:
            result = await self.execute()
        except Exception as e:
            import traceback
            traceback.print_exc()
            result = PhaseResult(
                status=PhaseStatus.FAILED,
                message=f"Exception: {e}"
            )

        result.elapsed_time = time.time() - start_time
        await self.on_exit(result)
        return result

    def get_param(self, key: str, default: Any = None) -> Any:
        """
        Get phase parameter, falling back to mission state params.

        Args:
            key: Parameter name
            default: Default value if not found

        Returns:
            Parameter value
        """
        if key in self.config:
            return self.config[key]
        if key in self.state.params:
            return self.state.params[key]
        return default
