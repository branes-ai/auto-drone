"""
Mission Configuration

Provides dataclasses for mission configuration and YAML loading.
"""

from dataclasses import dataclass, field
from typing import List, Dict, Any, Optional
from pathlib import Path
import yaml


@dataclass
class PhaseConfig:
    """Configuration for a single phase."""
    name: str              # Display name
    type: str              # Phase class name: "ascend", "scan", "navigate", etc.
    enabled: bool = True   # Whether to execute this phase
    params: Dict[str, Any] = field(default_factory=dict)  # Phase-specific parameters


@dataclass
class MissionConfig:
    """
    Complete mission configuration.

    Loaded from YAML file and optionally overridden by CLI arguments.
    """
    # Mission identification
    name: str = "unnamed_mission"
    description: str = ""

    # Connection
    robot_id: str = "drone"
    cameras: List[str] = field(default_factory=lambda: ["front"])

    # Target specification
    target_class: str = "orange ball"

    # Altitude parameters (positive meters, converted to NED internally)
    observation_altitude: float = 25.0  # meters above ground
    approach_altitude: float = 5.0      # meters above ground

    # Speed parameters
    ascent_speed: float = 3.0       # m/s vertical
    descent_speed: float = 2.0      # m/s vertical
    nav_speed: float = 5.0          # m/s horizontal
    approach_speed: float = 2.0     # m/s during final approach
    scan_yaw_rate: float = 0.5      # rad/s

    # Distance thresholds
    descent_threshold: float = 10.0    # Start descent when within XY distance
    arrival_distance: float = 3.0      # Mission complete when this close
    camera_handoff_distance: float = 8.0  # Switch to down camera

    # Timeouts
    mission_timeout: float = 180.0  # seconds total
    phase_timeout: float = 60.0     # seconds per phase

    # Phase sequence
    phases: List[PhaseConfig] = field(default_factory=list)

    @classmethod
    def from_yaml(cls, path: Path) -> 'MissionConfig':
        """
        Load config from YAML file.

        Args:
            path: Path to YAML config file

        Returns:
            MissionConfig instance

        Raises:
            FileNotFoundError: If config file doesn't exist
            yaml.YAMLError: If YAML is malformed
        """
        path = Path(path)
        with open(path, 'r') as f:
            data = yaml.safe_load(f)

        # Parse phases
        phases = []
        for phase_data in data.get('phases', []):
            phase_type = phase_data.get('type', '')
            phases.append(PhaseConfig(
                name=phase_data.get('name', phase_type),
                type=phase_type,
                enabled=phase_data.get('enabled', True),
                params=phase_data.get('params', {})
            ))

        return cls(
            name=data.get('name', 'unnamed_mission'),
            description=data.get('description', ''),
            robot_id=data.get('robot_id', 'drone'),
            cameras=data.get('cameras', ['front']),
            target_class=data.get('target_class', 'orange ball'),
            observation_altitude=data.get('observation_altitude', 25.0),
            approach_altitude=data.get('approach_altitude', 5.0),
            ascent_speed=data.get('ascent_speed', 3.0),
            descent_speed=data.get('descent_speed', 2.0),
            nav_speed=data.get('nav_speed', 5.0),
            approach_speed=data.get('approach_speed', 2.0),
            scan_yaw_rate=data.get('scan_yaw_rate', 0.5),
            descent_threshold=data.get('descent_threshold', 10.0),
            arrival_distance=data.get('arrival_distance', 3.0),
            camera_handoff_distance=data.get('camera_handoff_distance', 8.0),
            mission_timeout=data.get('mission_timeout', 180.0),
            phase_timeout=data.get('phase_timeout', 60.0),
            phases=phases
        )

    @classmethod
    def default_orange_ball(cls) -> 'MissionConfig':
        """Create default config for orange ball mission."""
        return cls(
            name="orange_ball_default",
            description="Default orange ball tracking mission",
            cameras=["front", "back", "down"],
            target_class="orange ball",
            phases=[
                PhaseConfig(name="Ascend", type="ascend"),
                PhaseConfig(name="Scan", type="scan", params={"rotation_degrees": 180}),
                PhaseConfig(name="Select Target", type="select"),
                PhaseConfig(name="Navigate", type="navigate"),
                PhaseConfig(name="Descend", type="descend"),
                PhaseConfig(name="Approach", type="approach"),
            ]
        )

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for serialization."""
        return {
            'name': self.name,
            'description': self.description,
            'robot_id': self.robot_id,
            'cameras': self.cameras,
            'target_class': self.target_class,
            'observation_altitude': self.observation_altitude,
            'approach_altitude': self.approach_altitude,
            'ascent_speed': self.ascent_speed,
            'descent_speed': self.descent_speed,
            'nav_speed': self.nav_speed,
            'approach_speed': self.approach_speed,
            'scan_yaw_rate': self.scan_yaw_rate,
            'descent_threshold': self.descent_threshold,
            'arrival_distance': self.arrival_distance,
            'camera_handoff_distance': self.camera_handoff_distance,
            'mission_timeout': self.mission_timeout,
            'phase_timeout': self.phase_timeout,
            'phases': [
                {
                    'name': p.name,
                    'type': p.type,
                    'enabled': p.enabled,
                    'params': p.params
                }
                for p in self.phases
            ]
        }

    def to_yaml(self, path: Path) -> None:
        """Save config to YAML file."""
        path = Path(path)
        with open(path, 'w') as f:
            yaml.dump(self.to_dict(), f, default_flow_style=False, sort_keys=False)

    def print_summary(self) -> None:
        """Print configuration summary."""
        print(f"Mission: {self.name}")
        print(f"  Description: {self.description}")
        print(f"  Target: '{self.target_class}'")
        print(f"  Cameras: {self.cameras}")
        print(f"  Observation altitude: {self.observation_altitude}m")
        print(f"  Approach altitude: {self.approach_altitude}m")
        print(f"  Phases: {len(self.phases)}")
        for i, p in enumerate(self.phases):
            status = "" if p.enabled else " [DISABLED]"
            print(f"    {i+1}. {p.name} ({p.type}){status}")
