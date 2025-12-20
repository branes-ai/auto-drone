"""
Mission Phases

Reusable phase implementations for drone missions.
"""

from typing import Dict, Type, TYPE_CHECKING

if TYPE_CHECKING:
    from .base import MissionPhase

# Phase registry - maps type names to phase classes
PHASE_REGISTRY: Dict[str, Type['MissionPhase']] = {}


def register_phase(name: str):
    """
    Decorator to register a phase class with a type name.

    Usage:
        @register_phase("ascend")
        class AscendPhase(MissionPhase):
            ...

    The phase can then be referenced in YAML configs as:
        phases:
          - type: ascend
    """
    def decorator(cls: Type['MissionPhase']) -> Type['MissionPhase']:
        PHASE_REGISTRY[name] = cls
        return cls
    return decorator


def get_phase_class(name: str) -> Type['MissionPhase']:
    """
    Get phase class by type name.

    Args:
        name: Phase type name (e.g., "ascend", "scan")

    Returns:
        Phase class

    Raises:
        KeyError: If phase type not found
    """
    if name not in PHASE_REGISTRY:
        available = ", ".join(sorted(PHASE_REGISTRY.keys()))
        raise KeyError(f"Unknown phase type '{name}'. Available: {available}")
    return PHASE_REGISTRY[name]


def list_phases() -> list:
    """List all registered phase types."""
    return sorted(PHASE_REGISTRY.keys())


# Import phase modules to trigger registration
# These imports must come after PHASE_REGISTRY is defined
from .base import MissionPhase, PhaseResult, PhaseStatus
from .ascend import AscendPhase
from .scan import ScanPhase
from .select import SelectPhase
from .navigate import NavigatePhase
from .descend import DescendPhase
from .approach import ApproachPhase
from .land import LandPhase

__all__ = [
    'PHASE_REGISTRY',
    'register_phase',
    'get_phase_class',
    'list_phases',
    'MissionPhase',
    'PhaseResult',
    'PhaseStatus',
    'AscendPhase',
    'ScanPhase',
    'SelectPhase',
    'NavigatePhase',
    'DescendPhase',
    'ApproachPhase',
    'LandPhase',
]
