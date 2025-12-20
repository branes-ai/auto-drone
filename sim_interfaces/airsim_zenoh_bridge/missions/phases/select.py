"""
Select Phase

Select target from inventory.
"""

import asyncio
import math

from . import register_phase
from .base import MissionPhase, PhaseResult, PhaseStatus


@register_phase("select")
class SelectPhase(MissionPhase):
    """
    Phase to select target from inventory.

    Chooses the best target based on strategy and sets state.selected_target.

    Config params:
        strategy: Selection strategy - "best_confidence", "largest", "nearest"
        min_confidence: Minimum confidence threshold (default: 0.25)

    State params used:
        target_class: Object class to select
    """

    async def execute(self) -> PhaseResult:
        # Get parameters
        target_class = self.get_param('target_class', 'orange ball')
        strategy = self.config.get('strategy', 'best_confidence')
        min_confidence = self.config.get('min_confidence', 0.25)

        print(f"  Target class: '{target_class}'")
        print(f"  Strategy: {strategy}")

        # Get candidates
        candidates = self.state.inventory.get_all(target_class)

        if not candidates:
            return PhaseResult(
                status=PhaseStatus.FAILED,
                message=f"No '{target_class}' objects in inventory"
            )

        # Filter by confidence
        candidates = [c for c in candidates if c.confidence >= min_confidence]

        if not candidates:
            return PhaseResult(
                status=PhaseStatus.FAILED,
                message=f"No '{target_class}' with confidence >= {min_confidence}"
            )

        # Select based on strategy
        if strategy == "best_confidence":
            selected = max(candidates, key=lambda o: o.confidence)
        elif strategy == "largest":
            selected = max(candidates, key=lambda o: o.area)
        elif strategy == "nearest":
            x, y, z, _ = self.drone.pose
            selected = min(candidates, key=lambda o: math.sqrt(
                (o.world_x - x) ** 2 + (o.world_y - y) ** 2
            ))
        else:
            # Default to best confidence
            selected = max(candidates, key=lambda o: o.confidence)

        # Set selected target
        self.state.selected_target = selected

        # Calculate distance
        x, y, z, _ = self.drone.pose
        dist = math.sqrt(
            (selected.world_x - x) ** 2 + (selected.world_y - y) ** 2
        )

        print(f"  Selected: confidence={selected.confidence:.2f}")
        print(f"  Position: ({selected.world_x:.1f}, {selected.world_y:.1f})")
        print(f"  Distance: {dist:.1f}m")

        return PhaseResult(
            status=PhaseStatus.COMPLETED,
            message=f"Selected '{target_class}' at ({selected.world_x:.1f}, {selected.world_y:.1f})",
            data={
                'target_x': selected.world_x,
                'target_y': selected.world_y,
                'confidence': selected.confidence,
                'distance': dist,
            }
        )
