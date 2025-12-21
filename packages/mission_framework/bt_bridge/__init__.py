"""
BT Bridge - Behavior Tree to Mission Framework Bridge

This package provides the Python-side components for behavior tree integration:
  - PhaseServer: Executes mission phases on RPC requests from C++ BT
  - StatePublisher: Publishes MissionState to Zenoh for C++ blackboard sync

Usage:
    python -m mission_framework.bt_bridge.run_server --connect tcp/localhost:7447
"""

from .phase_server import PhaseServer
from .state_publisher import StatePublisher

__all__ = ['PhaseServer', 'StatePublisher']
