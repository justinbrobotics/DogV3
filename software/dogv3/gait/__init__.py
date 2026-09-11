"""Gait generation: phase clock + trot/crawl foot-trajectory generator."""

from .phase import CRAWL_LIFT_ORDER, CRAWL_MIN_DUTY, PhaseClock, TROT_OFFSETS
from .trot import TrotGait, GaitCommand, FootTarget

__all__ = [
    "PhaseClock",
    "TROT_OFFSETS",
    "CRAWL_LIFT_ORDER",
    "CRAWL_MIN_DUTY",
    "TrotGait",
    "GaitCommand",
    "FootTarget",
]
