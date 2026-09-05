"""Perception: object detection and tracking helpers.

Works on a lightweight in-memory "detection" representation so the module
is testable without any dataset. Real detectors (YOLO etc.) can produce
these dicts downstream.
"""
from __future__ import annotations

from dataclasses import dataclass, field, asdict
from typing import List, Optional


@dataclass
class DetectedObject:
    """A single detected road object (bird's-eye-view frame)."""
    obj_id: int
    label: str                 # car / pedestrian / cow / ...
    x: float                   # forward position (m)
    y: float                   # lateral position (m)
    speed: float = 0.0         # m/s
    confidence: float = 1.0    # detector confidence [0, 1]

    def as_dict(self) -> dict:
        return asdict(self)


@dataclass
class PerceptionFrame:
    """All objects detected in one frame."""
    frame_id: int
    objects: List[DetectedObject] = field(default_factory=list)

    def by_label(self, label: str) -> List[DetectedObject]:
        return [o for o in self.objects if o.label == label]

    def nearest(self, x: float, y: float) -> Optional[DetectedObject]:
        """Closest object to a point, or None if the frame is empty."""
        if not self.objects:
            return None
        return min(self.objects, key=lambda o: (o.x - x) ** 2 + (o.y - y) ** 2)

    def as_dict(self) -> dict:
        return {"frame_id": self.frame_id,
                "objects": [o.as_dict() for o in self.objects]}


def filter_by_confidence(frame: PerceptionFrame, min_conf: float = 0.5) -> PerceptionFrame:
    """Return a new frame keeping only detections above min_conf."""
    return PerceptionFrame(
        frame_id=frame.frame_id,
        objects=[o for o in frame.objects if o.confidence >= min_conf],
    )
