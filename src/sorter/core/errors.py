"""Shared errors. "Nothing found" is never an exception: vision returns it as a result."""


class SorterError(Exception):
    """Base class for all project errors."""


class CameraError(SorterError):
    """No frame within timeout, device lost."""


class CalibrationError(SorterError):
    """No camera pose, invalid depth, file missing."""


class ArmError(SorterError):
    """SDK/bus fault, motion failed or timed out."""


class TargetRejected(ArmError):
    """Outside the zone workspace or IK failed. NO motion happened."""


class EStopped(ArmError):
    """The arm is held. Every motion raises this until recover()."""
