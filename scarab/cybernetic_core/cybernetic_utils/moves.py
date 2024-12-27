import copy
from typing import List, Dict
from dataclasses import dataclass
from cybernetic_core.geometry.angles import RobotPosition


@dataclass
class MoveSnapshot:
    move_type: str
    angles_snapshot: RobotPosition
