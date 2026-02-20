"""
This file contains several useful custom datatypes for you to use at your convenience! Generally, they provide structure for data that is commonly grouped together anyway (such as x and y coordinates, rectangle dimensions, and sensor measurements).

There is nothing you need to edit or fill in within this file, but feel free to alter the existing datatypes and add more as you see fit!
"""

from dataclasses import dataclass
from math import pow, sqrt, atan2
from typing import TypedDict

@dataclass(unsafe_hash=True)
class Position:
    """
    Represents an xy coordinate.
    """

    x: float = 0.0
    y: float = 0.0

    def to_dict(self):
        """
        Return in dictionary format.
        """
        return {
            "x": self.x,
            "y": self.y,
        }

    def to_string(self):
        """
        Return in string format.
        """
        return f"X{self.x}Y{self.y}"
    
    def __add__(self, other: 'Position') -> 'Position':
        """
        Add one position to another
        """
        return Position(self.x + other.x, self.y + other.y)
    
    def __sub__(self, other: 'Position') -> 'Position':
        """
        Subtract one position from another
        """
        return Position(self.x - other.x, self.y - other.y)
    
    @property
    def magnitude(self) -> float:
        return sqrt(pow(self.x, 2) + pow(self.y, 2))
    
    @property
    def angle(self) -> float:
        return atan2(self.y, self.x)
    
    def copy(self) -> 'Position':
        return Position(self.x, self.y)


@dataclass(unsafe_hash=True)
class Pose:
    """
    Represents an xy coordinate with an associated heading.
    """

    pos: Position = Position()
    theta: float = 0.0

    def to_dict(self):
        """
        Return in dictionary format.
        """
        return {
            "pos": self.pos.to_dict(),
            "theta": self.theta,
        }

    def to_string(self):
        """
        Return in string format.
        """
        return self.pos.to_string() + f"T{self.theta}"
    
    def copy(self) -> 'Pose':
        return Pose(self.pos.copy(), self.theta)


@dataclass(frozen=True)
class Bounds:
    """
    Represents any bounded area, including obstacles such as walls or the environment itself. The edge of a Bounds instance is considered to be contained by that instance.
    """

    x_min: float
    x_max: float
    y_min: float
    y_max: float

    def within_x(self, x: float) -> bool:
        """
        Check if an x value is within the x limits (inclusive).
        """
        return self.x_max >= x and self.x_min <= x

    def within_y(self, y: float) -> bool:
        """
        Check if an y value is within the y limits (inclusive).
        """
        return self.y_max >= y and self.y_min <= y

    def within_bounds(self, pos: Position) -> bool:
        """
        Check if an xy coordinate is within the bounds (inclusive).
        """
        return self.within_x(pos.x) and self.within_y(pos.y)
    
    def raycast(self, ray_start: Position, ray_end: Position) -> Position | None:
        """
        Docstring for raycast
        
        Args:
            ray_start: Description
            ray_end: Description
        Returns:
            The position of the first intersection between the ray and the edges of the Bounds, or None if there is no intersection
        """

        # TODO: Finish implementing
        edges: list[tuple[Position, Position]] = [
            (Position(self.x_min, self.y_min), Position(self.x_min, self.y_max)), # (min, min) -> (min, max)
            (Position(self.x_min, self.y_min), Position(self.x_max, self.y_min)), # (min, min) -> (max, min)
            (Position(self.x_min, self.y_max), Position(self.x_max, self.y_max)), # (min, max) -> (max, max)
            (Position(self.x_max, self.y_min), Position(self.x_max, self.y_max)), # (max, min) -> (max, max)
        ]

        collisions: list[Position] = []

        for edge in edges:
            pass
            


    def to_dict(self):
        """
        Return in dictionary format.
        """
        return {
            "x_min": self.x_min,
            "x_max": self.x_max,
            "y_min": self.y_min,
            "y_max": self.y_max,
        }

    def to_string(self):
        """
        Return in string format.
        """
        return f"X{self.x_min}-{self.x_max}Y{self.y_min}-{self.y_max}"


@dataclass(frozen=True)
class Landmark:
    """
    Represents an identifiable floating-point landmark.
    """

    pos: Position
    id: int

    def to_dict(self):
        """
        Return in dictionary format.
        """
        return {
            "id": self.id,
            "pos": self.pos.to_dict(),
        }

    def to_string(self):
        """
        Return in string format.
        """
        return f"L{self.id}" + self.pos.to_string()


@dataclass(frozen=True)
class BearingRange:
    """
    Represents the relationship between the robot and a landmark.
    """

    landmark_id: float
    bearing: float
    range: float

    def to_dict(self):
        """
        Return in dictionary format.
        """
        return {
            "landmark_id": self.landmark_id,
            "bearing": self.bearing,
            "range": self.range,
        }

    def to_string(self):
        """
        Return in string format.
        """
        return f"LM{self.landmark_id}B{self.bearing}R{self.range}"

class State(TypedDict):
    time: list[float]
    robot_pose: list[Pose]
    landmark_br: list[list[BearingRange]]
