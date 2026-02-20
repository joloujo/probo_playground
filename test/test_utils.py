from math import pi, sqrt
import pytest
from utils import Position, Pose, Bounds, Landmark, BearingRange

class TestUtils:
    def test_positions_add(self):
        assert Position(1, 2) + Position(3, 4) == Position(4, 6)

        # Make sure it works with negatives
        assert Position(-1, -2) + Position(3, 4) == Position(2, 2)
        assert Position(1, 2) + Position(-3, -4) == Position(-2, -2)

    def test_positions_subtract(self):
        assert Position(1, 2) - Position(3, 4) == Position(-2, -2)
        assert Position(3, 4) - Position(1, 2) == Position(2, 2)

    def test_magnitude(self):
        assert Position(0).magnitude == 0
        assert Position(3, 4).magnitude == 5
        assert Position(4, 3).magnitude == 5
        assert Position(-3, 4).magnitude == 5
        assert Position(3, -4).magnitude == 5
        assert Position(-3, -4).magnitude == 5
        assert Position(5, 12).magnitude == 13
        assert Position(1, 1).magnitude == sqrt(2)
    
    def test_angle(self):
        assert Position(0, 0).angle == 0
        assert Position(1, 0).angle == 0
        assert Position(1, 1).angle == pi/4
        assert Position(0, 1).angle == pi/2
        assert Position(-1, 0).angle == pi
        assert Position(1, sqrt(3)).angle == pi/3