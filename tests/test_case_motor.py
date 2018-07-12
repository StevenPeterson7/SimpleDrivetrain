import unittest
from motor import Motor
import numpy as np


class TestCaseMotor(unittest.TestCase):
    def __assertWithinRange(self, expected, observed, percent_range):
        if expected < 0:
            within_range = (observed >= (1.0 + percent_range) * expected) \
                           and (observed <= (1.0 - percent_range) * expected)
            self.assertTrue(within_range)
        elif expected > 0:
            within_range = (observed >= (1.0 - percent_range) * expected) \
                           and (observed <= (1.0 + percent_range) * expected)
            self.assertTrue(within_range)
        else:  # expected == 0
            within_range = (observed >= -percent_range) and (observed <= percent_range)
            self.assertTrue(within_range)

    def test_position(self):
        test_motor = Motor('fr', [np.sqrt(2.0)/2.0, np.sqrt(2.0)/2.0, 0], [-np.sqrt(2.0)/2.0, np.sqrt(2.0)/2.0, 0])
        expected = [0.0, 0.0, np.pi / 4.0]
        observed = test_motor.angle_position

        for i in range(0, len(expected)):
            if (expected[i] is None) or (observed[i] is None):
                self.assertTrue(expected[i] is observed[i])
            else:
                self.__assertWithinRange(expected[i], observed[i], 0.02)

        test_motor.position = [-np.sqrt(2.0)/2.0, np.sqrt(2.0)/2.0, 0]
        expected = [0.0, np.pi, 3.0 * np.pi / 4.0]
        observed = test_motor.angle_position

        for i in range(0, len(expected)):
            if (expected[i] is None) or (observed[i] is None):
                self.assertTrue(expected[i] is observed[i])
            else:
                self.__assertWithinRange(expected[i], observed[i], 0.02)

    def test_direction(self):

        test_motor = Motor('fr', [np.sqrt(2)/2, np.sqrt(2)/2, 0], [1, 1, 0])
        expected = [np.sqrt(2)/2, np.sqrt(2)/2, 0]
        observed = test_motor.direction

        for i in range(0, len(expected)):
            self.__assertWithinRange(expected[i], observed[i], 0.02)

    def test_inverted(self):
        test_motor1 = Motor('fr', [np.sqrt(2)/2, np.sqrt(2)/2, 0], [-1, 1, 0], False)
        self.assertTrue(not test_motor1.inverted)

        expected = [-np.sqrt(2)/2, np.sqrt(2)/2, 0]
        observed = test_motor1.direction

        for i in range(0, len(expected)):
            self.__assertWithinRange(expected[i], observed[i], 0.02)

        test_motor1.inverted = True
        expected = [-expected[0], -expected[1], -expected[2]]
        observed = test_motor1.direction

        for i in range(0, len(expected)):
            self.__assertWithinRange(expected[i], observed[i], 0.02)

        test_motor2 = Motor('fr_inverted', [np.sqrt(2)/2, np.sqrt(2)/2, 0], [-1, 1, 0], True)
        self.assertTrue(test_motor2.inverted)

        expected = [np.sqrt(2) / 2, -np.sqrt(2) / 2, 0]
        observed = test_motor2.direction

        for i in range(0, len(expected)):
            self.__assertWithinRange(expected[i], observed[i], 0.02)
