import unittest
import vectorutils as vutil
import numpy as np


class TestCaseVectorUtils(unittest.TestCase):
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

    def test_rotate_vector(self):

        test_inputs = (([1.0, 1.0, 1.0], 0.0, 0.0, np.pi / 2.0),
                       ([1.0, 1.0, 1.0], np.pi / 2.0, 0.0, 0.0),
                       ([1.0, 1.0, 1.0], 0.0, np.pi / 2.0, 0.0),
                       ([1.0, 1.0, 1.0], np.pi / 2.0, np.pi / 2.0, np.pi / 2.0))
        test_outputs = ([-1.0, 1.0, 1.0],
                        [1.0, -1.0, 1.0],
                        [1.0, 1.0, -1.0],
                        [1.0, 1.0, -1.0])

        for i in range(0, len(test_inputs)):
            observed = vutil.rotate_vector(test_inputs[i][0], test_inputs[i][1], test_inputs[i][2], test_inputs[i][3])
            expected = test_outputs[i]

            for j in range(0, len(expected)):
                self.assertAlmostEqual(observed[j], expected[j])

    def test_normalize(self):
        test_inputs = (np.array([0, 0, 5]),
                       np.array([5, 5, 5]))
        test_outputs = (np.array([0, 0, 1]),
                        np.array([5 / np.sqrt(75) for x in range(0, 2)]))

        for i in range(0, len(test_inputs)):
            expected = test_outputs[i]
            observed = vutil.normalize(test_inputs[i])

            for j in range(0, len(expected)):
                self.assertAlmostEqual(expected[j], observed[j])
