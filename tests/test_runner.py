import unittest
from test_case_simple_drivetrain import TestCaseSimpleDrivetrain
from test_case_motor import TestCaseMotor
from test_case_vectorutils import TestCaseVectorUtils

if __name__ == '__main__':
    test_case_motor_suite = unittest.TestLoader().loadTestsFromTestCase(TestCaseMotor)
    test_case_simple_drivetrain_suite = unittest.TestLoader().loadTestsFromTestCase(TestCaseSimpleDrivetrain)
    test_case_vectorutils_suite = unittest.TestLoader().loadTestsFromTestCase(TestCaseVectorUtils)

    unittest.TextTestRunner(verbosity=2).run(test_case_vectorutils_suite)
    unittest.TextTestRunner(verbosity=2).run(test_case_motor_suite)
    unittest.TextTestRunner(verbosity=2).run(test_case_simple_drivetrain_suite)
