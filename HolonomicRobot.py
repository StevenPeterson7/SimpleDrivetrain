import numpy as np
import math


class HolonomicRobot:
    #  HolonomicRobot's constructor takes the following as parameters:
    #    motor_pwm_range_tuple - a tuple containing 3 subtuples which defines the reverse, stop, and forward pwm values
    #                            for each motor, like the following:
    #                              ((FULL_REVERSE_1, FULL_REVERSE_2, FULL_REVERSE_N),
    #                               (FULL_STOP_1, FULL_STOP_2, FULL_STOP_N),
    #                               (FULL_FORWARD_1, FULL_FORWARD_2, FULL_FORWARD_N))
    #    motor_direction_vectors - np.array with 3D unit vectors representing the forward direction of each motor
    def __init__(self, motor_pwm_range_tuple, motor_direction_vectors):
        self.__motor_pwm_range_tuple = motor_pwm_range_tuple
        self.__motor_direction_tuple = motor_direction_vectors

        #  the indices are in order of Cartesian quadrants, as illustrated by the following diagram:
        #  1      |      0
        #       / |  _
        #      /  | |\
        #    |/_  |   \
        #  -------|-------
        #    \    |   _
        #     \   |   /|
        #     _\| |  /
        #  2      |      3
        self.__CCW_direction_tuple = ((-math.sqrt(2.0)/2.0, math.sqrt(2.0)/2.0, 0.0),
                                          (-math.sqrt(2.0) / 2.0, -math.sqrt(2.0) / 2.0, 0.0),
                                          (math.sqrt(2.0) / 2.0, -math.sqrt(2.0) / 2.0, 0.0),
                                          (math.sqrt(2.0) / 2.0, math.sqrt(2.0) / 2.0, 0.0))

    #  rotates a 3D vector about the z-axis (yaw) and about the x-axis (pitch)
    #  for all rotation, CCW is positive
    def __rotate_vector(self, vector, rot_yaw, rot_pitch):
        #  first rotate yaw
        vector[0] = (vector[0] * math.cos(rot_yaw)) - (vector[1] * math.sin(rot_yaw))
        vector[1] = (vector[0] * math.sin(rot_yaw)) + (vector[1] * math.cos(rot_yaw))

        #  then rotate pitch
        vector[1] = (vector[1] * math.cos(rot_pitch)) - (vector[2] * math.sin(rot_pitch))
        vector[2] = (vector[1] * math.sin(rot_pitch)) + (vector[2] * math.cos(rot_pitch))

        return vector

    #  rotation requires that CCW rotation values are positive, and a 3D translation vector is passed
    def get_motor_velocities_local_oriented(self, translation_vector, rotation):
        motor_vels = np.array([0.0 * x for x in range(0, len(self.__motor_direction_tuple))])

        for i in range(0, len(motor_vels)):
            x = self.__motor_direction_tuple[i][0]
            y = self.__motor_direction_tuple[i][1]
            z = self.__motor_direction_tuple[i][2]
            current_dir_vector = np.array([x, y, z])
            #  apply translation vector
            motor_vels[i] = np.dot(current_dir_vector, translation_vector)

            #  apply any applicable rotation
            if x < 0:
                if y > 0:
                    yaw_rot_vector = np.array([self.__CCW_direction_tuple[0][0], self.__CCW_direction_tuple[0][1]])
                    motor_vels[i] += rotation * np.dot(self.__CCW_direction_tuple, current_dir_vector)
                elif y < 0:
                    yaw_rot_vector = np.array([self.__CCW_direction_tuple[1][0], self.__CCW_direction_tuple[1][1]])
                    motor_vels[i] += rotation * np.dot(self.__CCW_direction_tuple, current_dir_vector)
            elif x > 0:
                if y < 0:
                    yaw_rot_vector = np.array([self.__CCW_direction_tuple[2][0], self.__CCW_direction_tuple[2][1]])
                    motor_vels[i] += rotation * np.dot(self.__CCW_direction_tuple, current_dir_vector)
                elif y > 0:
                    yaw_rot_vector = np.array([self.__CCW_direction_tuple[3][0], self.__CCW_direction_tuple[3][1]])
                    motor_vels[i] += rotation * np.dot(self.__CCW_direction_tuple, current_dir_vector)

            #  scale each motor velocity by the maximum velocity
            max_mag = max(map(abs, motor_vels))
            if max_mag > 1.0:
                motor_vels = map(lambda x: x / max_mag, motor_vels)

        return motor_vels

    #  rotation_vector = [yaw_rotation, pitching]
    #  for yaw and pitch, CCW is positive
    def get_motor_velocities_field_oriented(self, translation_vector, rotation_vector, yaw, pitch):
        motor_vels = np.array([0.0 * x for x in range(0, len(self.__motor_direction_tuple))])

        for i in range(0, len(motor_vels)):
            x = self.__motor_direction_tuple[i][0]
            y = self.__motor_direction_tuple[i][1]
            z = self.__motor_direction_tuple[i][2]

            #  Rotate each of the robot's local direction vectors according to
            #  the robot's direction in the global plane as an offset.
            #  This assumes the gyro considers CCW to be positive. If not, negate the yaw and/or pitch.
            current_dir_vector = self.__rotate_vector([x, y, z], -yaw, -pitch)
            current_dir_vector = np.array(current_dir_vector)

            #  apply translation vector
            motor_vels[i] = np.dot(current_dir_vector, translation_vector)

            #  apply any applicable rotation about the z-axis
            if x < 0:
                if y > 0:
                    yaw_rot_vector = np.array([self.__CCW_direction_tuple[0][0], self.__CCW_direction_tuple[0][1]])
                    motor_vels[i] += rotation_vector[0] * np.dot(yaw_rot_vector, current_dir_vector)
                elif y < 0:
                    yaw_rot_vector = np.array([self.__CCW_direction_tuple[1][0], self.__CCW_direction_tuple[1][1]])
                    motor_vels[i] += rotation_vector[0] * np.dot(yaw_rot_vector, current_dir_vector)
            elif x > 0:
                if y < 0:
                    yaw_rot_vector = np.array([self.__CCW_direction_tuple[2][0], self.__CCW_direction_tuple[2][1]])
                    motor_vels[i] += rotation_vector[0] * np.dot(yaw_rot_vector, current_dir_vector)
                elif y > 0:
                    yaw_rot_vector = np.array([self.__CCW_direction_tuple[3][0], self.__CCW_direction_tuple[3][1]])
                    motor_vels[i] += rotation_vector[0] * np.dot(yaw_rot_vector, current_dir_vector)

            #  scale each motor velocity by the maximum velocity
            max_mag = max(map(abs, motor_vels))
            if max_mag > 1.0:
                motor_vels = map(lambda x: x / max_mag, motor_vels)

        return motor_vels

    def get_pwm_motor_states_local(self, translation_vector, rotation):
        motor_vels = self.get_motor_velocities_local_oriented(translation_vector, rotation)
        for i in range(0, len(motor_vels)):
            if motor_vels[i] < 0:
                motor_range = abs(self.__motor_pwm_range_tuple[1][i] - self.__motor_pwm_range_tuple[0][i])
                motor_vels[i] = self.__motor_pwm_range_tuple[1][i] + (motor_vels[i] * motor_range)
            elif motor_vels[i] > 0:
                motor_range = abs(self.__motor_pwm_range_tuple[1][i] - self.__motor_pwm_range_tuple[2][i])
                motor_vels[i] = self.__motor_pwm_range_tuple[1][i] + (motor_vels[i] * motor_range)
        return motor_vels

    def get_pwm_motor_states_global(self, translation_vector, rotation, heading):
        motor_vels = self.get_motor_velocities_field_oriented(translation_vector, rotation, heading)
        for i in range(0, len(motor_vels)):
            if motor_vels[i] < 0:
                motor_range = abs(self.__motor_pwm_range_tuple[1][i] - self.__motor_pwm_range_tuple[0][i])
                motor_vels[i] = self.__motor_pwm_range_tuple[1][i] + (motor_vels[i] * motor_range)
            elif motor_vels[i] > 0:
                motor_range = abs(self.__motor_pwm_range_tuple[1][i] - self.__motor_pwm_range_tuple[2][i])
                motor_vels[i] = self.__motor_pwm_range_tuple[1][i] + (motor_vels[i] * motor_range)
        return motor_vels
