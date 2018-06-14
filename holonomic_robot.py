import numpy as np
import math


class HolonomicRobot:

    def __init__(self, motor_dirs, pwm_scale_info, orientation_init=(0.0, 0.0, 0.0)):
        self.__motor_dirs = motor_dirs
        self.__pwm_scale_info = pwm_scale_info
        self.orientation = orientation_init

    def _rotate_vector(self, vector, pitch, roll, yaw):
        vec_to_matrix = np.array([vector[0],
                                  vector[1],
                                  vector[2]])
        rot_x = np.array([[1.0, 0.0, 0.0],
                          [0.0, math.cos(pitch), -math.sin(pitch)],
                          [0.0, math.sin(pitch), math.cos(pitch)]])
        rot_y = np.array([[math.cos(roll), 0.0, math.sin(roll)],
                          [0.0, 1.0, 0.0],
                          [-math.sin(roll), 0.0, math.cos(roll)]])
        rot_z = np.array([[math.cos(yaw), -math.sin(yaw), 0.0],
                          [math.sin(yaw), math.cos(yaw), 0.0],
                          [0.0, 0.0, 1.0]])

        #  matrix multiplication of np.array uses np.dot
        vec_to_matrix = np.dot(rot_x, vec_to_matrix)
        vec_to_matrix = np.dot(rot_y, vec_to_matrix)
        vec_to_matrix = np.dot(rot_z, vec_to_matrix)

        vector = [vec_to_matrix[0], vec_to_matrix[1], vec_to_matrix[2]]

        return vector

    #  translation = (x velocity, y velocity, z velocity)
    #  rotation = (x-axis angular velocity, y-axis angular velocity, z-axis angular velocity)
    def get_motor_vels_local(self, translation, rotation):
        local_vector = translation
        motor_vels = []

        for i in range(0, len(self.__motor_dirs)):
            motor_vels.append(np.dot(self.__motor_dirs[i], local_vector))

            #  TODO apply any applicable rotation

        #  scale each motor velocity by the maximum velocity
        mag_list = np.array(map(abs, motor_vels))
        max_mag = mag_list.max()
        if max_mag > 1.0:
            motor_vels = map(lambda x: x / max_mag, motor_vels)

        return motor_vels
