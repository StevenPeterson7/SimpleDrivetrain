import numpy as np
from motor import Motor
import vectorutils as vutils


class SimpleDrivetrain(object):
    def __init__(self, orientation=(0.0, 0.0, np.pi / 2.0), motor_list=[]):
        self.__motors = motor_list
        self.orientation = orientation

    def add_new_motor(self, name, position, direction, inverted=False, pwm_bounds=(0, 512, 1024), pwm_scaling_func=None):
        self.__motors.append(Motor(name, position, direction, inverted, pwm_bounds, pwm_scaling_func))

    def get_motor_by_index(self, index):
        if 0 <= index < len(self.__motors):
            return self.__motors[index]
        else:
            raise IndexError('Attempted to access a motor with an index that is out of bounds.')

    def get_motor_by_name(self, name):
        for motor in self.__motors:
            if motor.name == name:
                return motor

    def remove_motor_by_index(self, index):
        if 0 <= index < len(self.__motors):
            self.__motors.pop(index)
        else:
            raise IndexError('Attempted to remove a motor with an index that is out of bounds.')

    def remove_motor_by_name(self, name):
        for motor in self.__motors:
            if motor.name == name:
                self.__motors.remove(motor)

    #  translation = (x-axis translational velocity, y-axis translational velocity, z-axis translational velocity)
    #  rotation = (x-axis angular velocity, y-axis angular velocity, z-axis angular velocity)
    #  force_local_oriented is a boolean value
    #    If set to True, ignores current drivetrain orientation and calculates local-oriented motor values
    #    If set to False, uses current drivetrain orientation to calculate field-oriented motor values
    def get_motor_vels(self, translation, rotation, force_local_oriented=False):
        global_vector = translation
        motor_vels = [x * 0.0 for x in range(0, len(self.__motors))]
        orientation_reference_vector = np.array((0.0, 0.0, np.pi / 2.0))
        orientation_difference_vector = self.orientation - orientation_reference_vector

        for i in range(0, len(self.__motors)):
            motor_angle_pos = self.__motors[i].angle_position
            motor_dir_local = self.__motors[i].direction
            motor_dir_globalized = motor_dir_local

            if not force_local_oriented:
                motor_dir_globalized = vutils.rotate_vector(motor_dir_local, orientation_difference_vector[0],
                                                            orientation_difference_vector[1],
                                                            orientation_difference_vector[2])

            motor_vels[i] = (np.dot(motor_dir_globalized, global_vector))

            #  apply any applicable rotational velocity
            yz_angle = motor_angle_pos[0]
            xz_angle = motor_angle_pos[1]
            xy_angle = motor_angle_pos[2]

            if yz_angle is not None:
                rot_vec_ccw_x = np.array([0, -np.sin(yz_angle), np.cos(yz_angle)])
                motor_vels[i] += rotation[0] * np.dot(motor_dir_local, rot_vec_ccw_x)

            if xz_angle is not None:
                rot_vec_ccw_y = np.array([-np.sin(xz_angle), 0, np.cos(xz_angle)])
                motor_vels[i] += rotation[1] * np.dot(motor_dir_local, rot_vec_ccw_y)

            if xy_angle is not None:
                rot_vec_ccw_z = np.array([-np.sin(xy_angle), np.cos(xy_angle), 0])
                motor_vels[i] += rotation[2] * np.dot(motor_dir_local, rot_vec_ccw_z)

        #  scale each motor velocity by the maximum velocity
        mag_list = np.array(map(abs, motor_vels))
        max_mag = mag_list.max()
        if max_mag > 1.0:
            motor_vels = map(lambda x: x / max_mag, motor_vels)

        return motor_vels
