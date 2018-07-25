import numpy as np
from motor import Motor
import vectorutils as vutils
from lxml import etree


class SimpleDrivetrain(object):
    def __init__(self, orientation=(0.0, 0.0, np.pi / 2.0)):
        self.__motors = None
        self.orientation = orientation

    def load_drivetrain_from_file(self, filepath):
        root = etree.parse(filepath).getroot()

        orientation_element = root.find('orientation')
        if orientation_element is not None:
            orientation_pitch = float(orientation_element.get('pitch'))
            orientation_roll = float(orientation_element.get('roll'))
            orientation_yaw = float(orientation_element.get('yaw'))
            orientation = (orientation_pitch, orientation_roll, orientation_yaw)
            self.orientation = orientation

        for motor_element in root.findall('motor'):
            name = motor_element.get('name')

            inverted = False
            if motor_element.get('inverted') == 'True':
                inverted = True

            position_element = motor_element.find('position')
            position_x = float(position_element.get('x'))
            position_y = float(position_element.get('y'))
            position_z = float(position_element.get('z'))
            position = (position_x, position_y, position_z)

            direction_element = motor_element.find('direction')
            direction_x = float(direction_element.get('x'))
            direction_y = float(direction_element.get('y'))
            direction_z = float(direction_element.get('z'))
            direction = (direction_x, direction_y, direction_z)

            pwm_bounds_element = motor_element.find('pwm_bounds')
            pwm_reverse = int(pwm_bounds_element.get('reverse'))
            pwm_stop = int(pwm_bounds_element.get('stop'))
            pwm_forward = int(pwm_bounds_element.get('forward'))
            pwm_bounds = (pwm_reverse, pwm_stop, pwm_forward)

            self.add_new_motor(name, position, direction, inverted, pwm_bounds)

    def add_new_motor(self, name, position, direction, inverted=False, pwm_bounds=(0, 512, 1024), pwm_scaling_func=None):
        if self.__motors is None:
            self.__motors = [Motor(name, position, direction, inverted, pwm_bounds, pwm_scaling_func)]
        else:
            self.__motors.append(Motor(name, position, direction, inverted, pwm_bounds, pwm_scaling_func))

    def get_motor_by_index(self, index):
        if (self.__motors is None) or not (0 <= index < len(self.__motors)):
            raise IndexError('Attempted to access a motor with an index that is out of bounds.')
        return self.__motors[index]

    def get_motor_by_name(self, name):
        if not (self.__motors is None):
            for motor in self.__motors:
                if motor.name == name:
                    return motor

    def remove_motor_by_index(self, index):
        if (self.__motors is None) or not (0 <= index < len(self.__motors)):
            raise IndexError('Attempted to remove a motor with an index that is out of bounds.')
        else:
            self.__motors.pop(index)

    def remove_motor_by_name(self, name):
        if not (self.__motors is None):
            for motor in self.__motors:
                if motor.name == name:
                    self.__motors.remove(motor)

    #  translation = (x-axis translational velocity, y-axis translational velocity, z-axis translational velocity)
    #  rotation = (x-axis angular velocity, y-axis angular velocity, z-axis angular velocity)
    #  force_local_oriented is a boolean value
    #    If set to True, ignores current drivetrain orientation and calculates local-oriented motor values
    #    If set to False, uses current drivetrain orientation to calculate field-oriented motor values
    def get_motor_vels(self, translation, rotation, force_local_oriented=False):
        if self.__motors is None:
            raise RuntimeError("Attempted to get motor velocities for a drivetrain with an empty motor list.")

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

    #  translation = (x-axis translational velocity, y-axis translational velocity, z-axis translational velocity)
    #  rotation = (x-axis angular velocity, y-axis angular velocity, z-axis angular velocity)
    #  force_local_oriented is a boolean value
    #    If set to True, ignores current drivetrain orientation and calculates local-oriented motor values
    #    If set to False, uses current drivetrain orientation to calculate field-oriented motor values
    def get_motor_vels_scaled(self, translation, rotation, force_local_oriented=False):
        motor_vels = self.get_motor_vels(translation, rotation, force_local_oriented)

        for i in range(0, len(self.__motors)):
            motor_vels[i] = self.__motors[i].scale_velocity_to_pwm(motor_vels[i])

        return motor_vels

    @property
    def motors(self):
        return self.__motors

    @motors.setter
    def motors(self, value):
        pass

    def __str__(self):
        outstr = self.__repr__() + '\n'

        for motor in self.__motors:
            outstr += '\tMotor ' + motor.name + ':\n'
            outstr += '\t\tInverted: ' + str(motor.inverted) + '\n'
            outstr += '\t\tPosition: ' + str(motor.position) + '\n'
            outstr += '\t\tDirection: ' + str(motor.direction) + '\n'
            outstr += '\t\tPWM Bounds: ' + str(motor.pwm_bounds) + '\n'
            if motor._Motor__pwm_scaling_func is not None:
                outstr += '\t\tPWM Scaling Function: Defined\n'
            else:
                outstr += '\t\tPWM Scaling Function: Undefined\n'

        outstr += '\n'
        return outstr
