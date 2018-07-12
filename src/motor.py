import numpy as np
import vectorutils as vutil


class Motor(object):
    def __init__(self, name, position, direction, inverted=False, pwm_bounds=(0, 512, 1024), pwm_scaling_func=None):
        self.name = name
        self.__position = np.array(position)

        self.__inverted = inverted

        if inverted:
            self.__direction = -vutil.normalize(np.array(direction))
        else:
            self.__direction = vutil.normalize(np.array(direction))

        self.__angle_position = self.__calculate_angle_position()
        self.__pwm_bounds = pwm_bounds
        self.__pwm_scaling_func = pwm_scaling_func

        if abs(pwm_bounds[1] - pwm_bounds[0]) == abs(pwm_bounds[2] - pwm_bounds[1]):
            self.__symmetric = True
        else:
            self.__symmetric = False

    def __str__(self):
        strout = 'Name: ' + self.name + '\n' + 'Position: ' + str(self.__position) + '\n' + \
                 'Direction: ' + str(self.__direction) + '\n' + 'Inverted: ' + str(self.__inverted) + '\n' \
                 + 'PWM Bounds:\n'

        if (self.__pwm_bounds is not None) and (len(self.__pwm_bounds) == 3):
            strout += '\tFull Reverse: ' + str(self.__pwm_bounds[0]) + '\n'
            strout += '\tFull Stop: ' + str(self.__pwm_bounds[1]) + '\n'
            strout += '\tFull Forward: ' + str(self.__pwm_bounds[2]) + '\n'
        else:
            strout += '\tNone\n'

        strout += 'PWM Scaling Function: '
        if self.__pwm_scaling_func is None:
            strout += 'Undefined\n'
        else:
            strout += 'Defined\n'

        return strout

    def scale_velocity_to_pwm(self, velocity):
        if self.__pwm_scaling_func is not None:
            return self.__pwm_scaling_func(velocity)
        else:  # pwm scaling function is not defined
            if self.__symmetric or velocity > 0:
                return self.__pwm_bounds[1] + (velocity * abs(self.__pwm_bounds[2] - self.__pwm_bounds[1]))
            elif velocity < 0:
                return self.__pwm_bounds[1] + (velocity * abs(self.__pwm_bounds[1] - self.__pwm_bounds[0]))
            else:  # velocity == 0
                return self.__pwm_bounds[1]

    def __calculate_angle_position(self):
        angle_position = [0.0, 0.0, 0.0]
        x = self.__position[0]
        y = self.__position[1]
        z = self.__position[2]

        #  calculate yaw from robot center
        angle_position[2] = vutil.calculate_angle_direction(x, y)

        #  calculate roll from robot center
        angle_position[1] = vutil.calculate_angle_direction(x, z)

        #  calculate pitch from robot center
        angle_position[0] = vutil.calculate_angle_direction(y, z)

        return angle_position

    @property
    def direction(self):
        return self.__direction

    @direction.setter
    def direction(self, value):
        if len(value) != 3:
            raise ValueError('Motor directions must be of length 3. A direction of length ' + str(len(value))
                             + ' was passed instead.')

        self.__direction = vutil.normalize(np.array(value))
        if self.__inverted:
            self.__direction = -self.direction

    @property
    def inverted(self):
        return self.__inverted

    @inverted.setter
    def inverted(self, value):
        if (self.__inverted is False and value is True) or (self.__inverted is True and value is False):
            self.__inverted = not self.__inverted
            self.__direction = -self.__direction

    @property
    def position(self):
        return self.__position

    @position.setter
    def position(self, value):
        self.__position = value
        self.__angle_position = self.__calculate_angle_position()

    @property
    def angle_position(self):
        return self.__angle_position

    # manual setting of angle position is discouraged
    @angle_position.setter
    def angle_position(self, value):
        pass
