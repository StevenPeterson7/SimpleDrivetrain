import numpy as np


def rotate_vector(vector, pitch, roll, yaw):
    vec_to_matrix = np.array([vector[0],
                              vector[1],
                              vector[2]])
    rot_x_mat = np.array([[1.0, 0.0, 0.0],
                          [0.0, np.cos(pitch), -np.sin(pitch)],
                          [0.0, np.sin(pitch), np.cos(pitch)]])
    rot_y_mat = np.array([[np.cos(roll), 0.0, np.sin(roll)],
                          [0.0, 1.0, 0.0],
                          [-np.sin(roll), 0.0, np.cos(roll)]])
    rot_z_mat = np.array([[np.cos(yaw), -np.sin(yaw), 0.0],
                          [np.sin(yaw), np.cos(yaw), 0.0],
                          [0.0, 0.0, 1.0]])

    #  matrix multiplication of np.array uses np.dot
    vec_to_matrix = np.dot(rot_x_mat, vec_to_matrix)
    vec_to_matrix = np.dot(rot_y_mat, vec_to_matrix)
    vec_to_matrix = np.dot(rot_z_mat, vec_to_matrix)

    vector = np.array([vec_to_matrix[0], vec_to_matrix[1], vec_to_matrix[2]])

    return vector


def calculate_angle_direction(horizontal, vertical):
    if horizontal < -0.05:
        if vertical < -0.05:  # Quadrant III
            angle_ref = np.abs(np.arctan(vertical / horizontal))
            angle = np.pi + angle_ref
        elif vertical > 0.05:  # Quadrant II
            angle_ref = np.abs(np.arctan(vertical / horizontal))
            angle = np.pi - angle_ref
        else:
            angle = np.pi
    elif horizontal > 0.05:
        if vertical < -0.05:  # Quadrant IV
            angle_ref = np.abs(np.arctan(vertical / horizontal))
            angle = 2 * np.pi - angle_ref
        elif vertical > 0.05:  # Quadrant I
            angle = np.abs(np.arctan(vertical / horizontal))
        else:
            angle = 0.0
    else:
        if vertical < -0.05:
            angle = (3.0 * np.pi) / 2.0
        elif vertical > 0.05:
            angle = np.pi / 2.0
        else:
            angle = None
            print('No direction can be calculated for a zero vector.')

    return angle


def normalize(vector):
    norm = np.linalg.norm(vector)
    if norm == 0:
        raise ZeroDivisionError('Attempted to normalize a vector of length 0.')
    else:
        return vector / norm
