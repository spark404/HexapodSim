import numpy as np

from location import Pose


def rotation_matrix_x(alpha):
    cos_alpha = np.cos(alpha)
    sin_alpha = np.sin(alpha)

    rotation_matrix = np.array([
        [1, 0, 0, 0],
        [0, cos_alpha, -sin_alpha, 0],
        [0, sin_alpha, cos_alpha, 0],
        [0, 0, 0, 1],
    ])

    return rotation_matrix


def rotation_matrix_y(beta):
    cos_beta = np.cos(beta)
    sin_beta = np.sin(beta)

    rotation_matrix = np.array([
        [cos_beta, 0, sin_beta, 0],
        [0, 1, 0, 0],
        [-sin_beta, 0, cos_beta, 0],
        [0, 0, 0, 1]
    ])

    return rotation_matrix


def rotation_matrix_z(theta):
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)

    rotation_matrix = np.array([
        [cos_theta, -sin_theta, 0, 0],
        [sin_theta, cos_theta, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ])

    return rotation_matrix


def create_transformation_matrix(pose: Pose) -> np.array:
    """
      :param pose: The pose use to create this transformation matrix in X Y Z R P Y
    """

    translation_matrix = np.array([
        [1, 0, 0, pose.coordinates()[0]],
        [0, 1, 0, pose.coordinates()[1]],
        [0, 0, 1, pose.coordinates()[2]],
        [0, 0, 0, 1]
    ])

    return translation_matrix @ (rotation_matrix_z(pose.rotation()[2]) @ rotation_matrix_x(pose.rotation()[0]))


def get_coxa_pose(length: np.double, angle: np.double) -> Pose:
    """
      Return the translation matrix for a leg given an angle and
      length between the mount point and the center of gravity.
      The coxa frame is rotated so the distance outward from the coxa is X,
      the direction up from the coxa is Y

      :param length: distance from center of the reference frame at angle
      :param angle: rotation on Z axis in radians

    """
    alpha = np.pi/2 - abs(angle)

    x = length * np.sin(alpha)
    y = np.sqrt(length**2 - x**2)

    # make sure the coordinates end up in the right quadrants
    if angle < 0:
        y *= -1

    translate_x = x
    translate_y = y
    translate_z = 0.
    rotation_angle_z = angle

    return Pose(translate_x, translate_y, translate_z, np.pi/2, 0, rotation_angle_z)
