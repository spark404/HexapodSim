import numpy as np
import math


def calculate_angles_3d_combined(x: np.double, y: np.double, z: np.double):
    # X is horizontal, Y is vertical from the joint
    # Z is plane distance between origin and target

    # A is our origin for this calculation
    # B is an unknown point of the joing linking legs a and c
    # C is the actual point we want to use
    # D is means to calculate, we only know its y and z component
    origin = [0, 0, 0]
    target = [x, y, z]
    D = [np.nan, 0, z]

    coxa = 24 # mm
    femur = 100 # mm
    tibia = 150 # mm

    # we calculate the hip angle using the z and x difference
    # between target and origin. Which is a right sided triangle
    # so arctan will give us the answer.
    delta_x = target[0] - origin[0]
    delta_z = target[2] - origin[2]
    omega = np.arctan2(delta_z, delta_x)

    # We know three things
    #  - the distance from origin to target (pythagoras), side c
    #  - the height difference between origin to target, side a
    #  - the corner opposite to the distance is 90 deg
    # we need to take the coxa length into account so our origin
    femur_origin = origin
    femur_origin[0] += coxa
    len_femur_origin_target = math.dist(femur_origin, target)

    # using the above we can calculate the length of the remaining side b
    b = np.sqrt(len_femur_origin_target ** 2 - delta_z ** 2)

    # using a right sided triangle with side delta_y and b
    # we calculate the downwards parts of angle a
    delta_y = target[1] - D[1]
    angle_a_accent = np.arcsin(delta_y / b)

    # Use the law of cosines to find the remaining angles
    # as we know all sides, but none of the angles
    # c**2 = a**2 + b**2 - 2ab * cos C
    angle_a = np.arccos((femur**2 + b**2 - tibia**2) / (2 * femur * b)) + angle_a_accent
    angle_b = np.pi - np.arccos((tibia**2 + femur**2 - b**2) / (2 * femur * tibia))

    return [angle_a, angle_b, omega]
