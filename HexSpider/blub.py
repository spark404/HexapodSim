import math
import numpy as np

from ik import calculate_angles_3d_combined
from matrix import *
from config import CONFIG


def print_coordinates(coordinates: np.array):
    return f"X: {coordinates[0]:5.2f}, Y: {coordinates[1]:5.2f}, Z: {coordinates[2]:5.2f}"


if __name__ == "__main__":
    legs = CONFIG["legs"]

    # Based on the start condition where the hexpod
    # is configured with the coxa joint at 0, the femur joint at 0
    # and the tibia joint at 90. So effectively 150mm from the
    # ground (femur length)

    # World is in ENU frame
    # Hexpod should be in vehicle frame X is in driving direction, Z upwards
    # See https://en.wikipedia.org/wiki/Axes_conventions#/media/File:RPY_angles_of_cars.png
    hexapod_location_in_world_frame = Pose(0, 0, 150, 0, 0, 0)

    # Fixed translation from hexapod to hexapod body
    body_location_in_hexapod_frame = Pose(0, 0, 0, 0, 0, 0)
    t_hexapod_body_matrix = create_transformation_matrix(body_location_in_hexapod_frame)

    # testing
    print(f"Hexapod location in World frame {hexapod_location_in_world_frame}")
    print(f"Body location in Hexapod frame {body_location_in_hexapod_frame}")

    t_world_to_hexapod_frame = np.linalg.inv(create_transformation_matrix(hexapod_location_in_world_frame))
    print(f"World location in Hexapod frame {print_coordinates(np.matmul(t_world_to_hexapod_frame, np.array([0, 0, 0, 1])))}")

    t_world_to_body_frame = np.matmul(t_world_to_hexapod_frame, np.linalg.inv(create_transformation_matrix(body_location_in_hexapod_frame)))
    test_coord_in_body = np.matmul(t_world_to_body_frame, np.array([0, 0, 0, 1]))
    print(f"World location in body frame {print_coordinates(test_coord_in_body)}")

    for leg in legs:
        name = leg["name"]
        tip_in_world_frame = leg.get("start_position")
        print(f"{name}: Leg tip in world frame: {print_coordinates(tip_in_world_frame)}")

        coords_in_hexapod = np.matmul(t_world_to_hexapod_frame, tip_in_world_frame)
        print(f"{name}: Leg tip in hexapod frame: {print_coordinates(coords_in_hexapod)}")

        coords_in_body = np.matmul(t_world_to_body_frame, tip_in_world_frame)
        print(f"{name}: Leg tip in body frame: {print_coordinates(coords_in_body)}")

        # Build a translation matrix translating body frame to coxa frame
        distance = leg.get("config").get("offset")
        angle_z = leg.get("config").get("angle_z")
        coxa_pose = get_coxa_pose(distance, angle_z)
        print(f"{name}: Coxa in body frame: {coxa_pose}")
        t_body_to_coxa = create_transformation_matrix(coxa_pose)

        # Invert the body to coxa frame and determine the leg position in
        # the body frame
        t_coxa_to_body = np.linalg.inv(t_body_to_coxa)
        leg["transformation"] = t_coxa_to_body
        coords_in_coxa = np.matmul(t_coxa_to_body, coords_in_body)
        print(f"{name}: Leg tip in coxa frame: {print_coordinates(coords_in_coxa)}")

        ik = calculate_angles_3d_combined(coords_in_coxa[0], coords_in_coxa[1], coords_in_coxa[2])
        print(
            f"{name}: Angles Servo 1 {np.degrees(ik[0]):5.2f}, Servo 2 {np.degrees(ik[1]):5.2f}, Servo 3 {np.degrees(ik[2]):5.2f}")

        # Test the reverse
        test_coord_in_body = np.matmul(t_body_to_coxa, coords_in_coxa)
        print(f"{name}: Test Leg tip in body frame: {print_coordinates(test_coord_in_body)}")
        print(f"{name}: Test Coxa in body frame: {print_coordinates(np.matmul(t_body_to_coxa, np.array([0,0,0,1])))}")


    print("Test Movement")
    # now we want to move the hexapod forwards 50 mm
    hexapod_location_in_world_frame.translate(50, 0, 0)

    t_world_to_hexapod_frame = np.linalg.inv(create_transformation_matrix(hexapod_location_in_world_frame))
    print(f"World location in Hexapod frame {print_coordinates(np.matmul(t_world_to_hexapod_frame, np.array([0, 0, 0, 1])))}")

    t_world_to_body_frame = np.matmul(t_world_to_hexapod_frame, np.linalg.inv(create_transformation_matrix(body_location_in_hexapod_frame)))
    test_coord_in_body = np.matmul(t_world_to_body_frame, np.array([0, 0, 0, 1]))
    print(f"World location in body frame {print_coordinates(test_coord_in_body)}")

    for leg in legs:
        name = leg["name"]
        tip_in_world_frame = leg.get("start_position")
        transform = leg["transformation"]

        # Use the new world translation to convert the leg position
        coords_in_body = np.matmul(t_world_to_body_frame, tip_in_world_frame)
        print(f"{name}: Next Leg tip in body frame: {print_coordinates(coords_in_body)}")

        # Translate the tip coordinates in body frame to this legs coxa frame
        new_coxa_coordinates = np.matmul(transform, coords_in_body)
        print(f"{name}: Next Tip in coxa frame from new body: {print_coordinates(new_coxa_coordinates)}")

        ik = calculate_angles_3d_combined(new_coxa_coordinates[0], new_coxa_coordinates[1], new_coxa_coordinates[2])
        print(f"{name}: Angles Servo 1 {np.degrees(ik[0]):5.2f}, Servo 2 {np.degrees(ik[1]):5.2f}, Servo 3 {np.degrees(ik[2]):5.2f}")

        print(
            f"{name}: Test Coxa in body frame: {print_coordinates(np.matmul(np.linalg.inv(transform), np.array([0, 0, 0, 1])))}")
