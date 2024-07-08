#!/bin/python3

# See:
# https://www.alanzucconi.com/2020/09/14/inverse-kinematics-in-3d/
# https://www.gauravmanek.com/blog/2015/hexapod-step-planner/

from gz.transport13 import Node
from gz.msgs10.double_pb2 import Double
from gz.msgs10.model_pb2 import Model

import time
import pygame
from pygame import MOUSEMOTION, MOUSEBUTTONUP, MOUSEBUTTONDOWN

from matrix import *
from ik import *
from config import CONFIG

step_time = 1000  # ms


def print_coordinates(coordinates: np.array):
    return f"X: {coordinates[0]:5.2f}, Y: {coordinates[1]:5.2f}, Z: {coordinates[2]:5.2f}"


def angles_to_position(angles: [np.double]) -> [np.double]:
    position = angles
    position[0] = -position[0]
    position[1] = position[1] - 0.44
    position[2] = -position[2]
    return position


def setup_leg_ik():
    for leg in CONFIG["legs"]:
        name = leg["name"]
        tip_in_world_frame = leg.get("start_position")
        leg["current_position"] = tip_in_world_frame

        # convert from body frame to coxa joint frame
        distance = leg.get("config").get("offset")
        angle_z = leg.get("config").get("angle_z")
        coxa_pose = get_coxa_pose(distance, angle_z)
        t_body_to_coxa = create_transformation_matrix(coxa_pose)
        t_coxa_to_body = np.linalg.inv(t_body_to_coxa)
        leg["transformation"] = t_coxa_to_body

        coords_in_body = np.matmul(t_world_to_body_frame, tip_in_world_frame)
        coords_in_coxa = np.matmul(t_coxa_to_body, coords_in_body)
        ik = calculate_angles_3d_combined(coords_in_coxa[0], coords_in_coxa[1], coords_in_coxa[2])
        leg["start_angles"] = angles_to_position([ik[0], ik[1], ik[2]])

        for y in range(3):
            leg["control_topics"][y]["pub"] = node.advertise(leg["control_topics"][y]["name"], Double())

        print(
            f"Start angles are {leg["start_angles"][0]:.2f} ({math.degrees(leg["start_angles"][0]):.2f}), {leg["start_angles"][1]:.2f} ({math.degrees(leg["start_angles"][1]):.2f}), {leg["start_angles"][2]:.2f} ({math.degrees(leg["start_angles"][2]):.2f})")


def move_to_start_position():
    for _i in range(5):
        for leg in CONFIG["legs"]:
            start_position = leg["start_angles"]
            d_upper.data = start_position[0]
            d_lower.data = start_position[1]
            d_twist.data = start_position[2]

            leg["control_topics"][0]["pub"].publish(d_twist)
            leg["control_topics"][1]["pub"].publish(d_upper)
            leg["control_topics"][2]["pub"].publish(d_lower)
        time.sleep(1)


def main_loop(t):
    # Read the joystick
    fw = 5  # joystick.get_axis(1)
    lr = 0   # joystick.get_axis(0)
    h = 0    # joystick.get_axis(3)
    r = 0    # joystick.get_axis(2)

    # We want to move body forward with respect to the world coordinates
    # hexapod_location_in_world_frame = Pose(0, 0, 100, 0, 0, 0)
    hexapod_location_in_world_frame.translate(fw, lr, h)

    t_world_to_hexapod_frame = np.linalg.inv(create_transformation_matrix(hexapod_location_in_world_frame))
    t_world_to_body_frame = np.matmul(t_world_to_hexapod_frame, np.linalg.inv(create_transformation_matrix(body_location_in_hexapod_frame)))

    for leg in CONFIG["legs"]:
        name = leg["name"]
        state = leg.get("state")
        if state is None:
            if name in ["Right Front", "Right Back", "Left Center"]:
                state = "grounded"
            else:
                state = "moving"
            leg["state"] = state

        transform = leg["transformation"]

        tip_in_world_frame = leg.get("current_position")

        if state == "moving":
            tip_in_world_frame[0] += fw * 2
            tip_in_world_frame[2] = 25
            if gait_step == 5:
                tip_in_world_frame[2] = 0
            leg["current_position"] = tip_in_world_frame

        coords_in_body = np.matmul(t_world_to_body_frame, tip_in_world_frame)
        new_coxa_coordinates = np.matmul(transform, coords_in_body)

        ik = calculate_angles_3d_combined(new_coxa_coordinates[0], new_coxa_coordinates[1], new_coxa_coordinates[2])
        target = angles_to_position([ik[0], ik[1], ik[2]])
        print(
            f"{name} angles: {math.degrees(ik[0]):.2f}, {math.degrees(ik[1]):.2f}, {math.degrees(ik[2]):.2f}")

        d_upper.data = target[0]
        d_lower.data = target[1]
        d_twist.data = target[2]

        leg["control_topics"][0]["pub"].publish(d_twist)
        leg["control_topics"][1]["pub"].publish(d_upper)
        leg["control_topics"][2]["pub"].publish(d_lower)

        if gait_step == 5:
            if state == "moving":
                leg["state"] = "grounded"
            else:
                leg["state"] = "moving"


def joint_state_callback(model: Model):
    for joint in model.joint:
        if joint.name.endswith("femur_joint"):
            continue
        # limit_lower and limit_upper
        state[joint.name] = joint.axis1.position


if __name__ == "__main__":
    # Setup pygame
    pygame.init()
    pygame.event.set_blocked((MOUSEMOTION, MOUSEBUTTONUP, MOUSEBUTTONDOWN))
    pygame.joystick.init()
    if pygame.joystick.get_count() > 0:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()

    state = {}

    node = Node()
    node.subscribe(Model, "/world/hexspider_world/model/hexspider/joint_state", joint_state_callback)

    print("Waiting for valid joint data...")
    while state.get("leg_fr_servo_1") is None or state.get("leg_fr_servo_1") == 0.0:
        time.sleep(0.1)
    print("Data valid!")

    d_upper = Double()
    d_lower = Double()
    d_twist = Double()

    # World is in ENU frame
    # Hexpod should be in vehicle frame X is in driving direction, Z upwards
    # See https://en.wikipedia.org/wiki/Axes_conventions#/media/File:RPY_angles_of_cars.png
    hexapod_location_in_world_frame = Pose(0, 0, 150, 0, 0, 0)

    # Fixed translation from hexapod to hexapod body
    body_location_in_hexapod_frame = Pose(0, 0, 0, 0, 0, 0)
    t_hexapod_body_matrix = create_transformation_matrix(body_location_in_hexapod_frame)
    t_world_to_hexapod_frame = np.linalg.inv(create_transformation_matrix(hexapod_location_in_world_frame))
    t_world_to_body_frame = np.matmul(t_world_to_hexapod_frame, np.linalg.inv(create_transformation_matrix(body_location_in_hexapod_frame)))

    setup_leg_ik()

    move_to_start_position()

    t = 0
    delta_t = 2 * math.pi / 60

    target_velocity = 50  # mm/s
    target_heading = 0  # straight ahead for now
    gait_step = 0

    hexapod_location_in_world_frame.translate(0,0, -70)
    try:
        while True:
            start = time.time_ns()
            #for event in [pygame.event.wait(), ]:
            #    pass

            print("Gait step: " + str(gait_step))

            main_loop(t)

            t = t + delta_t
            if t >= 2 * math.pi:
                t = 0

            gait_step = (gait_step + 1) % 6

            diff = time.time_ns() - start
            if diff > step_time * 1000:
                print(f"Overrun {diff}")
                continue
            else:
                print(f"Sleep {(step_time * 1000 - diff) / 1000}")

            time.sleep((step_time * 1000 - diff) / 1000000)

    except KeyboardInterrupt:
        pass
