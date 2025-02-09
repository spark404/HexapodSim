import numpy as np

"""
ENU world frame with the X axis pointing in the primary motion axis
and the Z axis upwards from the ground

     X
    ---
   / | \
Y_/__|  \
  \     /
   \   /
    ---
    
Rotations on the Z axis, consider positive Y as 0 deg

The recorded start positions are using fixed servo angles
 - servo 1 = 0
 - servo 2 = 90 deg
 - servo 3 = 0
 
 body |--o---o
             |
             |
"""

CONFIG = {
    "legs": [
        {
            "name": "Right Front",
            "start_position": [175.299, -122.745, 0, 1],
            "config": {
                "angle_z": np.radians(-35),
                "offset": 90
            },
            "control_topics": [
                {"name": "/model/hexspider/joint/leg_fr_servo_1/0/cmd_pos"},
                {"name": "/model/hexspider/joint/leg_fr_servo_2/0/cmd_pos"},
                {"name": "/model/hexspider/joint/leg_fr_servo_3/0/cmd_pos"}
            ],
        },
        {
            "name": "Right Center",
            "start_position": [0, -194, 0, 1],
            "config": {
                "angle_z": np.radians(-90),
                "offset": 70
            },
            "control_topics": [
                {"name": "/model/hexspider/joint/leg_cr_servo_1/0/cmd_pos"},
                {"name": "/model/hexspider/joint/leg_cr_servo_2/0/cmd_pos"},
                {"name": "/model/hexspider/joint/leg_cr_servo_3/0/cmd_pos"}
            ],
        },
        {
            "name": "Right Back",
            "start_position": [-175.299, -122.745, 0, 1],
            "config": {
                "angle_z": np.radians(-145),
                "offset": 90
            },
            "control_topics": [
                {"name": "/model/hexspider/joint/leg_br_servo_1/0/cmd_pos"},
                {"name": "/model/hexspider/joint/leg_br_servo_2/0/cmd_pos"},
                {"name": "/model/hexspider/joint/leg_br_servo_3/0/cmd_pos"}
            ],
        },
        {
            "name": "Left Front",
            "start_position": [175.299, 122.745, 0, 1],
            "config": {
                "angle_z": np.radians(35),
                "offset": 90
            },
            "control_topics": [
                {"name": "/model/hexspider/joint/leg_fl_servo_1/0/cmd_pos"},
                {"name": "/model/hexspider/joint/leg_fl_servo_2/0/cmd_pos"},
                {"name": "/model/hexspider/joint/leg_fl_servo_3/0/cmd_pos"}
            ],
        },
        {
            "name": "Left Center",
            "start_position": [0, 194, 0, 1],
            "config": {
                "angle_z": np.radians(90),
                "offset": 70
            },
            "control_topics": [
                {"name": "/model/hexspider/joint/leg_cl_servo_1/0/cmd_pos"},
                {"name": "/model/hexspider/joint/leg_cl_servo_2/0/cmd_pos"},
                {"name": "/model/hexspider/joint/leg_cl_servo_3/0/cmd_pos"}
            ],
        },
        {
            "name": "Left Back",
            "start_position": [-175.299, 122.745, 0, 1],
            "config": {
                "angle_z": np.radians(145),
                "offset": 90
            },
            "control_topics": [
                {"name": "/model/hexspider/joint/leg_bl_servo_1/0/cmd_pos"},
                {"name": "/model/hexspider/joint/leg_bl_servo_2/0/cmd_pos"},
                {"name": "/model/hexspider/joint/leg_bl_servo_3/0/cmd_pos"}
            ],
        },
    ]
}
