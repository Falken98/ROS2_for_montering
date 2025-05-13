from builtin_interfaces.msg import Duration
import math

def deg_to_rad(degrees):
    return degrees * (math.pi / 180.0)

POSITIONS = {
    'home': [0.96866, -1.84237, 1.65405, -1.38090, -1.56853, -0.0],
    'over_mir': [0.36634, -0.57037, 1.26938, -2.26788, -1.56905, -1.20951],
    'over_pipe': [deg_to_rad(64.46), deg_to_rad(-66.17), deg_to_rad(71.15), deg_to_rad(-97.82), deg_to_rad(-88.56), deg_to_rad(-28.18)],
    'pipe_grip': [0.36634, -0.51417, 1.26048, -2.31588, -1.56923, -1.20934],
    'pipe_release': [deg_to_rad(65.46), deg_to_rad(-65.56), deg_to_rad(75.25), deg_to_rad(-102.53), deg_to_rad(-88.75), deg_to_rad(-28.18)],
}

TRAJECTORIES = {
    'test1':[
        {
            "positions": [0.043128, -1.28824, 1.37179, -1.82208, -1.63632, -0.18],
            "velocities": [0, 0, 0, 0, 0, 0],
            "time_from_start": Duration(sec=4, nanosec=0),
        },
        {
            "positions": [-0.195016, -1.70093, 0.902027, -0.944217, -1.52982, -0.195171],
            "velocities": [0, 0, 0, 0, 0, 0],
            "time_from_start": Duration(sec=8, nanosec=0),
        },
    ],
    'test2':[
        {
            "positions": [-0.095016, -1.70093, 0.902027, -0.944217, -1.52982, -0.195171],
            "velocities": [0, 0, 0, 0, 0, 0],
            "time_from_start": Duration(sec=4, nanosec=0),
        },
        {
            "positions": [0.143128, -1.28824, 1.37179, -1.82208, -1.63632, -0.18],
            "velocities": [0, 0, 0, 0, 0, 0],
            "time_from_start": Duration(sec=8, nanosec=0),
        },
    ],
    'to_mir_pos':[
        {
            "positions": POSITIONS['home'],
            "velocities": [0, 0, 0, 0, 0, 0],
            "time_from_start": Duration(sec=4, nanosec=0),
        },
        {
            "positions": [0.39968, -0.98419, 0.91211, -1.49889, -1.56940, -0.60895],
            "velocities": [0, 0, 0, 0, 0, 0],
            "time_from_start": Duration(sec=8, nanosec=0),
        },
        {
            "positions": POSITIONS['over_mir'],
            "velocities": [0, 0, 0, 0, 0, 0],
            "time_from_start": Duration(sec=12, nanosec=0),
        },
    ],
    'to_grip_close_pos':[
        {
            "positions": POSITIONS['pipe_grip'],
            "velocities": [0, 0, 0, 0, 0, 0],
            "time_from_start": Duration(sec=4, nanosec=0),
        },
    ],
    'from_mir_pos':[
        {
            "positions": POSITIONS['over_mir'],
            "velocities": [0, 0, 0, 0, 0, 0],
            "time_from_start": Duration(sec=4, nanosec=0),
        },
        {
            "positions": [0.39968, -0.98419, 0.91211, -1.49889, -1.56940, -0.60895],
            "velocities": [0, 0, 0, 0, 0, 0],
            "time_from_start": Duration(sec=8, nanosec=0),
        },
        {
            "positions": POSITIONS['home'],
            "velocities": [0, 0, 0, 0, 0, 0],
            "time_from_start": Duration(sec=12, nanosec=0),
        },
    ],
    'to_pipe_pos':[
        {
            "positions": POSITIONS['over_pipe'],
            "velocities": [0, 0, 0, 0, 0, 0],
            "time_from_start": Duration(sec=4, nanosec=0),
        },
    ],
    'to_grip_open_pos':[
        {
            "positions": POSITIONS['pipe_release'],
            "velocities": [0, 0, 0, 0, 0, 0],
            "time_from_start": Duration(sec=4, nanosec=0),
        },
    ],
    'from_pipe_pos':[
        {
            "positions": POSITIONS['over_pipe'],
            "velocities": [0, 0, 0, 0, 0, 0],
            "time_from_start": Duration(sec=4, nanosec=0),
        },
        {
            "positions": POSITIONS['home'],
            "velocities": [0, 0, 0, 0, 0, 0],
            "time_from_start": Duration(sec=8, nanosec=0),
        },
    ],
}