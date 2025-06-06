from builtin_interfaces.msg import Duration
import math

def deg_to_rad(degrees:float) -> float:
    return degrees * (math.pi / 180.0)

POSITIONS = {
    'home': [0.96866, -1.84237, 1.65405, -1.38090, -1.56853, -0.0],
    'over_mir': [deg_to_rad(17.17), deg_to_rad(-32.59), deg_to_rad(68.07), deg_to_rad(-125.46), deg_to_rad(-89.94), deg_to_rad(-71.52)],
    'over_pipe': [deg_to_rad(64.27), deg_to_rad(-64.47), deg_to_rad(68.05), deg_to_rad(-93.92), deg_to_rad(-87.89), deg_to_rad(-29.30)],
    'pipe_grip': [deg_to_rad(17.17), deg_to_rad(-27.45), deg_to_rad(67.48), deg_to_rad(-130.02), deg_to_rad(-89.94), deg_to_rad(-71.52)],
    'pipe_release': [deg_to_rad(64.27), deg_to_rad(-63.90), deg_to_rad(72.59), deg_to_rad(-99.03), deg_to_rad(-87.87), deg_to_rad(-29.29)],
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
            "time_from_start": Duration(sec=2, nanosec=0),
        },
        {
            "positions": [0.39968, -0.98419, 0.91211, -1.49889, -1.56940, -0.60895],
            "velocities": [0, 0, 0, 0, 0, 0],
            "time_from_start": Duration(sec=4, nanosec=0),
        },
        {
            "positions": POSITIONS['over_mir'],
            "velocities": [0, 0, 0, 0, 0, 0],
            "time_from_start": Duration(sec=6, nanosec=0),
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
            "time_from_start": Duration(sec=6, nanosec=0),
        },
        {
            "positions": POSITIONS['home'],
            "velocities": [0, 0, 0, 0, 0, 0],
            "time_from_start": Duration(sec=8, nanosec=0),
        },
    ],
    'to_pipe_pos':[
        {
            "positions": POSITIONS['over_pipe'],
            "velocities": [0, 0, 0, 0, 0, 0],
            "time_from_start": Duration(sec=2, nanosec=0),
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
            "time_from_start": Duration(sec=6, nanosec=0),
        },
    ],
}