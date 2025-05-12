from builtin_interfaces.msg import Duration


POSITIONS = {
    'home': [0.96866, -1.84237, 1.65405, -1.38090, -1.56853, -0.0],
    'over_mir': [0.09983, -0.34348, 1.24564, -2.47278, -1.56800, 0.31119],
    'over_pipe': [1.35961, -1.17931, 1.37183, -1.76366, -1.57004, 1.56975],
    'pipe_grip': [0.09721, -0.22218, 1.17042, -2.51868, -1.56800, 0.31119],
    'pipe_release': [1.35944, -1.06029, 1.58441, -2.09509, -1.56888, -1.57045],
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