features = {
    "observation.state": {
        "dtype": "float32", "shape": (4,),
        "names": ["x", "y", "sin_yaw", "cos_yaw"],
    },
    "observation.environment_state": {
        "dtype": "float32", "shape": (2,), "names": ["goal_dx", "goal_dy"],
    },
    "action": {
        "dtype": "float32", "shape": (2,), "names": ["linear_x", "angular_z"],
    },
}
robot_type = "mobile_base"
repo_id = "single_o_to_rydges"
task_name = "drive from Single O to Rydges Sydney Central"
goal_xy = (-5.8, -9.5)
