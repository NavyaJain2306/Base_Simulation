LIFT_HEIGHT   = 0.10
DEFAULT_ROLL  = 0.0
DEFAULT_PITCH = 1.5708
DEFAULT_YAW   = 0.0

def action_to_commands(action_str: str,
                       block_positions: dict,
                       target_positions: dict) -> list:
    """
    Convert one UP action string into a list of arm movement commands.

    action_str format:  "pick arm1 block1 src_block1"
                        "place arm1 block1 pos_A"
    """
    tokens      = action_str.split()
    action_type = tokens[0]   # "pick" or "place"
    arm         = tokens[1]   # "arm1" or "arm2"
    block       = tokens[2]   # "block1" etc.
    location    = tokens[3]   # "src_block1" or "pos_A"

    if action_type == "pick":
        if block not in block_positions:
            print(f"WARNING: Block '{block}' not in SLAM positions.")
            return []
        x, y, z = block_positions[block]
        return _pick_commands(x, y, z, arm, block)

    elif action_type == "place":
        if location not in target_positions:
            print(f"WARNING: Target '{location}' not found.")
            return []
        x, y, z = target_positions[location]
        return _place_commands(x, y, z, arm, block, location)

    else:
        print(f"WARNING: Unknown action type '{action_type}'")
        return []


def plan_to_commands(plan: list,
                     block_positions: dict,
                     target_positions: dict) -> list:
    """Convert a full plan (list of action strings) to a flat command list."""
    all_commands = []
    for action in plan:
        cmds = action_to_commands(action, block_positions, target_positions)
        all_commands.extend(cmds)
    return all_commands


#  ROS message conversion 

def cmd_to_ros_msg(cmd: dict):
    return {
        "x":       cmd["x"],
        "y":       cmd["y"],
        "z":       cmd["z"],
        "roll":    cmd["roll"],
        "pitch":   cmd["pitch"],
        "yaw":     cmd["yaw"],
        "gripper": cmd["gripper"],
        "label":   cmd["label"],
    }


def cmd_to_pose_msg(cmd: dict):
    try:
        from geometry_msgs.msg import Pose, Point, Quaternion
        from tf_transformations import quaternion_from_euler

        q = quaternion_from_euler(cmd["roll"], cmd["pitch"], cmd["yaw"])
        pose = Pose(
            position=Point(x=cmd["x"], y=cmd["y"], z=cmd["z"]),
            orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        )
        return pose, cmd["gripper"]

    except ImportError:
        print("WARNING: geometry_msgs or tf_transformations not available.")
        return None, cmd["gripper"]


#  Internal helpers 
def _pick_commands(x, y, z, arm, block) -> list:
    return [
        _cmd(x, y, z + LIFT_HEIGHT, "OPEN",  arm, f"approach above {block}"),
        _cmd(x, y, z,               "OPEN",  arm, f"descend to {block}"),
        _cmd(x, y, z,               "CLOSE", arm, f"grip {block}"),
        _cmd(x, y, z + LIFT_HEIGHT, "CLOSE", arm, f"lift {block}"),
    ]


def _place_commands(x, y, z, arm, block, target) -> list:
    return [
        _cmd(x, y, z + LIFT_HEIGHT, "CLOSE", arm, f"move above {target}"),
        _cmd(x, y, z,               "CLOSE", arm, f"descend to {target}"),
        _cmd(x, y, z,               "OPEN",  arm, f"release at {target}"),
        _cmd(x, y, z + LIFT_HEIGHT, "OPEN",  arm, f"retreat from {target}"),
    ]


def _cmd(x, y, z, gripper, arm, label) -> dict:
    return {
        "arm":     arm,
        "x":       round(x, 4),
        "y":       round(y, 4),
        "z":       round(z, 4),
        "roll":    DEFAULT_ROLL,
        "pitch":   DEFAULT_PITCH,
        "yaw":     DEFAULT_YAW,
        "gripper": gripper,
        "label":   label,
    }


def print_commands(commands: list):
    for i, cmd in enumerate(commands):
        print(f"  [{cmd['arm']}] Step {i+1:02d}: "
              f"({cmd['x']:.2f}, {cmd['y']:.2f}, {cmd['z']:.2f}) "
              f"gripper={cmd['gripper']:5s}  ← {cmd['label']}")

#  Quick test 
if __name__ == "__main__":
    block_pos  = {"block1": (0.32, 0.15, 0.0), "block2": (0.55, 0.30, 0.0)}
    target_pos = {"pos_A":  (0.50, 0.50, 0.0), "pos_B":  (0.50, 0.60, 0.0)}

    fake_plan = [
        "pick arm1 block1 src_block1",
        "place arm1 block1 pos_A",
        "pick arm2 block2 src_block2",
        "place arm2 block2 pos_B",
    ]

    cmds = plan_to_commands(fake_plan, block_pos, target_pos)
    print(f"Generated {len(cmds)} commands:\n")
    print_commands(cmds)

    print("\nROS JSON payload for first command:")
    import json
    print(json.dumps(cmd_to_ros_msg(cmds[0]), indent=2))