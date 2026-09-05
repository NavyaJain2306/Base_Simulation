def split_plan(plan: list) -> tuple:

    pairs = []
    i = 0
    while i < len(plan) - 1:
        action_a = plan[i]
        action_b = plan[i + 1]

        # Verify it's a pick-place pair
        if action_a.startswith("pick") and action_b.startswith("place"):
            pairs.append((action_a, action_b))
            i += 2
        else:
            i += 1  # skip malformed

    arm1_queue = []
    arm2_queue = []

    for idx, (pick_action, place_action) in enumerate(pairs):
        # Reassign arm label
        arm = "arm1" if idx % 2 == 0 else "arm2"
        pick_reassigned  = reassign_arm(pick_action,  arm)
        place_reassigned = reassign_arm(place_action, arm)

        if arm == "arm1":
            arm1_queue.extend([pick_reassigned, place_reassigned])
        else:
            arm2_queue.extend([pick_reassigned, place_reassigned])

    return arm1_queue, arm2_queue


def reassign_arm(action_str: str, new_arm: str) -> str:
    tokens = action_str.split()
    if len(tokens) >= 2:
        tokens[1] = new_arm
    return " ".join(tokens)


def print_queues(arm1_queue: list, arm2_queue: list):
    print("\nArm 1 tasks:")
    for a in arm1_queue:
        print(f"  -> {a}")

    print("\nArm 2 tasks:")
    for a in arm2_queue:
        print(f"  -> {a}")


# Quick test 
if __name__ == "__main__":
    fake_plan = [
        "pick arm1 block1 src_block1",
        "place arm1 block1 pos_A",
        "pick arm1 block2 src_block2",
        "place arm1 block2 pos_B",
        "pick arm1 block3 src_block3",
        "place arm1 block3 pos_C",
        "pick arm1 block4 src_block4",
        "place arm1 block4 pos_D",
    ]

    arm1, arm2 = split_plan(fake_plan)
    print_queues(arm1, arm2)
