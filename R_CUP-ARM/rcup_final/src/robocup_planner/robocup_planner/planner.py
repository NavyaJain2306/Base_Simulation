from unified_planning.shortcuts import *
import traceback

get_environment().credits_stream = None

def build_up_problem(block_positions: dict, target_shape: dict):
    print(f"  DEBUG block_positions type: {type(block_positions)}, value: {block_positions}")
    print(f"  DEBUG target_shape type: {type(target_shape)}, value: {target_shape}")

    Block    = UserType("Block")
    Location = UserType("Location")
    Arm      = UserType("Arm")

    block_at     = Fluent("block_at",     BoolType(), block=Block, loc=Location)
    arm_free     = Fluent("arm_free",     BoolType(), arm=Arm)
    arm_holding  = Fluent("arm_holding",  BoolType(), arm=Arm, block=Block)
    target_empty = Fluent("target_empty", BoolType(), loc=Location)

    pick = InstantaneousAction("pick", arm=Arm, block=Block, location=Location)
    a, b, l = pick.arm, pick.block, pick.location
    pick.add_precondition(arm_free(a))
    pick.add_precondition(block_at(b, l))
    pick.add_effect(arm_holding(a, b), True)
    pick.add_effect(arm_free(a),       False)
    pick.add_effect(block_at(b, l),    False)

    place = InstantaneousAction("place", arm=Arm, block=Block, target=Location)
    a2, b2, t = place.arm, place.block, place.target
    place.add_precondition(arm_holding(a2, b2))
    place.add_precondition(target_empty(t))
    place.add_effect(block_at(b2, t),    True)
    place.add_effect(arm_free(a2),       True)
    place.add_effect(arm_holding(a2, b2), False)
    place.add_effect(target_empty(t),    False)

    problem = Problem("pick_and_place")
    problem.add_fluent(block_at,     default_initial_value=False)
    problem.add_fluent(arm_free,     default_initial_value=False)
    problem.add_fluent(arm_holding,  default_initial_value=False)
    problem.add_fluent(target_empty, default_initial_value=False)
    problem.add_action(pick)
    problem.add_action(place)

    blocks  = {name: Object(name, Block)                      for name in block_positions}
    sources = {f"src_{name}": Object(f"src_{name}", Location) for name in block_positions}
    targets = {name: Object(name, Location)                   for name in target_shape}
    arms    = {name: Object(name, Arm)                        for name in ["arm1", "arm2"]}

    for obj in (list(blocks.values()) + list(sources.values()) +
                list(targets.values()) + list(arms.values())):
        problem.add_object(obj)

    for arm_obj in arms.values():
        problem.set_initial_value(arm_free(arm_obj), True)

    for block_name in blocks:
        src_name = f"src_{block_name}"
        problem.set_initial_value(
            block_at(blocks[block_name], sources[src_name]), True
        )

    for target_name in targets:
        problem.set_initial_value(target_empty(targets[target_name]), True)

    block_list  = list(blocks.keys())
    target_list = list(targets.keys())
    for block_name, target_name in zip(block_list, target_list):
        problem.add_goal(
            block_at(blocks[block_name], targets[target_name])
        )

    return problem, blocks, sources, targets, arms

def parse_up_plan(result_plan) -> list:
    """
    Convert UP plan result into plain action strings.
    Example:
        pick(arm1, block1, src_block1)
    becomes:
        "pick arm1 block1 src_block1"
    """
    action_strings = []

    if hasattr(result_plan, "actions"):
        actions_list = result_plan.actions
    else:
        actions_list = list(result_plan)

    for ai in actions_list:
        action_name = ai.action.name
        params = [str(p) for p in ai.actual_parameters]
        action_strings.append(f"{action_name} {' '.join(params)}")

    return action_strings

def get_plan(block_positions: dict, target_shape: dict) -> list:
    print("  Building Unified Planning problem...")
    problem, blocks, sources, targets, arms = build_up_problem(
        block_positions, target_shape
    )

    env = get_environment()
    engine_names = list(env.factory.engines)

    print(f"  Available engines: {engine_names}")
    print(f"  Problem kind: {problem.kind}")

    if "pyperplan" not in engine_names:
        print("  ERROR: pyperplan engine not found!")
        return []

    for engine in ["pyperplan", "pyperplan-opt"]:
        try:
            with OneshotPlanner(name=engine) as planner:
                result = planner.solve(problem)

            status = result.status.name
            print(f"  Solver status: {status}")

            if status in ("SOLVED_SATISFICING", "SOLVED_OPTIMALLY"):
                print(f"  Solved with engine: {engine}")
                return parse_up_plan(result.plan)

        except Exception as e:
            print(f"  Engine {engine} failed: {e}")
            traceback.print_exc()
            continue

    return []

def fallback_plan(block_positions: dict, target_shape: dict) -> list:
    blocks  = list(block_positions.keys())
    targets = list(target_shape.keys())
    plan = []
    for i, (block, target) in enumerate(zip(blocks, targets)):
        arm = "arm1" if i % 2 == 0 else "arm2"
        plan.append(f"pick {arm} {block} src_{block}")
        plan.append(f"place {arm} {block} {target}")
    return plan

if __name__ == "__main__":
    from slam_reader import SLAMReader
    from shapes import get_shape

    reader = SLAMReader(mode="hardcoded")
    blocks = reader.get_block_positions()
    shape  = get_shape("square_2x2")

    print("Solving with Unified Planning...\n")
    plan = get_plan(blocks, shape)

    if not plan:
        print("UP solver not available, using fallback...\n")
        plan = fallback_plan(blocks, shape)

    print(f"\nFinal plan ({len(plan)} actions):")
    for i, action in enumerate(plan):
        print(f"  {i+1}. {action}")