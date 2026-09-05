# Planner uses Classical Unified planning
from unified_planning.shortcuts import *
import traceback   # CHANGED !!

# Suppress credits noise
get_environment().credits_stream = None


def build_up_problem(block_positions: dict, target_shape: dict):
    print(f"  DEBUG block_positions type: {type(block_positions)}, value: {block_positions}")
    print(f"  DEBUG target_shape type: {type(target_shape)}, value: {target_shape}")
    """
    Builds a Unified Planning Problem entirely in Python.

    Key fixes vs original:
      1. Replaced Not(target_filled) with target_empty (positive fluent)
         -- pyperplan does NOT support negation in preconditions
      2. Fixed initial state loop: use f"src_{block_name}" explicitly
         instead of zip(blocks, sources) which paired keys incorrectly
      3. Fixed goal loop: convert dict keys to lists before zipping
      4. Added explicit initial values for target_empty = True
    """

    # Types
    Block    = UserType("Block")
    Location = UserType("Location")
    Arm      = UserType("Arm")

    # Fluents
    block_at     = Fluent("block_at",     BoolType(), block=Block, loc=Location)
    arm_free     = Fluent("arm_free",     BoolType(), arm=Arm)
    arm_holding  = Fluent("arm_holding",  BoolType(), arm=Arm, block=Block)
    target_empty = Fluent("target_empty", BoolType(), loc=Location)  # FIX 1: positive fluent

    # PICK action
    pick = InstantaneousAction("pick", arm=Arm, block=Block, location=Location)
    a, b, l = pick.arm, pick.block, pick.location
    pick.add_precondition(arm_free(a))
    pick.add_precondition(block_at(b, l))
    pick.add_effect(arm_holding(a, b), True)
    pick.add_effect(arm_free(a),       False)
    pick.add_effect(block_at(b, l),    False)

    # PLACE action
    place = InstantaneousAction("place", arm=Arm, block=Block, target=Location)
    a2, b2, t = place.arm, place.block, place.target
    place.add_precondition(arm_holding(a2, b2))
    place.add_precondition(target_empty(t))          # FIX 1: no Not()
    place.add_effect(block_at(b2, t),    True)
    place.add_effect(arm_free(a2),       True)
    place.add_effect(arm_holding(a2, b2), False)
    place.add_effect(target_empty(t),    False)      # FIX 1: mark occupied

    # Problem
    problem = Problem("pick_and_place")
    problem.add_fluent(block_at,     default_initial_value=False)
    problem.add_fluent(arm_free,     default_initial_value=False)
    problem.add_fluent(arm_holding,  default_initial_value=False)
    problem.add_fluent(target_empty, default_initial_value=False)
    problem.add_action(pick)
    problem.add_action(place)

    # Objects
    blocks  = {name: Object(name, Block)                      for name in block_positions}
    sources = {f"src_{name}": Object(f"src_{name}", Location) for name in block_positions}
    targets = {name: Object(name, Location)                   for name in target_shape}
    arms    = {name: Object(name, Arm)                        for name in ["arm1", "arm2"]}

    for obj in (list(blocks.values()) + list(sources.values()) +
                list(targets.values()) + list(arms.values())):
        problem.add_object(obj)

    # Initial state: arms free
    for arm_obj in arms.values():
        problem.set_initial_value(arm_free(arm_obj), True)

    # FIX 2: correct initial block positions
    for block_name in blocks:
        src_name = f"src_{block_name}"
        problem.set_initial_value(
            block_at(blocks[block_name], sources[src_name]), True
        )

    # FIX 3 & 4: all targets start empty
    for target_name in targets:
        problem.set_initial_value(target_empty(targets[target_name]), True)

    # Goals
    block_list  = list(blocks.keys())   # FIX 3: list() before zip
    target_list = list(targets.keys())
    for block_name, target_name in zip(block_list, target_list):
        problem.add_goal(
            block_at(blocks[block_name], targets[target_name])
        )

    return problem, blocks, sources, targets, arms
    # changed  ← These lines are outside the function!
    print("  Checking problem kind...")
    print(f"  Problem kind: {problem.kind}")
    print(f"  Supported: {get_environment().factory.engines['pyperplan'].supports(problem.kind)}")

def parse_up_plan(result_plan) -> list:
    """
    Convert UP plan result into plain action strings.
    Example:
        pick(arm1, block1, src_block1)
    becomes:
        "pick arm1 block1 src_block1"
    """
    action_strings = []

    # Handle SequentialPlan safely
    if hasattr(result_plan, "actions"):
        actions_list = result_plan.actions
    else:
        actions_list = list(result_plan)

    for ai in actions_list:
        action_name = ai.action.name
        params = [str(p) for p in ai.actual_parameters]
        action_strings.append(f"{action_name} {' '.join(params)}")

    return action_strings
# def parse_up_plan(result_plan) -> list:
#     """
#     Convert UP plan result into plain action strings.
#     e.g. "pick arm1 block1 src_block1"
#     """
#     action_strings = []
#     actions_list = getattr(result_plan, 'actions', list(result_plan))
#     for ai in actions_list:
#         action_name = ai.action.name
#         params = [str(p) for p in ai.actual_parameters]
#         action_strings.append(f"{action_name} {' '.join(params)}")
#     return action_strings

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
            continue

    return []

# def get_plan(block_positions: dict, target_shape: dict) -> list:
#     """
#     Main entry point. Builds and solves the UP problem.
#     """
#     ###########
#     print("\nDEBUG INPUT CHECK")
#     print("block_positions =", block_positions)
#     print("target_shape =", target_shape)
#     print("num blocks =", len(block_positions))
#     print("num targets =", len(target_shape))

#     ###########
#     print("  Building Unified Planning problem...")
#     problem, blocks, sources, targets, arms = build_up_problem(
#         block_positions, target_shape
#     )

#     # CORRECT — works whether engines is a list or dict
#     """
#     env = get_environment()
#     available_engines = env.factory.engines  # could be list or dict
#     engine_names = list(available_engines.keys()) if hasattr(available_engines, 'keys') else list(available_engines)
#     print(f"  Available engines: {engine_names}")

#     if "pyperplan" not in engine_names:
#         print("  ERROR: pyperplan engine not found!")
#         return []
#     """    
#     supported = env.factory.engines["pyperplan"].supports(problem.kind)
#     print(f"  Problem kind: {problem.kind}")
#     print(f"  Pyperplan supports this problem: {supported}")

#     if not supported:
#         print("  ERROR: Problem features not supported by pyperplan")
#         return []

#     # Instead of hardcoding one engine, try in order of preference:
#     for engine in ["fast-downward", "pyperplan", "enhsp"]:
#         try:
#             with OneshotPlanner(name=engine) as planner:
#                 result = planner.solve(problem)
#                 status = result.status.name
#                 if status in ("SOLVED_SATISFICING", "SOLVED_OPTIMALLY"):
#                     print(f"  Solved with engine: {engine}")
#                     return parse_up_plan(result.plan)
#         except Exception:
#             print(f"  Engine {engine} failed, trying next...")
#             continue

#     return []  # all engines failed

# """
#     print("  Calling UP solver (pyperplan)...")
    
#     try:
#         with OneshotPlanner(name="pyperplan") as planner:
#             result = planner.solve(problem)

#         status = result.status.name
#         if status in ("SOLVED_SATISFICING", "SOLVED_OPTIMALLY"):
#             print(f"  Solver status: {status}")
#             return parse_up_plan(result.plan)
#         # else:
#         #     print(f"  WARNING: Solver returned: {status}")
#         #     return []

#         # Changed
#         else:
#             print(f"  WARNING: Solver returned: {status}")
#             print(f"  Full result object: {result}")
#             print(f"  Plan: {result.plan}")
#             return []

#     except Exception as e:
#         print(f"ERROR during planning:\n{traceback.format_exc()}")
#         return []
# """

def fallback_plan(block_positions: dict, target_shape: dict) -> list:
    """
    Manual sequential plan if UP solver fails.
    Alternates between arm1 and arm2.
    """
    blocks  = list(block_positions.keys())
    targets = list(target_shape.keys())
    plan = []
    for i, (block, target) in enumerate(zip(blocks, targets)):
        arm = "arm1" if i % 2 == 0 else "arm2"
        plan.append(f"pick {arm} {block} src_{block}")
        plan.append(f"place {arm} {block} {target}")
    return plan


# Quick test
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