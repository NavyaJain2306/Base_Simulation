import argparse
import threading
import traceback


try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseArray
    from std_msgs.msg import String
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


from .shapes            import get_shape, SHAPES
from .planner           import get_plan, fallback_plan
from .task_splitter     import split_plan, print_queues
from .command_generator import plan_to_commands
from .executor          import ArmExecutor

_NodeBase = Node if ROS_AVAILABLE else object


#  ROS Node 

class PlannerNode(_NodeBase):
    def __init__(self, shape_name: str = "square_2x2"):
        super().__init__("planner_node")

        self.shape_name   = shape_name
        self.target_shape = get_shape(shape_name)
        self.plan_done    = False

        self.arm_executor = ArmExecutor(mode="ros", ros_node=self)

        self.block_sub = self.create_subscription(
            PoseArray,
            "/block_poses",
            self._on_block_poses,
            10
        )

        self.get_logger().info(
            f"PlannerNode started. Shape: {shape_name}. "
            f"Waiting for /block_poses..."
        )

    def _on_block_poses(self, msg):
        """
        Callback — fires when perception node publishes block positions.
        This is the main trigger for the entire planning pipeline.
        """
        if self.plan_done:
            return  

        if not msg.poses:
            self.get_logger().warn("Received empty PoseArray. Ignoring.")
            return

        block_positions = {}
        for i, pose in enumerate(msg.poses):
            name = f"block{i + 1}"
            block_positions[name] = (
                pose.position.x,
                pose.position.y,
                pose.position.z,
            )

        self.get_logger().info(
            f"Received {len(block_positions)} blocks from perception."
        )
        for name, pos in block_positions.items():
            self.get_logger().info(f"  {name}: {pos}")

        #  Sanity check 
        if len(block_positions) < len(self.target_shape):
            self.get_logger().error(
                f"Need {len(self.target_shape)} blocks, "
                f"only got {len(block_positions)}. Waiting..."
            )
            return

        plan_thread = threading.Thread(
            target=self._plan_and_execute,
            args=(block_positions,),
            daemon=True
        )
        plan_thread.start()
        self.plan_done = True   
    def _plan_and_execute(self, block_positions: dict):
        """Runs in a background thread — full plan + execute pipeline."""

        self.get_logger().info("Running Unified Planning solver...")

        try:
            plan = get_plan(block_positions, self.target_shape)
        except Exception as e:
            self.get_logger().warn(f"UP solver error: {e}")
            self.get_logger().warn(f"Full traceback:\n{traceback.format_exc()}")  # ADD THIS
            plan = []
            
        if not plan:
            self.get_logger().warn("UP solver gave no plan. Using fallback.")
            plan = fallback_plan(block_positions, self.target_shape)

        self.get_logger().info(f"Plan ({len(plan)} actions):")
        for i, a in enumerate(plan):
            self.get_logger().info(f"  {i+1}. {a}")

        #  Split 
        arm1_plan, arm2_plan = split_plan(plan)

        #  Commands 
        arm1_commands = plan_to_commands(arm1_plan, block_positions, self.target_shape)
        arm2_commands = plan_to_commands(arm2_plan, block_positions, self.target_shape)

        self.get_logger().info(
            f"Executing: arm1={len(arm1_commands)} steps, "
            f"arm2={len(arm2_commands)} steps"
        )

        #  Execute (blocks until both arms done) 
        self.arm_executor.execute_both_arms(arm1_commands, arm2_commands)
        self.get_logger().info("All done! Shape complete.")


#  Standalone simulate mode (no ROS) 

def run_simulate(shape_name: str, slam_mode: str):
    """
    Run the full pipeline in simulate mode without ROS.
    Useful for testing your logic before connecting hardware.
    """
    from slam_reader import SLAMReader

    print("\n" + "="*60)
    print("  ROBOCUP PLANNER  [simulate mode — no ROS]")
    print("="*60)

    reader          = SLAMReader(mode=slam_mode)
    block_positions = reader.get_block_positions()
    target_shape    = get_shape(shape_name)

    print(f"\n[1] Blocks from SLAM ({slam_mode}):")
    for name, pos in block_positions.items():
        print(f"    {name}: {pos}")

    print(f"\n[2] Target shape: {shape_name}")
    for name, pos in target_shape.items():
        print(f"    {name}: {pos}")

    print("\n[3] Running UP solver...")
    try:
        plan = get_plan(block_positions, target_shape)
    except Exception as e:
        print(f"    UP error: {e}")
        plan = []

    if not plan:
        print("    Fallback plan.")
        plan = fallback_plan(block_positions, target_shape)

    print(f"\n    Plan ({len(plan)} actions):")
    for i, a in enumerate(plan):
        print(f"      {i+1}. {a}")

    print("\n[4] Splitting between arms...")
    arm1_plan, arm2_plan = split_plan(plan)
    print_queues(arm1_plan, arm2_plan)

    print("\n[5] Generating commands...")
    arm1_commands = plan_to_commands(arm1_plan, block_positions, target_shape)
    arm2_commands = plan_to_commands(arm2_plan, block_positions, target_shape)
    print(f"    arm1: {len(arm1_commands)} steps | arm2: {len(arm2_commands)} steps")

    print("\n[6] Executing (simulate)...")
    executor = ArmExecutor(mode="simulate")
    executor.execute_both_arms(arm1_commands, arm2_commands)
    print("\nDone!")


#  Entry point 

def main():
    parser = argparse.ArgumentParser(
        description="RoboCup Planner — ROS 2 node or simulate mode"
    )
    parser.add_argument("--shape", default="square_2x2",
                        choices=list(SHAPES.keys()),
                        help="Target shape")
    parser.add_argument("--simulate", action="store_true",
                        help="Run in simulate mode without ROS")
    parser.add_argument("--slam", default="hardcoded",
                        choices=["hardcoded", "live"],
                        help="SLAM source (simulate mode only)")
    args = parser.parse_args()

    if args.simulate:
        #  Simulate mode — no ROS needed 
        run_simulate(shape_name=args.shape, slam_mode=args.slam)

    else:
        #  ROS node mode 
        if not ROS_AVAILABLE:
            print("ERROR: rclpy not found. Install ROS 2 or use --simulate flag.")
            print("       python main.py --simulate --shape square_2x2")
            return

        rclpy.init()
        node = PlannerNode(shape_name=args.shape)

        try:
            rclpy.spin(node)        # keeps node alive, processing callbacks
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()