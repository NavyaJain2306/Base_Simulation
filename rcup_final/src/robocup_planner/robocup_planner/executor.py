import json
import time
import threading

class ArmExecutor:
    def __init__(self, mode: str = "simulate", ros_node=None):
        self.mode     = mode
        self.ros_node = ros_node
        self._print_lock = threading.Lock()

        if mode == "ros":
            self._init_ros()

    def _init_ros(self):
        from std_msgs.msg import String

        node = self.ros_node
        if node is None:
            raise RuntimeError(
                "ArmExecutor(mode='ros') requires ros_node= to be set.\n"
                "Pass your rclpy Node: ArmExecutor(mode='ros', ros_node=self)"
            )

        self.arm1_pub = node.create_publisher(String, "/arm1/command", 10)
        self.arm2_pub = node.create_publisher(String, "/arm2/command", 10)

        self.arm1_ack = threading.Event()
        self.arm2_ack = threading.Event()

        self.arm1_sub = node.create_subscription(
            String, "/arm1/ack",
            lambda msg: self._on_ack("arm1", msg),
            10
        )
        self.arm2_sub = node.create_subscription(
            String, "/arm2/ack",
            lambda msg: self._on_ack("arm2", msg),
            10
        )

        node.get_logger().info("Executor: ROS publishers and ACK subscribers ready.")

    def _on_ack(self, arm_name: str, msg):
        """Called when an arm publishes 'DONE' to its /ack topic."""
        if msg.data.strip().upper() == "DONE":
            if arm_name == "arm1":
                self.arm1_ack.set()
            else:
                self.arm2_ack.set()

    def execute_both_arms(self, arm1_commands: list, arm2_commands: list):

        print("\n" + "="*60)
        print("EXECUTING — Both arms running in parallel")
        print("="*60)

        t1 = threading.Thread(
            target=self._run_arm, args=("arm1", arm1_commands)
        )
        t2 = threading.Thread(
            target=self._run_arm, args=("arm2", arm2_commands)
        )

        t1.start()
        t2.start()

        t1.join()
        t2.join()

        print("\n" + "="*60)
        print("ALL ACTIONS COMPLETE")
        print("="*60)

    def _run_arm(self, arm_name: str, commands: list):

        if not commands:
            print(f"[{arm_name}] No commands assigned.")
            return

        for i, cmd in enumerate(commands):
            self._send_command(arm_name, cmd, step=i + 1, total=len(commands))

    def _send_command(self, arm_name: str, cmd: dict, step: int, total: int):
        """Dispatch to simulate or ROS sender."""
        if self.mode == "simulate":
            self._simulate_send(arm_name, cmd, step, total)
        elif self.mode == "ros":
            self._ros_send(arm_name, cmd, step, total)
        else:
            raise ValueError(f"Unknown mode '{self.mode}'; expected 'simulate' or 'ros'.")

    def _simulate_send(self, arm_name: str, cmd: dict, step: int, total: int):
        with self._print_lock:
            print(f"  [{arm_name}] {step:02d}/{total:02d} | "
                  f"pos=({cmd['x']:.2f},{cmd['y']:.2f},{cmd['z']:.2f}) "
                  f"rpy=({cmd['roll']:.2f},{cmd['pitch']:.2f},{cmd['yaw']:.2f}) "
                  f"gripper={cmd['gripper']:5s}  ← {cmd['label']}")
        time.sleep(0.05)

    def _ros_send(self, arm_name: str, cmd: dict, step: int, total: int):
        from std_msgs.msg import String

        ack_event = self.arm1_ack if arm_name == "arm1" else self.arm2_ack
        ack_event.clear()

        payload = {
            "x":       cmd["x"],
            "y":       cmd["y"],
            "z":       cmd["z"],
            "roll":    cmd["roll"],
            "pitch":   cmd["pitch"],
            "yaw":     cmd["yaw"],
            "gripper": cmd["gripper"],
            "label":   cmd["label"],
        }

        pub = self.arm1_pub if arm_name == "arm1" else self.arm2_pub
        msg = String()
        msg.data = json.dumps(payload)
        pub.publish(msg)

        self.ros_node.get_logger().info(
            f"[{arm_name}] {step:02d}/{total:02d} sent: {cmd['label']}"
        )

        ack_received = ack_event.wait(timeout=10.0)

        if not ack_received:
            self.ros_node.get_logger().warn(
                f"[{arm_name}] TIMEOUT waiting for ACK on step {step}. "
                f"Moving to next step anyway."
            )
        else:
            self.ros_node.get_logger().info(
                f"[{arm_name}] ACK received. Step {step} done."
            )

if __name__ == "__main__":
    from command_generator import _cmd

    arm1_cmds = [
        _cmd(0.32, 0.15, 0.10, "OPEN",  "arm1", "approach block1"),
        _cmd(0.32, 0.15, 0.00, "OPEN",  "arm1", "descend to block1"),
        _cmd(0.32, 0.15, 0.00, "CLOSE", "arm1", "grip block1"),
        _cmd(0.32, 0.15, 0.10, "CLOSE", "arm1", "lift block1"),
        _cmd(0.50, 0.50, 0.10, "CLOSE", "arm1", "move to pos_A"),
        _cmd(0.50, 0.50, 0.00, "CLOSE", "arm1", "descend pos_A"),
        _cmd(0.50, 0.50, 0.00, "OPEN",  "arm1", "release block1"),
        _cmd(0.50, 0.50, 0.10, "OPEN",  "arm1", "retreat"),
    ]
    arm2_cmds = [
        _cmd(0.55, 0.30, 0.10, "OPEN",  "arm2", "approach block2"),
        _cmd(0.55, 0.30, 0.00, "OPEN",  "arm2", "descend to block2"),
        _cmd(0.55, 0.30, 0.00, "CLOSE", "arm2", "grip block2"),
        _cmd(0.55, 0.30, 0.10, "CLOSE", "arm2", "lift block2"),
        _cmd(0.50, 0.60, 0.10, "CLOSE", "arm2", "move to pos_B"),
        _cmd(0.50, 0.60, 0.00, "CLOSE", "arm2", "descend pos_B"),
        _cmd(0.50, 0.60, 0.00, "OPEN",  "arm2", "release block2"),
        _cmd(0.50, 0.60, 0.10, "OPEN",  "arm2", "retreat"),
    ]

    executor = ArmExecutor(mode="simulate")
    executor.execute_both_arms(arm1_cmds, arm2_cmds)