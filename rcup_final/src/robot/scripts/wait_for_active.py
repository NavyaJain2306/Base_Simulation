#!/usr/bin/env python3
"""
wait_for_active.py

Blocks until the given lifecycle-managed node reports state == ACTIVE,
then exits with code 0. Used as a launch-time "barrier" so that a later
stage of a launch file only starts once an earlier stage is really up
(not just "process spawned").

Usage:
    wait_for_active.py <node_name> [namespace] [timeout_sec]

Example:
    wait_for_active.py amcl
    wait_for_active.py velocity_smoother /robot1 120
"""
import sys
import time

import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import GetState

ACTIVE_STATE_ID = 3  # lifecycle_msgs/msg/State.PRIMARY_STATE_ACTIVE


def main():
    if len(sys.argv) < 2:
        print('Usage: wait_for_active.py <node_name> [namespace] [timeout_sec]')
        sys.exit(1)

    node_name = sys.argv[1]
    namespace = sys.argv[2] if len(sys.argv) > 2 and sys.argv[2] != '' else ''
    timeout_sec = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0  # 0 = no timeout

    ns_prefix = f'/{namespace.strip("/")}' if namespace else ''
    service_name = f'{ns_prefix}/{node_name}/get_state'

    rclpy.init()
    node = Node('wait_for_active_' + node_name.replace('/', '_'))
    client = node.create_client(GetState, service_name)

    node.get_logger().info(f'Waiting for "{service_name}" to report ACTIVE...')

    start_time = time.time()

    try:
        while rclpy.ok():
            if timeout_sec > 0.0 and (time.time() - start_time) > timeout_sec:
                node.get_logger().error(
                    f'Timed out waiting for {node_name} to become ACTIVE.')
                sys.exit(1)

            if not client.wait_for_service(timeout_sec=1.0):
                node.get_logger().info(
                    f'Service {service_name} not available yet, retrying...')
                continue

            req = GetState.Request()
            future = client.call_async(req)
            rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)

            if future.done() and future.result() is not None:
                state_id = future.result().current_state.id
                state_label = future.result().current_state.label
                node.get_logger().info(
                    f'{node_name} current state: {state_label} ({state_id})')
                if state_id == ACTIVE_STATE_ID:
                    node.get_logger().info(f'{node_name} is ACTIVE. Continuing launch.')
                    break

            time.sleep(0.5)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(0)


if __name__ == '__main__':
    main()
