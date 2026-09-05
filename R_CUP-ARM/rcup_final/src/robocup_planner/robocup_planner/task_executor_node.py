import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sml_messages.msg import PlannedTask, Order, Station 

NAV_TIMEOUT = 120.0


class TaskExecutorNode(Node):

    def __init__(self):
        super().__init__('task_executor_node')

        self.pub = self.create_publisher(String, '/go_to_shelf_cmd', 10)

        self._nav_result = None
        self._nav_lock = threading.Event()

        self.result_sub = self.create_subscription(
            String, '/go_to_shelf_result', self._on_nav_result, 10
        )
        self.sub = self.create_subscription(
            PlannedTask, '/planned_task', self._on_planned_task, 10
        )

        self.get_logger().info(
            'TaskExecutorNode started. Waiting for /planned_task ...'
        )

    def _on_nav_result(self, msg: String):
        self._nav_result = msg.data
        self._nav_lock.set()
        self.get_logger().info(f'Nav result received: {msg.data}')

    def _on_planned_task(self, msg: PlannedTask):
        if not msg.order_list:
            self.get_logger().warn('Received empty order list.')
            return
        self.get_logger().info(
            f'Received planned task: {len(msg.order_list)} order(s), '
            f'{len(msg.arena_layout)} stations.'
        )
        thread = threading.Thread(
            target=self._run_execution, args=(msg,), daemon=True
        )
        thread.start()

    def _run_execution(self, msg: PlannedTask):
        storage_map = self._build_storage_map(msg.arena_layout)
        customer_map = self._build_customer_map(msg.arena_layout)
        workbench = self._get_workbench(msg.arena_layout)
        if workbench is None:
            self.get_logger().warn('No workbench found, using wb_top.')
            workbench = 'wb_top'

        self.get_logger().info(f'Storage map: {storage_map}')
        self.get_logger().info(f'Workbench: {workbench}')

        for order in msg.order_list:
            self._execute_order(order, storage_map, customer_map, workbench)

    def _execute_order(self, order, storage_map, customer_map, workbench):
        product_id = order.product_id
        block_sequence = self._decode_product(product_id)

        self.get_logger().info(
            f'Executing order: product_id={product_id}, '
            f'type={order.order_type}, blocks={block_sequence}'
        )

        if order.order_type == Order.OT_PRODUCE:
            for bt in block_sequence:
                station = storage_map.get(bt)
                if station is None:
                    self.get_logger().error(
                        f'No storage station for block type {bt}. Skipping.'
                    )
                    continue
                if not self._navigate_to(station, f'fetch block_type{bt}'):
                    self.get_logger().error('Aborting order.')
                    return

            if not self._navigate_to(workbench, 'go to workbench for assembly'):
                self.get_logger().error('Aborting order.')
                return

            if not self._navigate_to('cc_1', 'deliver product'):
                self.get_logger().error('Aborting order.')
                return

        elif order.order_type == Order.OT_RECYCLE:
            if not self._navigate_to('cc_1', 'fetch product for recycle'):
                self.get_logger().error('Aborting order.')
                return

            if not self._navigate_to(workbench, 'go to workbench for disassembly'):
                self.get_logger().error('Aborting order.')
                return

            for bt in reversed(block_sequence):
                station = storage_map.get(bt)
                if station is None:
                    self.get_logger().error(
                        f'No storage station for block type {bt}. Skipping.'
                    )
                    continue
                if not self._navigate_to(station, f'return block_type{bt}'):
                    self.get_logger().error('Aborting order.')
                    return

        self.get_logger().info(
            f'Order product_id={product_id} completed successfully.'
        )

    def _navigate_to(self, station_name: str, reason: str = '') -> bool:
        self.get_logger().info(
            f'  => navigating to: "{station_name}"'
            + (f'  ({reason})' if reason else '')
        )

        self._nav_result = None
        self._nav_lock.clear()

        msg = String()
        msg.data = station_name
        self.pub.publish(msg)

        arrived = self._nav_lock.wait(timeout=NAV_TIMEOUT)

        if not arrived:
            self.get_logger().error(
                f'TIMEOUT waiting for result from "{station_name}"'
            )
            return False

        if self._nav_result == f'{station_name}:success':
            self.get_logger().info(f'  => reached "{station_name}" successfully')
            return True
        else:
            self.get_logger().error(
                f'  => FAILED for "{station_name}": {self._nav_result}'
            )
            return False

    def _build_storage_map(self, stations):
        storage_map = {}
        for station in stations:
            if station.station_type == Station.ST_STORAGE:
                for bt in station.material_ids:
                    if bt not in storage_map:
                        storage_map[bt] = station.name
        return storage_map

    def _build_customer_map(self, stations):
        customer_map = {}
        for station in stations:
            if station.station_type == Station.ST_CUSTOMER:
                for bt in station.material_ids:
                    if bt not in customer_map:
                        customer_map[bt] = station.name
        return customer_map

    def _get_workbench(self, stations):
        for station in stations:
            if station.station_type == Station.ST_WORKBENCH:
                return station.name
        return None

    def _decode_product(self, product_id: int) -> list:
        return [int(d) for d in str(product_id)]

def main(args=None):
    rclpy.init(args=args)
    node = TaskExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()