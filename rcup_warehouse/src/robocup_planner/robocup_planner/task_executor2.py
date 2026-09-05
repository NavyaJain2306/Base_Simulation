# """
# Task executor node - 2.0 - MAY BE FINAL 
# =====================================================

# Subscribes to:
#     /planned_task   (sml_messages/PlannedTask)
#     TODO ---future---    perception --> to get the cordinates of block.

# Action Client:
#     /base_command    (sml_messages/action/BaseCommand)
#     /arm_command     (sml_messages/action/ArmCommand)


# //////////////////////////////////////////////////////
#         NOTE 
#         FUTURE - CONNECT WITH EXECUTOR NODE OF 
#         WORKBENCH TO START PICKING UP TOY ONLY 
#         AFTER RECEIVING SUCCESS
# //////////////////////////////////////////////////////


# - follow the PlanStep from /planned_task
# - FETCH_BLOCK - send request to action server (base)
#               - wait for result
#               - send request to action server (arm) 
#                     to pick from workbench and place on base
#               - wait for result 

# - GOTO_WORKBENCH

# - waiting

# """

# import threading

# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionClient
# from sml_messages.msg import PlannedTask, PlanStep

# from sml_messages.action import BaseCommand

# self.current_station = None  # TO AVOID UNNECESSARY REPEATATION

# class TaskExecutorNode(Node):

#     def __init__(self):
#         super().__init__("task_executor_node_2")

#         self.task_sub = self.create_subscription(
#             PlannedTask,
#             '/planned_task',
#             self._on_planned_task,
#             10
#         )

#         self.base_client = ActionClient(
#             self,
#             BaseCommand,
#             '/base_command',
#         )

#         self.get_logger().info('Task Executor Node 2.0 started. Wating for /planned_task ...')
    
#     def _on_planned_task(self, msg: PlannedTask):
#         if not msg.steps:
#             self.get_logger().warn('Received PlannedTask with no steps.')
#             return 
        
#         self.get_logger().info(f'Received PlannedTask: {len(msg.steps)} steps.')

#         thread = threading.Thread(
#             target = self._execute_plan,
#             args = (msg.steps,),
#             daemon = True
#         )
#         thread.start()

    
#     def _execute_plan(self, steps):

#         for i, step in enumerate(steps):
            
#             self.get_logger().info(
#                 f"Step {i+1}: {step.label}"
#             )

#             success = self._execute_step(step)

#             if not success:
#                 self.get_logger().error(
#                     f'Step {i+1} failed. Stoppig Execution.'            # TODO - IN THE FUTURE DO AN ALTERNATIVE STEP INSTEAD OF QUITING.
#                 )
#                 return 
            
#         self.get_logger().info("Plan completed successfully.")

#     def _execute_step(self, step):

#         if step.action == PlanStep.FETCH_BLOCK:
#             if not self._go_to(step.station):
#                 return False
            
#             arm_success = self._execute_action(step.action, step.)
                

        

#     def _go_to(self, station):
#         self.get_logger().info(f"Going to {station}")

#         goal = BaseCommand.Goal()
#         goal.station_name = station

#         return True
    

#     def _execute_action(self, step):

#         if step.action == PlanStep.FETCH_BLOCK:
#             # arm action

#         elif step.action == PlanStep.GOTO_WORKBENCH:
#             return True

#         elif step.action == PlanStep.LOAD_TOY:
#             # arm action

#         elif step.action == PlanStep.DELIVER:
#             # arm action


    
 

#     def _navigate_to(self, station):

#         if self.current_station == station:
#             self.get_logger().info(
#                 f"Already at {station}. Skipping navigation."
#             )
#             return True

#         #result = BaseCommand.Result()
#         if result.success:
#             self.current_station = station
#             retrun True
#         retrun False



# def main(args=None):

#     rclpy.init(args=args)

#     node = TaskExecutorNode()

#     try:
#         rclpy.spin(node)
    
#     except KeyboardInterrupt:
#         pass
    
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == "__main__":
#     main()
            



            




