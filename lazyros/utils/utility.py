import os
import threading
import yaml
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

def _spin(executor):
    executor.spin()

def start_ros_in_thread(ros_node):
    if not rclpy.ok():
        rclpy.init()

    executor = MultiThreadedExecutor()
    executor.add_node(ros_node)

    thread = threading.Thread(target=_spin, args=(executor,), daemon=True)
    thread.start()
    return executor, thread

def stop_ros_thread(executor, thread, ros_node):
    executor.shutdown()
    ros_node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    thread.join(timeout=1.0)
