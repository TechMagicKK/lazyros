import subprocess

from rclpy.node import Node
from rclpy.action import graph
from textual.app import ComposeResult
from textual.containers import Container, Horizontal
from textual.widgets import RichLog, Button, Label
from rich.markup import escape
from dataclasses import dataclass

import rclpy
from lifecycle_msgs.srv import GetState, ChangeState, GetAvailableTransitions
from lifecycle_msgs.msg import State as LifecycleState
from rclpy.callback_groups import ReentrantCallbackGroup
from typing import List

def escape_markup(text: str) -> str:
    """Escape text for rich markup."""
    return escape(text)

@dataclass
class LifecycleData:
    is_lifecycle: bool
    get_lifecycle_client: callable
    change_lifecycle_client: callable
    get_transition_client: callable
    current_lifecycle_id: int = None 
    current_transition_id: List[int] = None
    state_changed: bool = False

class LifecycleWidget(Container):
    """Widget for displaying ROS node information."""

    DEFAULT_CSS = """
    #lifecycle-state {
        height: 1fr;      /* 通常は全体に広がる */
        max-height: 30%;  /* でも画面の 30% まで */
    }
    #lifecycle-transitions > Button {
        margin: 0 1;
        padding: 0 2;
        height: 3;
        min-width: 10;

        background: black;
        color: white;
        border: round white;
    }
    #lifecycle-transitions > Button:hover {
        background: white;
        color: black;
    }
    #lifecycle-transitions > Button:focus {
        border: heavy white;
    }
    """

    def __init__(self, ros_node: Node, **kwargs) -> None:
        super().__init__(**kwargs)
        self.ros_node = ros_node # May not be directly used if info comes from subprocess
        self.info_log = RichLog(wrap=True, highlight=True, markup=True, id="lifecycle-state", max_lines=1000)
        self.lifecycle_dict: dict[str, list[str]] = {} # Cache for node info
        
        self.selected_node_data = None
        self.current_node_full_name = None

    def compose(self) -> ComposeResult:
        yield self.info_log
        yield Horizontal(id="lifecycle-transitions")

    def on_mount(self) -> None:
        self.set_interval(0.5, self.update_display)  # Update info every 0.5 seconds

    def update_transition_buttons(self):
        for button in self.query(Button):
            if button.id and button.id.startswith("transition-button-"):
                button.remove()
        for transition in self.get_available_transitions():
            self.query_one("#lifecycle-transitions").mount(
                Button(transition.transition.label, id=f"transition-button-{transition.transition.id}")
            )

    def update_display(self):
        node_listview = self.app.query_one("#node-listview")
        self.selected_node_data = node_listview.node_listview_dict["/"+node_listview.selected_node_name]

        if self.selected_node_data is None:
            self.info_log.clear()
            self.info_log.write("[red]No node is selected yet.[/]")
            return

        if self.selected_node_data.status != "green":
            self.info_log.clear()
            self.info_log.write("[red]Selected node is shutdown.[/]")

        # Node is the same, no need to update
        #if self.selected_node_data.full_name == self.current_node_full_name:
        #    return
               
        self.info_log.clear()
        self.current_node_full_name = self.selected_node_data.full_name
        if self.selected_node_data.full_name not in self.lifecycle_dict:
            self.create_lifecycle_data()
        
        if not self.lifecycle_dict[self.selected_node_data.full_name].is_lifecycle:
            transition_buttons = self.query_one("#lifecycle-transitions")
            transition_buttons.display = False
            return self.info_log.write(f"[red]Node {self.selected_node_data.full_name} is not a lifecycle node.[/]")
        
        transition_buttons = self.query_one("#lifecycle-transitions")
        transition_buttons.display = True 
        info_lines = self.get_lifecycle_state()
        self.info_log.write("\n".join(info_lines))

        if self.lifecycle_dict[self.selected_node_data.full_name].state_changed:
            self.update_transition_buttons()

    def create_lifecycle_data(self) -> bool:
        node = self.selected_node_data.node_name
        namespace = self.selected_node_data.namespace

        is_lifecycle = False
        srvs = self.ros_node.get_service_names_and_types_by_node(node, namespace)
        for srv in srvs:
            if "lifecycle_msgs/srv/GetState" in srv[1]:
                is_lifecycle = True
                break
            
        if is_lifecycle:
            self.lifecycle_dict[self.selected_node_data.full_name] = \
                LifecycleData(is_lifecycle=True,
                            get_lifecycle_client=self.ros_node.create_client(GetState, f"{self.selected_node_data.full_name}/get_state", callback_group=ReentrantCallbackGroup()),
                            change_lifecycle_client=self.ros_node.create_client(ChangeState, f"{self.selected_node_data.full_name}/change_state", callback_group=ReentrantCallbackGroup()),
                            get_transition_client=self.ros_node.create_client(GetAvailableTransitions, f"{self.selected_node_data.full_name}/get_available_transitions", callback_group=ReentrantCallbackGroup()))
        else:
            self.lifecycle_dict[self.selected_node_data.full_name] = \
                LifecycleData(is_lifecycle=False,
                            get_lifecycle_client=None,
                            change_lifecycle_client=None,
                            get_transition_client=None)

    def get_lifecycle_state(self) -> None:
        """If the node is a lifecycle node, get its state."""
        
        full_name = self.selected_node_data.full_name
        
        lifecycle_client = self.lifecycle_dict[full_name].get_lifecycle_client
        if not lifecycle_client.wait_for_service(timeout_sec=1.0):
            return f"[red]Lifecycle service for {full_name} is not available[/]"

        req = GetState.Request()
        future = lifecycle_client.call_async(req)
        self.ros_node.executor.spin_until_future_complete(future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            return f"[red]Failed to get lifecycle state for {full_name}[/]"

        current = future.result().current_state
        if self.lifecycle_dict[full_name].current_lifecycle_id != current.id:
            self.lifecycle_dict[full_name].state_changed = True
            self.lifecycle_dict[full_name].current_lifecycle_id = current.id
        else:
            self.lifecycle_dict[full_name].state_changed = False

        info_lines = []
        info_lines.append(f"Lifecycle State for {full_name}:")
        info_lines.append(f"  {current.label}[{current.id}]")
        return info_lines

    # change lifecycle state
    def get_available_transitions(self):
        full_name = self.selected_node_data.full_name
        
        get_transition_client = self.lifecycle_dict[full_name].get_transition_client
        if not get_transition_client.wait_for_service(timeout_sec=1.0):
            return f"[red]Lifecycle service for {full_name} is not available[/]"

        while not get_transition_client.wait_for_service(timeout_sec=1.0):
            self.ros_node.get_logger().info('get_available_transitions service not available, waiting again...')

        request = GetAvailableTransitions.Request()
        future = get_transition_client.call_async(request)
        self.ros_node.executor.spin_until_future_complete(future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            return f"[red]Failed to get available transitions for {full_name}[/]"

        transitions = future.result().available_transitions
        return transitions

    def on_button_pressed(self, event: Button.Pressed) -> None:
        if event.button.id.startswith("transition-button-"):
            transition_id = int(event.button.id.split("-")[-1])
            self.trigger_transition(transition_id)

    def trigger_transition(self, transition_id: int):
        full_name = self.selected_node_data.full_name

        change_lifecycle_client = self.lifecycle_dict[full_name].change_lifecycle_client
        if not change_lifecycle_client.wait_for_service(timeout_sec=1.0):
            return f"[red]Lifecycle service for {full_name} is not available[/]"

        request = ChangeState.Request()
        request.transition.id = transition_id
        future = change_lifecycle_client.call_async(request)
        self.ros_node.executor.spin_until_future_complete(future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            return f"[red]Failed to change lifecycle state for {full_name}[/]"
        if not future.result().success:
            return f"[red]Failed to change lifecycle state for {full_name}[/]"
