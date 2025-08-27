import re
import asyncio
from rcl_interfaces.srv import GetParameters, SetParameters

from rclpy.node import Node
from textual.app import ComposeResult
from textual.containers import Container
from textual.widgets import RichLog
from rich.markup import escape
from dataclasses import dataclass

import rclpy
from rcl_interfaces.msg import ParameterType

from rclpy.callback_groups import ReentrantCallbackGroup


PARAMETER_TYPE_MAP = {
    ParameterType.PARAMETER_BOOL: "bool_value",
    ParameterType.PARAMETER_INTEGER: "integer_value",
    ParameterType.PARAMETER_DOUBLE: "double_value",
    ParameterType.PARAMETER_STRING: "string_value",
    ParameterType.PARAMETER_BYTE_ARRAY: "byte_array_value",
    ParameterType.PARAMETER_BOOL_ARRAY: "bool_array_value",
    ParameterType.PARAMETER_INTEGER_ARRAY: "integer_array_value",
    ParameterType.PARAMETER_DOUBLE_ARRAY: "double_array_value",
    ParameterType.PARAMETER_STRING_ARRAY: "string_array_value",
}

def escape_markup(text: str) -> str:
    """Escape text for rich markup."""
    return escape(text)

@dataclass
class ParameterClients:
    get_parameter: None
    set_parameter: None

class ParameterValueWidget(Container):
    """Widget for displaying ROS parameter values."""

    DEFAULT_CSS = """
    ParameterValueWidget {
        overflow-y: scroll;
    }
    """

    def __init__(self, ros_node: Node, **kwargs) -> None:
        super().__init__(**kwargs)
        self.ros_node = ros_node
        self.rich_log = RichLog(wrap=True, highlight=True, markup=True, id="parameter-value-log", max_lines=1000)
        self.param_client_dict = {}
        self.listview_widget = None

        self.current_parameter = None
        self.selected_parameter = None

        self.ros_node.create_timer(1, self.update_display, callback_group=ReentrantCallbackGroup())

    def compose(self) -> ComposeResult:
        yield self.rich_log

    def update_display(self):
        self.listview_widget = self.app.query_one("#parameter-listview")
        self.selected_parameter = self.listview_widget.selected_param if self.listview_widget else None

        if not self.selected_parameter:
            self.rich_log.clear()
            self.rich_log.write("[red]No parameter is selected yet.[/]")
            return

        if self.selected_parameter == self.current_parameter:
            return

        self.current_parameter = self.selected_parameter
        self.rich_log.clear()
        value_lines = self.show_param_value()
        self.rich_log.write("\n".join(value_lines))

    def show_param_value(self):
        match = re.fullmatch(r"([^:]+):\s*(.+)", self.current_parameter)
        if not match:
            return [f"[red]Invalid parameter format: {escape_markup(self.current_parameter)}[/]"]
        
        node_name = match.group(1).strip()
        param_name = match.group(2).strip()

        if node_name not in self.param_client_dict:
            get_param_client = self.ros_node.create_client(GetParameters, f"{node_name}/get_parameters", callback_group=ReentrantCallbackGroup())
            set_param_client = self.ros_node.create_client(SetParameters, f"{node_name}/set_parameters", callback_group=ReentrantCallbackGroup())
            self.param_client_dict[node_name] = ParameterClients(get_parameter=get_param_client, 
                                                                 set_parameter=set_param_client)
        else:
            get_param_client = self.param_client_dict[node_name].get_parameter
            set_param_client = self.param_client_dict[node_name].set_parameter
        
        req = GetParameters.Request()
        req.names = [param_name]
        future = get_param_client.call_async(req)
        self.ros_node.executor.spin_until_future_complete(future, timeout_sec=1.0)
        if not future.done() or future.result() is None:
            return [f"[red]Failed to get parameter: {escape_markup(param_name)}[/]"] 

        res = future.result().values[0]
        if res.type == ParameterType.PARAMETER_NOT_SET:
            return [f"[red]Parameter {escape_markup(param_name)} is not set.[/]"]

        field = PARAMETER_TYPE_MAP.get(res.type, None)
        value = getattr(res, field, None)
        if field is None or value is None:
            return [f"[red]Unsupported parameter type for {escape_markup(param_name)}[/]"]

        value_lines = []
        value_lines.append(f"[bold]Parameter Value for {escape_markup(param_name)}:[/bold]")
        value_lines.append(f"[bold]Type:[/] {escape_markup(field)}")
        value_lines.append(f"[green]Value: {escape_markup(str(value))}[/green]")
        return value_lines
