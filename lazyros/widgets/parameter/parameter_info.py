import re
import asyncio

from rclpy.node import Node
from textual.app import ComposeResult
from textual.containers import Container
from textual.widgets import RichLog
from rich.markup import escape

from rcl_interfaces.srv import DescribeParameters
from rcl_interfaces.msg import ParameterType
from rclpy.callback_groups import ReentrantCallbackGroup

def escape_markup(text: str) -> str:
    """Escape text for rich markup."""
    return escape(text)

PARAMETER_TYPE_MAP = {
    ParameterType.PARAMETER_BOOL: "bool",
    ParameterType.PARAMETER_INTEGER: "integer",
    ParameterType.PARAMETER_DOUBLE: "double",
    ParameterType.PARAMETER_STRING: "string",
    ParameterType.PARAMETER_BYTE_ARRAY: "byte_array",
    ParameterType.PARAMETER_BOOL_ARRAY: "bool_array",
    ParameterType.PARAMETER_INTEGER_ARRAY: "integer_array",
    ParameterType.PARAMETER_DOUBLE_ARRAY: "double_array",
    ParameterType.PARAMETER_STRING_ARRAY: "string_array",
}

class ParameterInfoWidget(Container):
    """Widget for displaying ROS parameter information."""

    DEFAULT_CSS = """
    ParameterInfoWidget {
        overflow-y: scroll;
    }
    """

    def __init__(self, ros_node: Node, **kwargs) -> None:
        super().__init__(**kwargs)

        self.ros_node = ros_node
        self.rich_log = RichLog(wrap=True, highlight=True, markup=True, id="parameter-info-log", max_lines=1000)
        self.param_client_dict = {}
        self.listview_widget = None

        self.current_parameter = None
        self.select_parameter = None

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
        info_lines = self.show_param_info()
        self.rich_log.write("\n".join(info_lines))

    def show_param_info(self):
        match = re.fullmatch(r"([^:]+):\s*(.+)", self.current_parameter)
        if not match:
            return [f"[red]Invalid parameter format: {escape_markup(self.current_parameter)}[/]"]
        
        node_name = match.group(1).strip()
        param_name = match.group(2).strip()

        if node_name not in self.param_client_dict:
            client = self.ros_node.create_client(DescribeParameters, f"{node_name}/describe_parameters", callback_group=ReentrantCallbackGroup())
            self.param_client_dict[node_name] = client
        else:
            # TODO: info of same param can be saved instead of calling everytime.
            client = self.param_client_dict[node_name]

        req = DescribeParameters.Request()
        req.names = [param_name]
        future = client.call_async(req)
        self.ros_node.executor.spin_until_future_complete(future, timeout_sec=1.0)
        if not future.done() or future.result() is None:
            return [f"[red]Failed to get parameter info for: {escape_markup(self.current_parameter)}[/]"]   

        res = future.result().descriptors[0]
        if res.type == ParameterType.PARAMETER_NOT_SET:
            return [f"[red]Parameter {escape_markup(param_name)} is not set.[/]"]

        field = PARAMETER_TYPE_MAP.get(res.type, None)
        if field is None:
            return [f"[red]Unsupported parameter type for {escape_markup(param_name)}[/]"]
        description = res.description if res.description else "No description available."        

        info_lines = []
        info_lines.append(f"[bold]Parameter name: {escape_markup(param_name)}[/bold]")
        info_lines.append(f"[bold]Type:[/] {escape_markup(field)}")
        info_lines.append(f"[bold]Description:[/] {escape_markup(description)}")
        return info_lines
