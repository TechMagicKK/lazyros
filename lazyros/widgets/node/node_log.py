from rcl_interfaces.msg import Log
from rclpy.node import Node
from textual.app import ComposeResult
from textual.containers import Container
from textual.widgets import RichLog
from rich.markup import escape
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile
import re    
from textual.binding import Binding
from textual.events import Focus


def escape_markup(text: str) -> str:
    """Escape text for rich markup."""
    return escape(text)


class MyRichLog(RichLog):
    BINDINGS = [
        Binding("g,g", "go_top", "Top", show=False),     # gg -> 先頭へ
        Binding("G", "go_bottom", "Bottom", show=False), # G  -> 末尾へ
        Binding("j", "scroll_down", "Down", show=False), # 1行下
        Binding("k", "scroll_up", "Up", show=False),     # 1行上
    ]

    def action_go_top(self) -> None:
        super().action_scroll_home()
        self.auto_scroll = False

    def action_go_bottom(self) -> None:
        super().action_scroll_end()
        self.auto_scroll = True

    def action_scroll_up(self) -> None:
        super().action_scroll_up()
        self.auto_scroll = False

    def action_scroll_down(self) -> None:
        super().action_scroll_down()
        self.auto_scroll = False

class LogViewWidget(Container):
    """A widget to display ROS logs from /rosout."""

    def __init__(self, ros_node: Node, **kwargs) -> None:
        super().__init__(**kwargs)
        self.ros_node = ros_node
        self.rich_log = MyRichLog(wrap=True, highlight=True, markup=True, max_lines=1000, auto_scroll=True) 
        self.log_level_styles = {
            Log.DEBUG: "[dim cyan]",
            Log.INFO: "[dim white]",
            Log.WARN: "[yellow]",
            Log.ERROR: "[bold red]",
            Log.FATAL: "[bold magenta]",
        }
        self.logs_by_node: dict[str, list[str]] = {}
        self.current_node = None
        self.selected_node = None
        qos_profile = QoSProfile(depth=10)
        self.ros_node.create_subscription(
            Log,
            '/rosout',
            self.log_callback,
            qos_profile,
            callback_group=ReentrantCallbackGroup()
        )
        self.ros_node.create_timer(0.5, self.display_logs, callback_group=ReentrantCallbackGroup())
        self._log_buffer = -1000 # for log buffer

    def compose(self) -> ComposeResult:
        yield self.rich_log

    def _level_to_char(self, level: int) -> str:
        if level == Log.DEBUG[0]: return "DEBUG" # Compare with Log.DEBUG directly
        if level == Log.INFO[0]: return "INFO"
        if level == Log.WARN[0]: return "WARN"
        if level == Log.ERROR[0]: return "ERROR"
        if level == Log.FATAL[0]: return "FATAL"
        return "?"

    def log_callback(self, msg: Log) -> None:
        """Callback to handle incoming log messages."""

        time_str = f"{msg.stamp.sec + msg.stamp.nanosec / 1e9:.6f}"
        level_style = self.log_level_styles.get(msg.level, "[dim white]")
        level_char = self._level_to_char(msg.level)
        
        escaped_msg_content = str(msg.msg).replace("[", "\\[")

        formatted_log = (
            f"{level_style}[{level_char}] "
            f"{level_style}[{time_str}] "
            f"{level_style}[{msg.name}] " 
            f"{level_style}{escaped_msg_content}[/]"
        )

        if msg.name not in self.logs_by_node:
            self.logs_by_node[msg.name] = []
        self.logs_by_node[msg.name].append(formatted_log)
            
    def display_logs(self):
        """Display logs for the currently selected node. """

        node_listview = self.app.query_one("#node-listview")
        node_name = node_listview.selected_node_name
        if node_name:
            self.selected_node = re.sub(r'^/', '', node_name).replace('/', '.')

        if not self.selected_node:
            self.rich_log.clear()
            self.rich_log.write("[bold red]No log to display.[/]")
            return

        if self.current_node != self.selected_node:
            self.current_node = self.selected_node
            self._log_buffer = -1000 # reset
            self.rich_log.clear()

        if self.current_node in self.logs_by_node:
            logs = self.logs_by_node[self.current_node][self._log_buffer:]
            self._log_buffer = len(self.logs_by_node[self.current_node])
            for log in logs:
                self.rich_log.write(log)
        else:
            self.rich_log.clear()
            self.rich_log.write(f"[yellow]No logs found for node: {self.current_node}[/]") 

