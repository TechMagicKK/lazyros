import subprocess
import threading
import time

from rclpy.node import Node
from textual.app import ComposeResult
from textual.containers import Container
from textual.widgets import RichLog
from rich.markup import escape
from rich.text import Text as RichText
from rclpy.callback_groups import ReentrantCallbackGroup
from rosidl_runtime_py.utilities import get_message
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile
from textual.binding import Binding

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


class EchoViewWidget(Container):
    """Widget for displaying ROS topic echo messages."""

    DEFAULT_CSS = """
    EchoViewWidget {
        overflow-y: scroll;
    }
    """

    def __init__(self, ros_node: Node, **kwargs) -> None:
        super().__init__(**kwargs)
        self.ros_node = ros_node
        self.rich_log = MyRichLog(wrap=True, highlight=True, markup=True, id="echo-log", max_lines=1000)
        self.current_topic = None

        self.topic_listview = None
        self.echo_dict = None
        self._sub = None

        self.ros_node.create_timer(1, self.update_display, callback_group=ReentrantCallbackGroup())

    def compose(self) -> ComposeResult:
        yield self.rich_log

    def update_display(self):
        self.topic_listview = self.app.query_one("#topic-listview")
        self.selected_topic = self.topic_listview.selected_topic if self.topic_listview else None

        if self.selected_topic is None:
            self.rich_log.clear()
            self.rich_log.write("[red]No topic is selected yet.[/]")
            return

        if self.selected_topic == self.current_topic:
            return

        self.current_topic = self.selected_topic
        self.rich_log.clear()
        self.start_echo()

    def start_echo(self):
        if self._sub is not None:
            self.ros_node.destroy_subscription(self._sub)

        topic_dict = self.topic_listview.topic_dict if self.topic_listview else None
        if not topic_dict:
            return [f"[red]Topic {escape_markup(self.current_topic)} is not set.[/]"]


        topic_type = topic_dict.get(self.current_topic, None)
        if not topic_type:
            return [f"[red]Topic {escape_markup(self.current_topic)} is not valid.[/]"]

        self.rich_log.write(f"[bold]Echoing topic: {escape_markup(self.current_topic)}, {topic_type[0]}[/bold]") 
        try:
            msg_type = get_message(topic_type[0])
        except Exception as e:
            self.rich_log.write(f"[red]Failed to get message type for {escape_markup(self.current_topic)}: {escape_markup(str(e))}[/]")
            return 

        self._sub = self.ros_node.create_subscription(msg_type, self.current_topic, self.echo_callback, qos_profile=QoSProfile(depth=1), callback_group=ReentrantCallbackGroup())

    def echo_callback(self, msg):
        message = f"[dim]Message from {escape_markup(self.current_topic)}: [/dim] {escape_markup(str(msg))}"
        self.rich_log.write(message)
