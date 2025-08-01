import threading

import rclpy
from rclpy.node import Node
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Container, Horizontal, Vertical, ScrollableContainer
from textual.widgets import (
    Footer,
    Header,
    Static,
    TabbedContent,
)

from lazyros.widgets.node_list_widget import NodeListWidget
from lazyros.widgets.log_view_widget import LogViewWidget
from lazyros.widgets.info_view_widget import InfoViewWidget
from lazyros.widgets.topic_list_widget import TopicListWidget
from lazyros.widgets.service_list_widget import ServiceListWidget
from lazyros.widgets.parameter_list_widget import ParameterListWidget
from lazyros.modals.topic_info_modal import TopicInfoModal  # Import TopicInfoModal
from lazyros.modals.message_modal import MessageModal  # Import MessageModal
from lazyros.utils.utility import ros_spin_thread, signal_shutdown, load_restart_config


class LazyRosApp(App):
    """A Textual app to monitor ROS information."""

    BINDINGS = [
        ("ctrl+q", "quit", "Quit"),
    ]

    CSS_PATH = "lazyros.css"

    def __init__(self, ros_node: Node, restart_config=None):
        super().__init__()
        self.ros_node = ros_node
        self.restart_config = load_restart_config("config/restart_config.yaml")

    def on_mount(self) -> None:
        """Called when app is mounted. Perform async setup here."""
        pass

    def compose(self) -> ComposeResult:
        """Create child widgets for the app."""
        print("LazyRosApp.compose: Composing the application layout...")
        yield Header()

        with Horizontal():
            with Container(id="left-frame", classes="left-pane"):
                print("Adding left pane...")
                with Vertical():
                    with Container(classes="list-container"):
                        yield Static("Nodes", classes="frame-title")
                        yield NodeListWidget(self.ros_node, self.restart_config, id="node-list-content")
                    with ScrollableContainer(classes="list-container"):
                        yield Static("Topics", classes="frame-title")
                        yield TopicListWidget(self.ros_node, id="topic-list-content")
                    with Container(classes="list-container"):
                        yield Static("Parameters", classes="frame-title")
                        yield ParameterListWidget(self.ros_node, id="parameter-list-content")
                    with ScrollableContainer(classes="list-container"):
                        yield Static("Services", classes="frame-title")
                        yield ServiceListWidget(self.ros_node, id="service-list-content")

            with Container(id="right-frame", classes="right-pane"):
                print("Adding right pane...")
                yield Static("Logs and Info", classes="frame-title")
                with TabbedContent("Log", "Info"):
                    yield LogViewWidget(self.ros_node, id="log-view-content")
                    yield InfoViewWidget(self.ros_node, id="info-view-content")

        yield Footer()
        print("Application layout composed.")

    def action_toggle_dark(self) -> None:
        """An action to toggle dark mode."""
        self.dark = not self.dark

    def action_restart_node(self) -> None:
        """Forward restart_node action to the NodeListWidget."""
        print("LazyRosApp.action_restart_node: Forwarding action to NodeListWidget")
        node_list = self.query_one(NodeListWidget)
        if node_list:
            node_list.action_restart_node()

    def action_focus_left_pane(self) -> None:
        """Focus the left pane and highlight it."""
        left_pane: Container = self.query_one("#left-frame")
        right_pane: Container = self.query_one("#right-frame")

        left_pane.styles.border = ("heavy", "white")
        right_pane.styles.border = ("solid", "white")

        left_pane.focus()

    def action_focus_right_pane(self) -> None:
        """Focus the right pane and highlight it."""
        left_pane: Container = self.query_one("#left-frame")
        right_pane: Container = self.query_one("#right-frame")

        left_pane.styles.border = ("solid", "white")
        right_pane.styles.border = ("heavy", "white")

        right_pane.focus()

    def action_handle_topic_click(self, topic_name: str) -> None:
        """Handle clicks on topic 'links' in the InfoViewWidget."""
        print(f"Topic clicked: {topic_name}")
        self.push_screen(TopicInfoModal(topic_name))

    def action_handle_message_click(self, message_type: str) -> None:
        """Handle clicks on message type 'links' in the InfoViewWidget."""
        print(f"Message type clicked: {message_type}")
        self.push_screen(MessageModal(message_type))

    # Removed custom run_async method


def main(args=None):
    rclpy.init(args=args)
    ros_node = None
    app: LazyRosApp | None = None  # type: ignore
    ros_thread = None
    try:
        ros_node = Node("lazyros_monitor_node")

        ros_thread = threading.Thread(target=ros_spin_thread, args=(ros_node,), daemon=True)
        ros_thread.start()

        app = LazyRosApp(ros_node)
        # Run the app using its own run method, which should handle async setup
        app.run()

    except Exception as e:
        print(f"Error initializing ROS or running the TUI: {e}")
    finally:
        signal_shutdown()

        if ros_thread:
            ros_thread.join(timeout=1.0)

        if ros_node:
            ros_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

        print("LazyRos exited cleanly.")


if __name__ == "__main__":
    main()
