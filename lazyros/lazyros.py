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
    TabPane,
)

from lazyros.widgets.node.node_list_widget import NodeListWidget
from lazyros.widgets.node.log_view_widget import LogViewWidget
from lazyros.widgets.node.info_view_widget import InfoViewWidget
from lazyros.widgets.topic.topic_list_widget import TopicListWidget
from lazyros.widgets.parameter.parameter_list_widget import ParameterListWidget
from lazyros.widgets.topic.echo_view_widget import EchoViewWidget
from lazyros.widgets.parameter.parameter_value_widget import ParameterValueWidget
from lazyros.widgets.parameter.parameter_info_widget import ParameterInfoWidget
from lazyros.modals.topic_info_modal import TopicInfoModal
from lazyros.modals.message_modal import MessageModal
from lazyros.utils.utility import ros_spin_thread, signal_shutdown

from textual.screen import ModalScreen


class HelpModal(ModalScreen):
    CSS = """
    HelpModal {
        align: center middle;
        layer: modal;
    }
    
    #modal-container {
        width: auto;
        height: auto;
        border: round white;
        background: $background;
    }
    """
    
    BINDINGS = [
        Binding("escape", "dismiss", "Quit Modal")
    ]
    
    def compose(self):
        help_text = (
            "Help Menu\n"
            "\n"
            "enter        Focus right window\n"
            "\[            Previous Tab\n"
            "]            Next Tab\n"
            "tab          Focus next container\n"
            "shift+tab    Focus previous container"
        )

        yield Static(help_text, id="modal-container")

class LazyRosApp(App):
    """A Textual app to monitor ROS information."""

    BINDINGS = [
        Binding("q", "quit", "Quit", show=True),
        Binding("?", "help", "Help", show=True),
        Binding("tab", "focus_next_pane", "Next Pane", show=False),
        Binding("shift+tab", "focus_previous_pane", "Previous Pane", show=False),
        Binding("[", "previous_tab", "Previous Tab", show=False),
        Binding("]", "next_tab", "Next Tab", show=False),
        Binding("enter", "focus_right_pane", "Focus Right Pane", show=False),
    ] 

    CSS_PATH = "lazyros.css"
    
    NODE_TAB_ID_LIST = ["log", "info"]
    TOPIC_TAB_ID_LIST = ["echo", "info"]
    PARAMETER_TAB_ID_LIST = ["value", "info"]

    TAB_ID_DICT = {
        "node": ["log", "info"],
        "topic": ["echo", "info"],
        "parameter": ["value", "info"],
    }

    def __init__(self, ros_node: Node):
        super().__init__()
        self.ros_node = ros_node

        self.left_pane_widgets = [
            "#node-listview",
            "#topic-listview", 
            "#parameter-listview"
        ]
        self._left_containers = ["node", "topic", "parameter"]
        self.current_pane_index = 0
        self.current_right_pane_config = "node"
        self.current_selected_topic = None
        self._topic_update_timer = None
        self.focused_pane = "left"

    def on_mount(self) -> None:
        """Called when app is mounted. Perform async setup here."""

        node_list_widget = self.query_one("#node-listview")
        if node_list_widget:
            node_list_widget.listview.focus()

    def on_key(self, event) -> None:
        """Handle key events, override default tab behavior."""

        if event.key == "tab":
            self.action_focus_next_listview()
            event.stop()
        elif event.key == "shift+tab":
            self.action_focus_previous_listview()
            event.stop()
        elif event.key == "enter" and self.focused_pane == "left":
            self.action_focus_right_pane()
            event.stop()
        elif event.key == "[":
            self.action_previous_tab()
            event.stop()
        elif event.key == "]":
            self.action_next_tab()
            event.stop()

    def compose(self) -> ComposeResult:
        """Create child widgets for the app."""

        yield Header()

        with Horizontal():
            with Container(classes="left-pane", id="left-frame"):
                with Vertical():
                    with ScrollableContainer(classes="list-container", id="node-container"):
                        yield Static("Nodes", classes="frame-title")
                        yield NodeListWidget(self.ros_node, id="node-listview")
                    with ScrollableContainer(classes="list-container", id="topic-container"):
                        yield Static("Topics", classes="frame-title")
                        yield TopicListWidget(self.ros_node, id="topic-listview")
                    with ScrollableContainer(classes="list-container", id="parameter-container"):
                        yield Static("Parameters", classes="frame-title")
                        yield ParameterListWidget(self.ros_node, id="parameter-listview")

            with Container(classes="right-pane", id="right-frame"):
                with TabbedContent("Log", "Info", id="node-tabs"):
                    with TabPane("Log", id="log"):
                        yield LogViewWidget(self.ros_node, id="log-view-content")
                    with TabPane("Info", id="info"):
                        yield InfoViewWidget(self.ros_node, id="info-view-content")
                with TabbedContent("Info", "Echo", id="topic-tabs", classes="hidden"):
                    with TabPane("Echo", id="echo"):
                        yield EchoViewWidget(self.ros_node, id="echo-view-content")
                    with TabPane("Info", id="info"):
                        yield InfoViewWidget(self.ros_node, id="topic-info-view-content")
                with TabbedContent("Info", "Value", id="parameter-tabs", classes="hidden"):
                    with TabPane("Value", id="value"):
                        yield ParameterValueWidget(self.ros_node, id="parameter-value-view-content")
                    with TabPane("Info", id="info"):
                        yield ParameterInfoWidget(self.ros_node, id="parameter-info-view-content")

        yield Footer()

    # keybindings for actions
    def action_help(self):
        """Show the help modal."""

        self.push_screen(HelpModal())

    def action_focus_right_pane(self) -> None:
        """Focus the right pane and highlight it."""

        self.focused_pane = "right"
        self._reset_frame_highlight()

    def action_focus_left_pane(self) -> None:
        """Focus the left pane and highlight it."""

        self.focused_pane = "left"
        self._reset_frame_highlight()

    def action_focus_next_listview(self) -> None:

        if self.focused_pane == "right":
            self.action_focus_left_pane()
        else:
            self.current_pane_index = (self.current_pane_index + 1) % len(self.left_pane_widgets)
            self._focus_current_listview()

    def action_focus_previous_listview(self) -> None:

        if self.focused_pane == "right":
            self.action_focus_left_pane()
        else:
            self.current_pane_index = (self.current_pane_index - 1) % len(self.left_pane_widgets)
            self._focus_current_listview()

    def _reset_frame_highlight(self) -> None:
        """Reset the visual style of the left and right panes."""
        
        right_pane: Container = self.query_one("#right-frame")
        
        if self.focused_pane == "right":
            right_pane.styles.border = ("round", "green")
            listview_container_widget = self.query_one(f'#{self._left_containers[self.current_pane_index]}-container')
            listview_container_widget.styles.border = ("round", "white")
        else:
            right_pane.styles.border = ("round", "white")
            for name in self._left_containers:
                widget = self.query_one(f'#{name}-container')
                if name == self._left_containers[self.current_pane_index]:
                    widget.styles.border = ("round", "green")
                else:
                    widget.styles.border = ("round", "white")

    def _focus_right_pane_tab(self):
        """Focus the right pane tab based on the current configuration."""
        
        left_container = self._left_containers[self.current_pane_index]
        self.query_one(f'#{left_container}-tabs').focus()
   
    def _focus_left_pane_listview(self):
        """Focus the left pane listview based on the current configuration."""
        
        focused_listview = self._left_containers[self.current_pane_index]        
        self.query_one(f'#{focused_listview}-listview').listview.focus()

    def _focus_current_listview(self) -> None:
        """Focus the current listview based on current_pane_index."""

        self._reset_frame_highlight()
        self._focus_left_pane_listview()
        self._update_right_pane()    

    def _update_right_pane(self) -> None:
        current_listview = self._left_containers[self.current_pane_index]
        
        for i in self._left_containers:
            if i == current_listview:
                self.query_one(f'#{i}-tabs').remove_class("hidden")
                self.current_right_pane_config = i
            else:
                self.query_one(f'#{i}-tabs').add_class("hidden")   

    def action_previous_tab(self):
        """Focus the right pane tab based on the current configuration."""
        
        current_listview = self._left_containers[self.current_pane_index]
        tabs = self.query_one(f'#{current_listview}-tabs')
        current_tab = tabs.active
        
        if self.TAB_ID_DICT[current_listview][0] == current_tab:
            return
        for i in self.TAB_ID_DICT[current_listview][1:]:
            if current_tab == i:
                tabs.active = self.TAB_ID_DICT[current_listview][self.TAB_ID_DICT[current_listview].index(i) - 1]
                break

    def action_next_tab(self):
        """Focus the right pane tab based on the current configuration."""
        
        current_listview = self._left_containers[self.current_pane_index]
        tabs = self.query_one(f'#{current_listview}-tabs')
        current_tab = tabs.active
        
        if self.TAB_ID_DICT[current_listview][-1] == current_tab:
            return
        for i in self.TAB_ID_DICT[current_listview][:-1]:
            if current_tab == i:
                tabs.active = self.TAB_ID_DICT[current_listview][self.TAB_ID_DICT[current_listview].index(i) + 1]
                break

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
