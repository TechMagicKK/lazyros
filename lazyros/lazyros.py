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
    
    #title {
        dock: top;
        width: 100%;
        text-align: center;
        padding: 1;
    }

    #modal-container {
        width: 40%;
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
    TOPIC_TAB_ID_LIST = ["info", "echo"]
    PARAMETER_TAB_ID_LIST = ["info", "value"]

    def __init__(self, ros_node: Node):
        super().__init__()
        self.ros_node = ros_node

        self.left_pane_widgets = [
            "#node-list-content",
            "#topic-list-content", 
            "#parameter-list-content"
        ]
        self.left_container_widgets = [
            "#node-list-container",
            "#topic-list-container", 
            "#parameter-list-container"
        ]
        self.current_pane_index = 0
        self.current_right_pane_config = "default"
        self.current_selected_topic = None
        self._topic_update_timer = None
        self.focused_pane = "left"

    def on_mount(self) -> None:
        """Called when app is mounted. Perform async setup here."""
        # Set initial focus to the first (Nodes) list
        node_list_widget = self.query_one("#node-list-content")
        if node_list_widget:
            node_list_widget.node_list_view.focus()

    def on_key(self, event) -> None:
        """Handle key events, override default tab behavior."""

        if event.key == "tab":
            self.action_focus_next_pane()
            event.stop()
        elif event.key == "shift+tab":
            self.action_focus_previous_pane()
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
            with Container(id="left-frame", classes="left-pane"):
                with Vertical():
                    with ScrollableContainer(classes="list-container", id="node-list-container"):
                        yield Static("Nodes", classes="frame-title")
                        yield NodeListWidget(self.ros_node, id="node-list-content")
                    with ScrollableContainer(classes="list-container", id="topic-list-container"):
                        yield Static("Topics", classes="frame-title")
                        yield TopicListWidget(self.ros_node, id="topic-list-content")
                    with ScrollableContainer(classes="list-container", id="parameter-list-container"):
                        yield Static("Parameters", classes="frame-title")
                        yield ParameterListWidget(self.ros_node, id="parameter-list-content")

            with Container(id="right-frame", classes="right-pane"):
                yield Static("Logs and Info", classes="frame-title", id="right-pane-title")
                with TabbedContent("Log", "Info", id="default-tabs"):
                    with TabPane("Log", id="log"):
                        yield LogViewWidget(self.ros_node, id="log-view-content")
                    with TabPane("Info", id="info"):
                        yield InfoViewWidget(self.ros_node, id="info-view-content")
                with TabbedContent("Info", "Echo", id="topic-tabs", classes="hidden"):
                    with TabPane("Info", id="info"):
                        yield InfoViewWidget(self.ros_node, id="topic-info-view-content")
                    with TabPane("Echo", id="echo"):
                        yield EchoViewWidget(self.ros_node, id="echo-view-content")
                with TabbedContent("Info", "Value", id="parameter-tabs", classes="hidden"):
                    with TabPane("Info", id="info"):
                        yield ParameterInfoWidget(self.ros_node, id="parameter-info-view-content")
                    with TabPane("Value", id="value"):
                        yield ParameterValueWidget(self.ros_node, id="parameter-value-view-content")

        yield Footer()


    def action_help(self):
        """Show the help modal."""
        self.push_screen(HelpModal())

    def action_toggle_dark(self) -> None:
        """An action to toggle dark mode."""
        self.dark = not self.dark

    def action_restart_node(self) -> None:
        """Forward restart_node action to the NodeListWidget."""
        print("LazyRosApp.action_restart_node: Forwarding action to NodeListWidget")
        node_list = self.query_one(NodeListWidget)
        if node_list:
            node_list.action_restart_node()
    
    def set_left_pane_visual(self) -> None:
        """Set the visual style for the left pane."""

        if self.focused_pane == "right":
            self.query_one(self.left_container_widgets[self.current_pane_index]).styles.border = ("round", "white")

        else:
            for i, name in enumerate(self.left_container_widgets):
                widget = self.query_one(name)
                if i == self.current_pane_index:
                    widget.styles.border = ("round", "green")
                else:
                    widget.styles.border = ("round", "white")


    def action_focus_left_pane(self) -> None:
        """Focus the left pane and highlight it."""
        self.focused_pane = "left"

        left_pane: Container = self.query_one("#left-frame")
        right_pane: Container = self.query_one("#right-frame")
        right_pane.styles.border = ("round", "white")

        self.set_left_pane_visual()
        left_pane.focus()

    def action_focus_right_pane(self) -> None:
        """Focus the right pane and highlight it."""

        self.focused_pane = "right"
        right_pane: Container = self.query_one("#right-frame")
        right_pane.styles.border = ("round", "green")
        self.set_left_pane_visual()
        
        try:
            if self.current_right_pane_config == "topics":
                topic_tabs = self.query_one("#topic-tabs")
                topic_tabs.focus()
            elif self.current_right_pane_config == "parameters":
                parameter_tabs = self.query_one("#parameter-tabs")
                parameter_tabs.focus()
            else:
                default_tabs = self.query_one("#default-tabs")
                default_tabs.focus()
        except Exception:
            right_pane.focus()

    def action_handle_topic_click(self, topic_name: str) -> None:
        """Handle clicks on topic 'links' in the InfoViewWidget."""
        print(f"Topic clicked: {topic_name}")
        self.push_screen(TopicInfoModal(topic_name))

    def action_handle_message_click(self, message_type: str) -> None:
        """Handle clicks on message type 'links' in the InfoViewWidget."""
        print(f"Message type clicked: {message_type}")
        self.push_screen(MessageModal(message_type))

    def update_topic_display(self, topic_name: str) -> None:
        """Update the topic-specific Info and Echo tabs with the selected topic."""
        try:
            print(f"[UPDATE] Called with topic: {topic_name}")
            
            # Only update if we're currently in topic mode
            if self.current_right_pane_config == "topics":
                # Store the current topic immediately (no delay)
                old_topic = getattr(self, 'current_selected_topic', None)
                self.current_selected_topic = topic_name
                print(f"[UPDATE] Topic changed from '{old_topic}' to '{topic_name}'")
                
                # Cancel previous timer if exists
                if hasattr(self, '_topic_update_timer') and self._topic_update_timer is not None:
                    print("[UPDATE] Cancelling previous timer")
                    self._topic_update_timer.cancel()
                
                # Set new timer for 1 second delay
                print(f"[UPDATE] Setting timer for topic: {topic_name}")
                self._topic_update_timer = self.set_timer(1.0, self._delayed_topic_update)
                
        except Exception as e:
            print(f"[MAIN APP] Error updating topic display: {e}")
    
    def _delayed_topic_update(self) -> None:
        """Delayed update for both Info and Echo tabs after 1 second."""
        try:
            print(f"[DELAYED] Timer fired! Current topic: {getattr(self, 'current_selected_topic', 'None')}")
            
            if self.current_right_pane_config == "topics" and hasattr(self, 'current_selected_topic') and self.current_selected_topic:
                topic_name = self.current_selected_topic
                print(f"[DELAYED] Actually updating for topic: {topic_name}")
                
                # Update Info tab
                try:
                    info_widget = self.query_one("#topic-info-view-content")
                    print(f"[DELAYED] Updating Info tab for: {topic_name}")
                    info_widget.update_topic_info(topic_name)
                    print(f"[DELAYED] Info tab updated")
                except Exception as e:
                    print(f"[MAIN APP] Error updating Info tab: {e}")
                
                # Update Echo tab
                try:
                    echo_widget = self.query_one("#echo-view-content")
                    print(f"[DELAYED] Updating Echo tab for: {topic_name}")
                    echo_widget.start_echo(topic_name)
                    print(f"[DELAYED] Echo tab updated")
                except Exception as e:
                    print(f"[MAIN APP] Error updating Echo tab: {e}")
            else:
                print(f"[DELAYED] Skipping update - config: {self.current_right_pane_config}, topic: {getattr(self, 'current_selected_topic', 'None')}")
                
        except Exception as e:
            print(f"[MAIN APP] Error in delayed topic update: {e}")
    
    def action_focus_next_pane(self) -> None:
        """Focus the next pane. 
            If on left pane, move to next left pane. 
            If on right pane, move to left pane.
        """

        if self.focused_pane == "right":
            # If on right pane, move to left pane
            self.action_focus_left_pane()
            self._focus_current_pane()
        else:
            # If on left pane, move to next left pane
            self.current_pane_index = (self.current_pane_index + 1) % len(self.left_pane_widgets)
            self._focus_current_pane()

    def action_focus_previous_pane(self) -> None:
        """Focus the previous pane in the left panel (Parameters -> Topics -> Node -> Parameters)."""

        self.current_pane_index = (self.current_pane_index - 1) % len(self.left_pane_widgets)
        self._focus_current_pane()
    
    def action_previous_tab(self) -> None:
        """Switch to previous tab in the right pane."""
        try:
            if self.current_right_pane_config == "topics":
                topic_tabs = self.query_one("#topic-tabs")
                current_tab_= topic_tabs.active
                if self.TOPIC_TAB_ID_LIST[0] == current_tab_:
                    return
                for i in self.TOPIC_TAB_ID_LIST[1:]:
                    if current_tab_ == i:
                        topic_tabs.active = self.TOPIC_TAB_ID_LIST[self.TOPIC_TAB_ID_LIST.index(i) - 1]

            elif self.current_right_pane_config == "parameters":
                parameter_tabs = self.query_one("#parameter-tabs")
                current_tab_= parameter_tabs.active
                if self.PARAMETER_TAB_ID_LIST[0] == current_tab_:
                    return
                for i in self.PARAMETER_TAB_ID_LIST[1:]:
                    if current_tab_ == i:
                        parameter_tabs.active = self.PARAMETER_TAB_ID_LIST[self.PARAMETER_TAB_ID_LIST.index(i) - 1]

            elif self.current_right_pane_config == "default":
                default_tabs = self.query_one("#default-tabs")
                current_tab_= default_tabs.active
                if self.NODE_TAB_ID_LIST[0] == current_tab_:
                    return
                for i in self.NODE_TAB_ID_LIST[1:]:
                    if current_tab_ == i:
                        default_tabs.active = self.NODE_TAB_ID_LIST[self.NODE_TAB_ID_LIST.index(i) - 1]
        except Exception as e:
            print(f"Error switching to previous tab: {e}")
            import traceback
            traceback.print_exc()
    
    def action_next_tab(self) -> None:
        """Switch to next tab in the right pane."""
        try:
            if self.current_right_pane_config == "topics":
                topic_tabs = self.query_one("#topic-tabs")
                current_tab_= topic_tabs.active
                if self.TOPIC_TAB_ID_LIST[-1] == current_tab_:
                    return
                for i in self.TOPIC_TAB_ID_LIST[:-1]:
                    if current_tab_ == i:
                        topic_tabs.active = self.TOPIC_TAB_ID_LIST[self.TOPIC_TAB_ID_LIST.index(i) + 1]

            elif self.current_right_pane_config == "parameters":
                parameter_tabs = self.query_one("#parameter-tabs")
                current_tab_= parameter_tabs.active
                if self.PARAMETER_TAB_ID_LIST[-1] == current_tab_:
                    return
                for i in self.PARAMETER_TAB_ID_LIST[:-1]:
                    if current_tab_ == i:
                        parameter_tabs.active = self.PARAMETER_TAB_ID_LIST[self.PARAMETER_TAB_ID_LIST.index(i) + 1]

            elif self.current_right_pane_config == "default":
                default_tabs = self.query_one("#default-tabs")
                current_tab_= default_tabs.active
                if self.NODE_TAB_ID_LIST[-1] == current_tab_:
                    return
                
                for i in self.NODE_TAB_ID_LIST[:-1]:
                    if current_tab_ == i:
                        default_tabs.active = self.NODE_TAB_ID_LIST[self.NODE_TAB_ID_LIST.index(i) + 1]
        except Exception as e:
            print(f"Error switching to next tab: {e}")
            import traceback
            traceback.print_exc()

    def _focus_current_pane(self) -> None:
        """Focus the current pane based on current_pane_index."""

        try:
            widget_id = self.left_pane_widgets[self.current_pane_index]
            widget = self.query_one(widget_id)
            
            # Set focus to left pane and update visuals
            self.focused_pane = "left"
            right_pane_widget = self.query_one("#right-frame")
            right_pane_widget.styles.border = ("round", "white")
            self.set_left_pane_visual()
            
            if widget_id == "#node-list-content":
                widget.node_list_view.focus()
                self._update_right_pane_for_nodes()
            elif widget_id == "#topic-list-content":
                widget.topic_list_view.focus()
                self._update_right_pane_for_topics()
            elif widget_id == "#parameter-list-content":
                widget.parameter_list_view.focus()
                self._update_right_pane_for_parameters()
        except Exception as e:
            print(f"Error focusing pane: {e}")
    
    def _update_right_pane_for_topics(self) -> None:
        """Update right pane to show Info and Echo tabs for Topics."""
        print("[MAIN APP] _update_right_pane_for_topics called!")
        try:
            # Update title
            title_widget = self.query_one("#right-pane-title")
            title_widget.update("Topic Info and Echo")
            
            # Hide default tabs and show topic tabs
            default_tabs = self.query_one("#default-tabs")
            topic_tabs = self.query_one("#topic-tabs")
            
            default_tabs.add_class("hidden")
            topic_tabs.remove_class("hidden")
            
            self.current_right_pane_config = "topics"
            print(f"[MAIN APP] current_right_pane_config set to: {self.current_right_pane_config}")
            
            # Check if there's a currently selected topic and display it
            try:
                topic_widget = self.query_one("#topic-list-content")
                if topic_widget.topic_list_view.index is not None:
                    index = topic_widget.topic_list_view.index
                    if 0 <= index < len(topic_widget.topic_list_view.children):
                        selected_item = topic_widget.topic_list_view.children[index]
                        if selected_item.children:
                            child = selected_item.children[0]
                            topic_name = str(child.renderable).strip()
                            if topic_name.startswith("/"):
                                print(f"[MAIN APP] Initial topic selection: {topic_name}")
                                self.update_topic_display(topic_name)
            except Exception as e:
                print(f"Error getting current topic selection: {e}")
            
        except Exception as e:
            print(f"Error updating right pane for topics: {e}")
    
    def _update_right_pane_for_nodes(self) -> None:
        """Update right pane to show Log and Info tabs for Nodes."""
        try:
            # Stop echo if it's running
            self._stop_topic_echo()
            
            # Update title
            title_widget = self.query_one("#right-pane-title")
            title_widget.update("Logs and Info")
            
            # Show default tabs and hide topic tabs
            default_tabs = self.query_one("#default-tabs")
            topic_tabs = self.query_one("#topic-tabs")
            
            default_tabs.remove_class("hidden")
            topic_tabs.add_class("hidden")
            
            self.current_right_pane_config = "default"
        except Exception as e:
            print(f"Error updating right pane for nodes: {e}")
    
    def _update_right_pane_for_parameters(self) -> None:
        """Update right pane to show Info and Value tabs for Parameters."""
        try:
            # Update title
            title_widget = self.query_one("#right-pane-title")
            title_widget.update("Parameter Info and Value")
            
            # Hide default tabs and topic tabs, show parameter tabs
            default_tabs = self.query_one("#default-tabs")
            topic_tabs = self.query_one("#topic-tabs")
            parameter_tabs = self.query_one("#parameter-tabs")
            
            default_tabs.add_class("hidden")
            topic_tabs.add_class("hidden")
            parameter_tabs.remove_class("hidden")
            
            self.current_right_pane_config = "parameters"
            
        except Exception as e:
            print(f"Error updating right pane for parameters: {e}")
    
    def _stop_topic_echo(self) -> None:
        """Stop the topic echo if it's currently running."""
        try:
            echo_widget = self.query_one("#echo-view-content")
            echo_widget.stop_echo()
        except Exception as e:
            print(f"Error stopping topic echo: {e}")
    
    def set_echo_rate(self, rate: float) -> None:
        """Set the echo update rate in Hz."""
        try:
            echo_widget = self.query_one("#echo-view-content")
            echo_widget.set_update_rate(rate)
        except Exception as e:
            print(f"Error setting echo rate: {e}")
    
    def get_echo_rate(self) -> float:
        """Get the current echo update rate in Hz."""
        try:
            echo_widget = self.query_one("#echo-view-content")
            return echo_widget.get_update_rate()
        except Exception as e:
            print(f"Error getting echo rate: {e}")
            return 2.0  # Default rate


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
