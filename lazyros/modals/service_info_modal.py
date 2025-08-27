import subprocess
from textual.app import ComposeResult
from textual.binding import Binding
from textual.containers import Container
from textual.widgets import Label, Log
from textual.screen import ModalScreen
from rich.markup import escape as escape_markup

from rclpy.node import Node

class ServiceInfoModal(ModalScreen[None]):
    """A modal screen to display service information."""

    CSS = """
    ServiceInfoModal {
        align: center middle;
        layer: modal;
    }

    #modal-container {
        width: 40%;
        height: auto;
        border: round white;
        background: $background;
    }
    
    #modal-title {
        dock: top;
        width: 100%;
        text-align: center;
        padding: 1;
        background: $primary-background-darken-1;
    }

    #modal-content {
        width: 100%;
        height: auto;
        border: round $primary;
        margin: 0 1;
    }

    #modal-instruction {
        dock: bottom;
        width: 100%;
        text-align: center;
        padding: 1;
    }
    """

    BINDINGS = [
        Binding("escape", "dismiss", "Quit Modal")
    ]

    def __init__(self, service_name: str, ros_node: Node, **kwargs):
        super().__init__(**kwargs)
        self.service_name = service_name
        self.service_info = self.get_service_info()
        self.ros_node = ros_node

    def get_service_info(self) -> str:
        """Fetch the service information using `ros2 service info` command."""
        try:
            result = self.ros_node.get_service_names_and_types()
            return escape_markup(result)
        except subprocess.CalledProcessError as e:
            return f"Error fetching service info: {e}"