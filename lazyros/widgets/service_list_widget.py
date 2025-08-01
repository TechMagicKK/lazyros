from textual.app import ComposeResult
from textual.widgets import ListView, ListItem, Label
from textual.containers import Container
from rich.markup import escape

from rclpy.node import Node
from lazyros.utils.ignore_parser import IgnoreParser


def escape_markup(text: str) -> str:
    """
    Escape text for rich markup.
    """
    return escape(text)

class ServiceListWidget(Container):
    """
    A widget to display the list of ROS services.
    """

    def __init__(self, ros_node: Node, **kwargs):
        super().__init__(**kwargs)
        self.ros_node = ros_node
        self.previous_service_data = {}
        self.ignore_parser = IgnoreParser("config/ignore_services.yaml")

        BINDINGS = [
            ("q", "quit", "Quit the application"),
            ("r", "refresh", "Refresh the service list"),
        ]

    def compose(self) -> ComposeResult:
        """
        Create the vertical scrollable service list 
        """
        yield Label('ROS Services')
        yield ListView(id='service_list')

    def on_mount(self) -> None:
        """
        Initialize the service list with some dummy data.
        """
        self._fetch_ros_services()
        self.set_interval(3.0, self._fetch_ros_services)  # Refresh every 3 seconds
    
    def _fetch_ros_services(self):
        """
        call ros2 service list command to fetch the running service names.
        """
        try:
            # Fetch the service names and types from the ROS node
            service_name_and_types_list = self.ros_node.get_service_names_and_types()
            new_service_data: dict[str, str] = {}
            for name, types_list in service_name_and_types_list:
                new_service_data[name] = types_list[0] if types_list else ""
            
            # Data has changed, update the service data
            if new_service_data != self.previous_service_data:
                self.previous_service_data = new_service_data
                self._refresh_display_list()
        
        except Exception as e:
            service_list_view = self.query_one(ListView)
            service_list_view.clear()
            service_list_view.append(ListItem(Label(f'Error fetching services: {escape_markup(str(e))}')))
    
    def _refresh_display_list(self):
        """
        Clears and repopulates the ListView with the latest service data.
        """

        try:
            # Use self.previous_service_data to populate the ListView
            all_service_names = list(self.previous_service_data.keys())

            # filter service based on the ignore list
            filtered_service_names = [
                name for name in all_service_names
                if not self.ignore_parser.should_ignore(name, 'service')
            ]

            items = []
            display_names = sorted(filtered_service_names)
            for name_str in display_names:
                # Ensure name_str is actually a string
                if not isinstance(name_str, str):
                    items.append(ListItem(Label(f'[Error] Invalid service name: {name_str}')))
                    continue
                else:
                    items.append(ListItem(Label(escape_markup(name_str))))
            service_list_view = self.query_one(ListView)
            service_list_view.clear()
            service_list_view.extend(items)

        except Exception as e:
            service_list_view = self.query_one(ListView)
            service_list_view.clear()
            service_list_view.append(ListItem(Label(f'Error updating service list: {escape_markup(str(e))}')))