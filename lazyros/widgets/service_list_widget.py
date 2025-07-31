from textual.app import ComposeResult
from textual.widgets import ListView, ListItem, Label
from textual.containers import Container

class ServiceListWidget(Container):
    """
    A widget to display the list of ROS services.
    """

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

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
        service_list = self.query_one(ListView)
        # Example services, replace with actual ROS service names
        services = ['service_one', 'service_two', 'service_three']
        
        for service in services:
            item = ListItem(Label(service))
            service_list.append(item)