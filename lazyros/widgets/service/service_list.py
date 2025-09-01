import asyncio
from textual.app import ComposeResult
from textual.widgets import ListView, ListItem, Label
from textual.containers import Container
from textual.events import Focus
from rich.markup import escape
from rich.text import Text as RichText

from rclpy.node import Node
from lazyros.utils.ignore_parser import IgnoreParser

import os


def escape_markup(text: str) -> str:
    """
    Escape text for rich markup.
    """
    return escape(text)


class MyListView(ListView):
    """Cusonm ListView that automatically focuses on mount"""
    def on_focus(self, event: Focus) -> None:
        if self.children and not self.index:
            self.index = 0
            self.refresh(layout=True)

class ServiceListWidget(Container):
    """
    A widget to display the list of ROS services.
    """

    def __init__(self, ros_node: Node, **kwargs):
        super().__init__(**kwargs)
        self.ros_node = ros_node
        self.listview = MyListView()

        ignore_file_path = os.path.join(os.path.dirname(__file__), '../../../config/display_ignore.yaml')
        self.ignore_parser = IgnoreParser(ignore_file_path)

        self.service_dict = {}
        self.selected_service = None

    def compose(self) -> ComposeResult:
        """
        Create the vertical scrollable service list 
        """
        yield self.listview

    def on_mount(self) -> None:
        """
        Initialize the service list with some dummy data.
        """
        asyncio.create_task(self.update_service_list())
        self.set_interval(1, lambda: asyncio.create_task(self.update_service_list()))
        if self.listview.children:
            self.listview.index = 0
    
    async def update_service_list(self) -> None:
        """Fetch and update the list of services."""
        services = self.ros_node.get_service_names_and_types()
        need_update = False

        for service in services:
            print(f'service : {service}')
            if self.ignore_parser.should_ignore(service[0], 'service'):
                continue
            if service[0] not in self.service_dict:
                need_update = True
                self.service_dict[service[0]] = service[1]

        if not need_update:
            return
        
        self.listview.clear()
        service_list = []
        for service in list(self.service_dict.keys()):
            label = RichText.assemble(RichText(service))
            service_list.append(ListItem(Label(label)))

        self.listview.extend(service_list)
    
    def on_list_view_highlighted(self, event):
        index = self.listview.index
        if index is None or not (0 <= index < len(self.listview.children)):
            self.selected_service = None
            return
        item = self.listview.children[index]
        if not item.children:
            self.selected_service = None
            return

        service_name = str(item.children[0].renderable).strip()
        if service_name:
            service_type = self.service_dict.get(service_name)[0]
            self.selected_service = (service_name, service_type)