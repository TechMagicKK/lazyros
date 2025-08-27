import asyncio
from rclpy.node import Node
from textual.app import ComposeResult
from textual.binding import Binding
from textual.containers import Container, VerticalScroll, ScrollableContainer
from textual.widgets import (
    Label,
    ListItem,
    ListView,
    Input,
)
from textual.events import Key
from rich.markup import escape

from lazyros.utils.ignore_parser import IgnoreParser
import os

from rich.text import Text as RichText
from textual.events import Focus
from textual import on


def escape_markup(text: str) -> str:
    """Escape text for rich markup."""
    return escape(text)

class MyListView(ListView):
    """Custom ListView that automatically focuses on mount."""

    def on_focus(self, event: Focus) -> None:
        if self.children and not self.index:
            self.index = 0
            self.refresh(layout=True)

class TopicListWidget(Container):
    """A widget to display the list of ROS topics."""

    DEFAULT_CSS = """
    TopicListWidget {
        overflow: hidden;
    }

    #scroll-area {
        overflow-x: auto;
        overflow-y: auto;
        height: 1fr;
    }
    """

    def __init__(self, ros_node: Node, **kwargs):
        super().__init__(**kwargs)
        self.ros_node = ros_node
        self.listview = MyListView()

        ignore_file_path = os.path.join(os.path.dirname(__file__), '../../../config/display_ignore.yaml')
        self.ignore_parser = IgnoreParser(os.path.abspath(ignore_file_path))
        
        self.topic_dict = {}
        self.selected_topic = None

    def compose(self) -> ComposeResult:
        yield self.listview

    def on_mount(self) -> None:
        asyncio.create_task(self.update_topic_list())
        self.set_interval(1, lambda: asyncio.create_task(self.update_topic_list()))
        if self.listview.children:
            self.listview.index = 0

    async def update_topic_list(self) -> None:
        """Fetch and update the list of topics."""
    
        topics = self.ros_node.get_topic_names_and_types()
        need_update = False

        for topic in topics:
            if self.ignore_parser.should_ignore(topic[0], 'topic'):
                continue
            if topic[0] not in self.topic_dict:
                need_update = True
                self.topic_dict[topic[0]] = topic[1]

        if not need_update:
            return

        self.listview.clear()
        topic_list = []
        for topic in list(self.topic_dict.keys()):
            label = RichText.assemble(RichText(topic))
            topic_list.append(ListItem(Label(label)))
        
        self.listview.extend(topic_list)

    def on_list_view_highlighted(self, event):

        index = self.listview.index
        if index is None or not (0 <= index < len(self.listview.children)):
            self.selected_topic = None
            return
        item = self.listview.children[index]
        if not item.children:
            self.selected_topic = None
            return

        topic_name = str(item.children[0].renderable).strip()
        if self.selected_topic != topic_name:
            self.selected_topic = topic_name 
