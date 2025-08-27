import subprocess
import asyncio
from rclpy.node import Node
from textual.app import ComposeResult
from textual.binding import Binding
from textual.containers import Container
from textual.widgets import Label, ListItem, ListView
from rich.markup import escape
from rich.text import Text as RichText
from dataclasses import dataclass

from lazyros.utils.ignore_parser import IgnoreParser
from lazyros.modals.lifecycle_modal import LifecycleModal

import os
import signal

import re

def escape_markup(text: str) -> str:
    return escape(text)


@dataclass
class NodeData:
    full_name: str
    status: str
    namespace: str = ""
    node_name: str = ""

class NodeListWidget(Container):
    def __init__(self, ros_node: Node, ignore_file_path='config/display_ignore.yaml', **kwargs) -> None:
        super().__init__(**kwargs)
        self.ros_node = ros_node
        self.listview = ListView()
        self.node_listview_dict = {}
        
        self.selected_node_name = None
        ignore_file_path = os.path.join(os.path.dirname(__file__), '../../../config/display_ignore.yaml')
        self.ignore_parser = IgnoreParser(os.path.abspath(ignore_file_path))

    def compose(self) -> ComposeResult:
        yield self.listview

    def on_mount(self) -> None:
        asyncio.create_task(self.update_node_list())
        self.set_interval(1, lambda: asyncio.create_task(self.update_node_list()))

        if self.listview.children:
            self.listview.index = 0

    async def update_node_list(self) -> None:
        """Update the list of nodes."""

        nodes_and_namespaces = self.ros_node.get_node_names_and_namespaces()
        launched_node_set =list(self.node_listview_dict.keys())
        need_update = False
        
        for tuple in nodes_and_namespaces:
            node = tuple[0]
            namespace = tuple[1]
            if namespace == "/":
                node_name = namespace + node 
            else:
                node_name = namespace + "/" + node

            if self.ignore_parser.should_ignore(node_name, 'node'):
                continue
           
            if node_name not in launched_node_set:
                need_update = True
                # If the node is not in the launched nodes, add it
                self.node_listview_dict[node_name] = NodeData(full_name=node_name, status="green", namespace=namespace, node_name=node)
            else:
                if self.node_listview_dict[node_name].status != "green":
                    need_update = True
                    self.node_listview_dict[node_name].status = "green"
                launched_node_set.remove(node_name)
           
        # Set nodes that are no longer launched to red
        for dead_node in launched_node_set:
            if self.node_listview_dict[dead_node].status == "green":
                self.node_listview_dict[dead_node].status = "red"
                need_update = True

        if not need_update:
            return
                
        nodes_list = list(self.node_listview_dict.keys())
        sorted_nodes = sorted(nodes_list)
          
        # update node listview 
        self.listview.clear()
        nodes = []
        for node in sorted_nodes:
             status = self.node_listview_dict[node].status
             label = RichText.assemble(
                RichText("●", style=f"bold {status}"),
                "    ",
                RichText(node)
             )
             nodes.append(ListItem(Label(label)))
             
        self.listview.extend(nodes)

        current_index = self.listview.index
        if current_index is not None and current_index < len(nodes):
            self.listview.index = current_index
        elif nodes:
            self.listview.index = 0

    def on_list_view_highlighted(self, event):
        index = self.listview.index
        if index is None or not (0 <= index < len(self.listview.children)):
            self.selected_node_name = None
            return
        item = self.listview.children[index]
        if not item.children:
            self.selected_node_name = None
            return

        # Extract the display name
        name_str = str(item.children[0].renderable).strip()
        # Extract node name after the first slash (if any)
        node_name = name_str.split("/", 1)[-1] if "/" in name_str else name_str

        if self.selected_node_name != node_name:
            self.selected_node_name = node_name

        #self.update_window_display()

    def update_window_display(self):
        if self.selected_node_name is None:
            return

        log_widget = self.app.query_one("#log-view-content")
        filtered_node = re.sub(r'^/', '', self.selected_node_name).replace('/', '.')
        log_widget.selected_node = filtered_node 
        
        info_widget = self.app.query_one("#info-view-content")
        info_widget.selected_node_data = self.node_listview_dict["/"+self.selected_node_name]

        lifecycle_widget = self.app.query_one("#lifecycle-view-content")
        lifecycle_widget.selected_node_data = self.node_listview_dict["/"+self.selected_node_name]
