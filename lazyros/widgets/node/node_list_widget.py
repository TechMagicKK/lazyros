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


def escape_markup(text: str) -> str:
    return escape(text)


@dataclass
class NodeData:
    name: str
    status: str
    is_lifecycle: bool


class NodeListWidget(Container):
    BINDINGS = [
        Binding("k", "kill_node", "Kill Node"),
    ]

    def __init__(self, ros_node: Node, ignore_file_path='config/display_ignore.yaml', **kwargs) -> None:
        super().__init__(**kwargs)
        self.ros_node = ros_node
        self.listview = ListView()
        self.previous_node_names = set()
        self.selected_node_name = None
        self.ignore_parser = IgnoreParser(ignore_file_path)
        self.launched_nodes = {}

        self._highlight_task = None
        self._highlight_lock = asyncio.Lock()
        self._last_highlight_time = 0.0
        self._highlight_delay = 0.3
        self._last_log_filter = None
        self._current_node = None

    def compose(self) -> ComposeResult:
        #yield Label("ROS Nodes:")
        yield self.listview

    def on_mount(self) -> None:
        asyncio.create_task(self.update_node_list())
        self.set_interval(5, lambda: asyncio.create_task(self.update_node_list()))
        self.set_interval(1, self._update_if_ready)
        self.listview.focus()
        # Ensure first item is highlighted on startup
        if self.listview.children:
            self.listview.index = 0

    async def update_node_list(self) -> None:
        node_set = set(self.launched_nodes.keys())
        nodes = []
        need_update = False

        try:
            result = await asyncio.get_event_loop().run_in_executor(
                None, lambda: subprocess.run("ros2 node list", shell=True, capture_output=True, text=True)
            )
            if result.returncode != 0:
                self.listview.clear()
                self.listview.append(ListItem(Label("[red]Error fetching nodes[/]")))
                return

            for line in result.stdout.splitlines():
                node_name = line.strip()
                if self.ignore_parser.should_ignore(node_name, 'node'):
                    continue

                raw_name = node_name[1:] if node_name.startswith("/") else node_name
                if raw_name not in self.launched_nodes:
                    # Defer lifecycle check - just add as non-lifecycle for now
                    self.launched_nodes[raw_name] = NodeData(name=raw_name, status="green", is_lifecycle=False)
                    # Schedule async lifecycle check
                    asyncio.create_task(self._check_lifecycle_async(node_name, raw_name))
                    need_update = True
                elif self.launched_nodes[raw_name].status != "green":
                    self.launched_nodes[raw_name].status = "green"
                    need_update = True
                node_set.discard(raw_name)

            for dead_node in node_set:
                if self.launched_nodes[dead_node].status != "red":
                    self.launched_nodes[dead_node].status = "red"
                    need_update = True

            if not need_update:
                return

            sorted_names = sorted(self.launched_nodes.keys())
            current_index = self.listview.index
            self.listview.clear()
            for name in sorted_names:
                status = self.launched_nodes[name].status
                is_lifecycle = self.launched_nodes[name].is_lifecycle
                if is_lifecycle:
                    status_symbol = " Ⓛ　"
                else:
                    status_symbol = "    "

                label = RichText.assemble(RichText("●", style=f"bold {status}"), RichText(status_symbol, style=f"bold yellow"), "  ", RichText("/"+name))
                nodes.append(ListItem(Label(label)))
            self.listview.extend(nodes)

            if current_index is not None and current_index < len(nodes):
                self.listview.index = current_index
            elif nodes:
                self.listview.index = 0

        except Exception as e:
            self.listview.clear()
            self.listview.append(ListItem(Label(f"[red]Error fetching nodes: {e}[/]")))

    def on_list_view_highlighted(self, event):
        try:
            index = self.listview.index
            if index is None or index < 0 or index >= len(self.listview.children):
                self.selected_node_name = None
                return

            selected_item = self.listview.children[index]
            if not selected_item.children:
                self.selected_node_name = None
                return

            child = selected_item.children[0]
            raw_name = str(child.renderable).strip()

            if self._current_node == raw_name:
                return

            i = raw_name.find("/") + 1
            self.selected_node_name = raw_name[i:] if i != -1 else raw_name
            self._current_node = raw_name

            if self._highlight_task and not self._highlight_task.done():
                self._highlight_task.cancel()
            self._highlight_task = asyncio.create_task(self._delayed_update())

        except Exception as e:
            print(f"[highlight error] {e}")
            self.selected_node_name = None

    def action_kill_node(self) -> None:
        try:
            result = subprocess.run(
                ['pgrep', '-f', self.selected_node_name],
                stdout=subprocess.PIPE,
                text=True,
                check=True
            )
            pids = result.stdout.splitlines()
            if result.returncode != 0:
                return
            for pid in pids:
                os.kill(int(pid), signal.SIGTERM)
        except subprocess.CalledProcessError as e:
            pass
        except Exception as e:
            pass

    def action_show_lifecycle_state(self) -> None:
        """Show a modal with information about the selected node."""

        node_data = self.launched_nodes[self.selected_node_name]
        self.app.push_screen(LifecycleModal(self.ros_node, node_data))

    async def _delayed_update(self):
        await asyncio.sleep(self._highlight_delay)
        await self._update_log_and_info_async()

    async def _update_log_and_info_async(self):
        if not self.selected_node_name or self.selected_node_name.startswith("["):
            return

        async with self._highlight_lock:
            try:
                log_filter = self.selected_node_name
                if self._last_log_filter == log_filter:
                    return
                self._last_log_filter = log_filter

                log_view = self.app.query_one("#log-view-content")  # type: ignore
                info_view = self.app.query_one("#info-view-content")  # type: ignore

                log_view.filter_logs(log_filter.replace("/", "."))
                info_view.update_info("/" + log_filter)

            except Exception as e:
                print(f"[log/info update error] {e}")

    async def _check_lifecycle_async(self, node_name: str, raw_name: str) -> None:
        """Check if a node is a lifecycle node asynchronously."""
        try:
            cmd = ["ros2", "node", "info", node_name]
            result = await asyncio.get_event_loop().run_in_executor(
                None, lambda: subprocess.run(cmd, capture_output=True, text=True, check=True)
            )
            if "lifecycle_msgs" in result.stdout:
                if raw_name in self.launched_nodes:
                    self.launched_nodes[raw_name].is_lifecycle = True
                    # Update the display
                    await self.update_node_list()
        except (subprocess.CalledProcessError, asyncio.CancelledError):
            pass

    def _update_if_ready(self):
        if self._highlight_task and self._highlight_task.done():
            self._highlight_task = None
