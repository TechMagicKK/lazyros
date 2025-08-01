import subprocess
import re 
import asyncio
from typing import List, Optional, Tuple
from concurrent.futures import ThreadPoolExecutor

from rclpy.node import Node
from textual.app import ComposeResult
from textual.binding import Binding
from textual.containers import Container, ScrollableContainer
from textual.css.query import DOMQuery
from textual.widgets import (
    Label,
    ListItem,
    ListView,
)
from rich.markup import escape

from lazyros.modals.parameter_value_modal import ParameterValueModal
from lazyros.modals.set_parameter_modal import SetParameterModal
from lazyros.utils.ignore_parser import IgnoreParser


def escape_markup(text: str) -> str:
    """Escape text for rich markup."""
    return escape(text)

class ParameterListWidget(Container):
    """A widget to display the list of ROS parameters using 'ros2 param list'."""

    BINDINGS = [
        Binding("s", "set_selected_parameter", "Set Value", show=True),  # Add set binding
    ]

    def __init__(self, ros_node: Node, **kwargs) -> None:
        super().__init__(**kwargs)
        self.ros_node = ros_node
        self.listview = ListView()
        self.previous_parameters_display_list: List[str] = []
        self._executor = ThreadPoolExecutor(max_workers=2, thread_name_prefix="param_list")
        self._current_update_task: Optional[asyncio.Task] = None
        self.ignore_parser = IgnoreParser('config/display_ignore.yaml')
        self._selected_param = None
        self._last_processed_parameter = None
        
        self._highlight_task = None

    def _log_error(self, msg: str):
        if hasattr(self.ros_node, 'get_logger'):
            self.ros_node.get_logger().error(f"[ParameterListWidget] {msg}")

    def compose(self) -> ComposeResult:
        #yield Label("ROS Parameters:")
        yield ScrollableContainer(self.listview)

    def on_mount(self) -> None:
        self.listview.initial_index = 0
        self.set_interval(5, self.trigger_update_list)
        self.trigger_update_list()

    def _parse_ros2_param_list_output(self, output: str) -> List[str]:
        parsed_params: List[str] = []
        current_node_name = None
        lines = output.splitlines()

        for line_raw in lines:
            line = line_raw.strip()
            if not line:
                continue

            if line.endswith(':'):
                potential_node_name = line[:-1].strip()
                if potential_node_name.startswith('/'):
                    current_node_name = potential_node_name
                else:
                    current_node_name = None

            elif current_node_name and line:
                if line_raw.startswith("  ") and not line.startswith(" "):
                    param_name = line.strip()
                    if param_name:
                        # Store raw text for parsing later, display escaped text
                        parsed_params.append(escape_markup(f"{current_node_name}: {param_name}"))

        parsed_params.sort()
        return parsed_params

    def _fetch_and_parse_parameters(self) -> List[str]:
        try:
            cmd = "ros2 param list"
            process_result = subprocess.run(
                cmd, shell=True, capture_output=True, text=True, timeout=10
            )

            if process_result.returncode == 0:
                if process_result.stdout:
                    output_str = process_result.stdout.strip()
                    if output_str:
                        parsed_list = self._parse_ros2_param_list_output(output_str)

                        # Filter parameters based on the ignore list
                        filtered_params = [
                            param_str for param_str in parsed_list
                            if not self.ignore_parser.should_ignore(param_str, 'parameter')
                        ]

                        if not filtered_params:
                            return ["[No parameters found after filtering]"]
                        return filtered_params
                    else:
                        return ["[No parameters found: 'ros2 param list' returned empty output]"]
                else:
                    return ["['ros2 param list' succeeded but gave no stdout]"]
            else:
                err_msg = process_result.stderr.strip() if process_result.stderr else "Unknown error"
                # self._log_error(f"Thread: 'ros2 param list' failed. RC: {process_result.returncode}. Error: {err_msg}")
                return [escape_markup(f"[Error (RC {process_result.returncode}) running 'ros2 param list'. See logs]")]

        except subprocess.TimeoutExpired:
            # self._log_error("Thread: 'ros2 param list' command timed out.")
            return ["[Error: 'ros2 param list' command timed out. Check ROS environment]"]

        except Exception as e:
            # self._log_error(f"Thread: Error during parameter list fetch: {type(e).__name__} - {str(e)}")
            return [escape_markup(f"[General Error in list fetch thread: {type(e).__name__}. See logs]")]

    def _update_view(self, new_params_list: List[str]):
        """Update the parameter list view with new data."""

        if self.previous_parameters_display_list != new_params_list:
            self.listview.clear()
            items = [ListItem(Label(param_str)) for param_str in new_params_list] if new_params_list else [ListItem(Label("[No parameters available or error during fetch]"))]
            self.listview.extend(items)
            if items and (self.listview.index is None or self.listview.index >= len(items)):
                self.listview.index = 0
            elif not items:
                self.listview.index = None
            self.previous_parameters_display_list = new_params_list

    async def _update_list_async(self) -> None:
        """Asynchronously update the parameter list."""

        try:
            result = await asyncio.get_event_loop().run_in_executor(
                self._executor, self._fetch_and_parse_parameters
            )
            self._update_view(result)
        except asyncio.CancelledError:
            pass
        except Exception as e:
            self._log_error(f"Error during async parameter list update: {e}")
            self._update_view([f"[Error during update: {e}]"])

    def trigger_update_list(self) -> None:
        """Trigger an update of the parameter list, cancelling any existing update."""

        if self._current_update_task and not self._current_update_task.done():
            self._current_update_task.cancel()
        
        self._current_update_task = asyncio.create_task(self._update_list_async())

    def _parse_selected_item(self, item_text: str) -> Optional[Tuple[str, str]]:
        """Regex to capture node_name and param_name from "node_name: param_name."""

        match = re.fullmatch(r"([^:]+):\s*(.+)", item_text)
        if match:
            node_name = match.group(1).strip()
            param_name = match.group(2).strip()
            return node_name, param_name
        # self._log_error(f"Could not parse selected item: '{item_text}'")
        return None


    def on_list_view_highlighted(self, event):
        """Handle when a parameter is highlighted/selected in the ListView."""

        try:
            index = self.listview.index
            
            if index is None or index < 0 or index >= len(self.listview.children):
                self._selected_param = None
                return
            
            selected_item = self.listview.children[index]
            if not selected_item.children:
                self._selected_param = None
                return
            
            child = selected_item.children[0]
            parameter_text = str(child.renderable).strip()
            
            if self._last_processed_parameter == parameter_text:
                return
                
            if ":" in parameter_text and not parameter_text.startswith("["):
                self._selected_param = parameter_text
                self._last_processed_parameter = parameter_text
                
                if self._highlight_task and not self._highlight_task.done():
                    self._highlight_task.cancel()
                self._highlight_task = asyncio.create_task(self._update_window_display())

            else:
                self._selected_param = None
                self._last_processed_parameter = None
                
        except Exception:
            self._selected_param = None
            self._last_processed_parameter = None


    async def _update_window_display(self):
        """Update main window display"""

        if not self._selected_param or not ":" in self._selected_param:
            self.log.error("No valid parameter selected for display update.")
            return

        try:
            if self.app.current_right_pane_config == "parameters":
                try:
                    info_widget = self.app.query_one("#parameter-info-view-content")
                    info_widget.current_parameter = self._selected_param
                
                    value_widget = self.app.query_one("#parameter-value-view-content")
                    value_widget.current_parameter = self._selected_param

                except Exception:
                    self.log.error("Error updating parameter display widgets.")

        except Exception:
            self.log.error("Error updating parameter display in main window.")


    # --- Action to Set Parameter Value ---
    def action_set_selected_parameter(self) -> None:
        """Action to set the value of the currently selected parameter."""

        highlighted_item_widget: Optional[ListItem] = self.listview.highlighted_child
        if not highlighted_item_widget:
            self.app.bell()
            return

        children_query: DOMQuery[Label] = highlighted_item_widget.query(Label)  # type: ignore
        if not children_query:
            self.app.bell()
            return

        selected_label: Label = children_query.first()
        item_text_renderable = selected_label.renderable
        item_text_plain = str(item_text_renderable)  # Convert RichText or str to plain str

        parsed_names = self._parse_selected_item(item_text_plain)
        if not parsed_names:
            self.app.bell()
            self.app.push_screen(ParameterValueModal(title="Error", content="Could not parse selected parameter string."))
            return

        node_name, param_name = parsed_names

        try:
            cmd = f"ros2 param describe \"{node_name}\" \"{param_name}\""
            process_result = subprocess.run(
                cmd, shell=True, capture_output=True, text=True, timeout=5
            )

            if process_result.returncode == 0:
                # Parse the output to get the type
                # Example output:
                # Parameter: use_sim_time
                #   Type: bool
                #   Description: Use simulation time
                description_output = process_result.stdout.strip()
                type_match = re.search(r"Type: (\w+)", description_output)
                param_type = type_match.group(1) if type_match else "Unknown"

                self.app.push_screen(SetParameterModal(node_name, param_name, param_type))

            else:
                err_msg = process_result.stderr.strip() if process_result.stderr else "Unknown error"
                # self._log_error(f"Error describing param {node_name} {param_name}: RC {process_result.returncode}, Err: {err_msg}")
                self.app.push_screen(ParameterValueModal(title="Error", content=f"Could not describe parameter:\n{err_msg}"))

        except subprocess.TimeoutExpired:
            self.app.push_screen(ParameterValueModal(title="Error", content="Timeout describing parameter."))
        except Exception as e:
            self.app.push_screen(ParameterValueModal(title="Error", content=f"An error occurred describing parameter: {e}"))