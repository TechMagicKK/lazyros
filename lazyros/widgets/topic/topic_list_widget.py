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

from lazyros.modals.topic_info_modal import TopicInfoModal  # Import TopicInfoModal
from lazyros.modals.topic_echo_modal import TopicEchoModal  # Import TopicEchoModal
from lazyros.utils.ignore_parser import IgnoreParser  # Import IgnoreParser


def escape_markup(text: str) -> str:
    """Escape text for rich markup."""
    return escape(text)


class TopicListWidget(Container):
    """A widget to display the list of ROS topics."""

    BINDINGS = [
        Binding("i", "show_topic_info", "Info"),
        Binding("e", "echo_topic", "Echo"),
        Binding("/", "start_search", "Search"),
        Binding("escape", "clear_search", "Clear Search", show=False),
    ]

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
        self.listview = ListView()
        self.search_input = Input(placeholder="Search topics...")
        self.search_input.display = False
        self.is_searching = False
        self.previous_topic_data: dict[str, str] = {}
        self.current_search_term: str = ""
        self.ignore_parser = IgnoreParser('config/display_ignore.yaml')
        
        # NodeListWidget pattern
        self.selected_topic_name = None
        self._current_topic = None
        self._highlight_task = None
        self._highlight_delay = 1.0  # Debounce delay: update only after 0.5 seconds of no movement

    def compose(self) -> ComposeResult:
        #yield Label("ROS Topics:")
        yield self.search_input
        yield self.listview

    def on_mount(self) -> None:
        self._fetch_ros_topics()  # Initial fetch
        self.set_interval(5, self._fetch_ros_topics)  # Fetch new data periodically (increased interval)
        # Ensure first item is highlighted on startup
        if self.listview.children:
            self.listview.index = 0

    def on_key(self, event: Key) -> None:
        if self.search_input.has_focus:
            if event.key == "up":
                if self.listview.index is not None and self.listview.index > 0:
                    self.listview.index -= 1
                elif self.listview.index is None and len(self.topic_list_view.children) > 0:
                    self.listview.index = len(self.topic_list_view.children) - 1  # Select last if None
                self.listview.scroll_visible()  # Scroll to the new index
                event.stop()
            elif event.key == "down":
                if self.listview.index is not None and self.listview.index < len(self.topic_list_view.children) - 1:
                    self.listview.index += 1
                elif self.listview.index is None and len(self.topic_list_view.children) > 0:
                    self.listview.index = 0  # Select first if None
                self.listview.scroll_visible()  # Scroll to the new index
                event.stop()
            elif event.key == "escape":  # Escape from search input
                self.action_clear_search()
                event.stop()
            # Let Enter be handled by on_input_submitted
        elif event.key == "escape" and self.is_searching:  # Fallback if focus isn't on input but search is active
            self.action_clear_search()
            event.stop()

    def action_start_search(self) -> None:
        """Activate search mode."""
        self.search_input.display = True
        self.search_input.value = ""
        self.current_search_term = ""
        self.search_input.focus()
        self.is_searching = True
        self._refresh_display_list()  # Refresh with empty search

    def action_clear_search(self) -> None:
        """Clear search and deactivate search mode."""
        if self.is_searching or self.search_input.display:
            self.search_input.value = ""
            self.current_search_term = ""
            self.search_input.display = False
            self.is_searching = False
            self.listview.focus()
            self._refresh_display_list()  # Refresh to show all items

    async def on_input_changed(self, event: Input.Changed) -> None:
        """Handle text changes in the search input."""
        if event.input == self.search_input:
            new_search_val = self.search_input.value.lower()
            if new_search_val != self.current_search_term:
                self.current_search_term = new_search_val
                self._refresh_display_list()

    async def on_input_submitted(self, event: Input.Submitted) -> None:
        """Handle submission of the search input."""
        if event.input == self.search_input:
            # self.is_searching = False # User might want to keep searching with current term
            self.listview.focus()

    def _fetch_ros_topics(self) -> None:
        """Fetches ROS topics and updates the list if data has changed."""
        try:
            topic_names_and_types_list = self.ros_node.get_topic_names_and_types()
            new_topic_data: dict[str, str] = {}
            for name, types_list in topic_names_and_types_list:
                new_topic_data[name] = types_list[0] if types_list else ""

            if new_topic_data != self.previous_topic_data:
                self.previous_topic_data = new_topic_data
                self._refresh_display_list()  # Data changed, so refresh the displayed list
        except Exception as e:
            # Handle error display - perhaps a dedicated error label or log
            # For now, if list is empty, show error there. This might get overwritten.
            if not self.listview.children:  # Check if list is already empty
                self.listview.clear()
                self.listview.append(ListItem(Label(f"Error fetching topics: {escape_markup(str(e))}")))

    def _refresh_display_list(self) -> None:
        """Clears and repopulates the ListView based on current data and search term."""
        try:
            search_term = self.current_search_term

            # Use self.previous_topic_data as the source of truth for all topics
            all_topic_names = list(self.previous_topic_data.keys())

            # Filter topics based on the ignore list
            filtered_topic_names = [
                name for name in all_topic_names
                if not self.ignore_parser.should_ignore(name, 'topic')
            ]

            all_topic_names_sorted = sorted(filtered_topic_names)

            display_names = all_topic_names_sorted
            if search_term:  # Filter if search_term is active
                display_names = [name for name in all_topic_names_sorted if search_term in name.lower()]

            current_index = self.listview.index
            self.listview.clear()
            items = []

            if not self.previous_topic_data and not search_term:
                items.append(ListItem(Label("[No topics found]")))
            elif not display_names and search_term:
                items.append(ListItem(Label(f"[No topics match '{escape_markup(search_term)}']")))
            elif not display_names and not self.previous_topic_data:  # Should be covered by first case
                items.append(ListItem(Label("[No topics found]")))
            else:
                for name_str in display_names:
                    # Ensure name_str is actually a string
                    if not isinstance(name_str, str):
                        items.append(ListItem(Label(f"[Error: Invalid topic name type ({type(name_str)})]")))
                        continue

                    if search_term and search_term in name_str.lower():
                        start_index = name_str.lower().find(search_term)
                        end_index = start_index + len(search_term)
                        display_text_markup = (
                            f"{escape_markup(name_str[:start_index])}"
                            f"[b yellow]{escape_markup(name_str[start_index:end_index])}[/b yellow]"
                            f"{escape_markup(name_str[end_index:])}"
                        )
                        items.append(ListItem(Label(display_text_markup, shrink=False)))  # Changed here
                    else:
                        items.append(ListItem(Label(escape_markup(name_str), shrink=False)))

            self.listview.extend(items)

            if items:
                if current_index is not None and 0 <= current_index < len(items):
                    self.listview.index = current_index
                elif len(items) > 0:
                    self.listview.index = 0
            # If no items, index will be None, which is fine.

        except Exception as e:
            # This is a fallback error display if _refresh_display_list itself fails
            self.listview.clear()  # Clear again to ensure no partial content
            self.listview.append(ListItem(Label(f"Error rendering topic list: {escape_markup(str(e))}")))

    def action_show_topic_info(self) -> None:
        """Show detailed information about the selected topic in a modal."""
        if self.listview.index is None:
            return

        # Check if previous_topic_data contains an error
        if self.previous_topic_data.get("error") is not None:
            self.app.bell()
            return

        # Ensure previous_topic_data is not empty and is a dict of topics
        if not self.previous_topic_data or "error" in self.previous_topic_data:  # "error" check might be obsolete
            self.app.bell()
            return

        # Get the currently displayed (and potentially filtered) names
        # This requires knowing what's actually in the ListView items
        # For simplicity, let's re-derive the displayed names if searching
        search_term = self.search_input.value.lower() if self.search_input.display else ""
        all_topic_names = sorted(list(self.previous_topic_data.keys()))

        displayed_names = all_topic_names
        if search_term:
            displayed_names = [name for name in all_topic_names if search_term in name.lower()]

        if self.listview.index is None or not (0 <= self.listview.index < len(displayed_names)):
            return

        selected_topic_name = displayed_names[self.topic_list_view.index]
        if selected_topic_name == "[No topics found]" or not selected_topic_name.startswith("/"):
            # Add a message to the app's log or a status bar if available
            self.app.bell()  # Simple feedback
            return

        self.app.push_screen(TopicInfoModal(topic_name=selected_topic_name))

    def action_echo_topic(self) -> None:
        """Echo the selected topic in a modal dialog."""
        if self.listview.index is None:
            return

        if self.previous_topic_data.get("error") is not None:
            self.app.bell()
            return

        if not self.previous_topic_data or "error" in self.previous_topic_data:  # "error" check might be obsolete
            self.app.bell()
            return

        # Similar logic as action_show_topic_info to get the correct selected topic
        search_term = self.search_input.value.lower() if self.search_input.display else ""
        all_topic_names = sorted(list(self.previous_topic_data.keys()))

        displayed_names = all_topic_names
        if search_term:
            displayed_names = [name for name in all_topic_names if search_term in name.lower()]

        if self.listview.index is None or not (0 <= self.listview.index < len(displayed_names)):
            return

        selected_topic_name = displayed_names[self.topic_list_view.index]

        if selected_topic_name == "[No topics found]" or selected_topic_name.startswith("[No topics match") or not selected_topic_name.startswith("/"):
            self.app.bell()
            return

        # Get type from the original self.previous_topic_data
        selected_topic_type = self.previous_topic_data.get(selected_topic_name)

        if selected_topic_type is None:
            self.app.bell()
            # self.app.notify(f"Error: No type found for topic {selected_topic_name}", severity="error", timeout=3)
            return

        self.app.push_screen(TopicEchoModal(topic_name=selected_topic_name, topic_type=selected_topic_type))

    def on_list_view_highlighted(self, event):
        """Handle when a topic is highlighted/selected in the ListView."""
        try:
            index = self.listview.index
            print(f"[TOPIC LIST] Highlighted event, index: {index}")
            
            if index is None or index < 0 or index >= len(self.topic_list_view.children):
                self.selected_topic_name = None
                print("[TOPIC LIST] Invalid index")
                return

            selected_item = self.listview.children[index]
            if not selected_item.children:
                self.selected_topic_name = None
                print("[TOPIC LIST] No children")
                return

            child = selected_item.children[0]
            topic_name = str(child.renderable).strip()
            print(f"[TOPIC LIST] Topic name: '{topic_name}'")

            if self._current_topic == topic_name:
                print(f"[TOPIC LIST] Same topic, skipping: {topic_name}")
                return

            # Only process valid topic names (should start with /)
            if topic_name.startswith("/"):
                self.selected_topic_name = topic_name
                self._current_topic = topic_name
                print(f"[TOPIC LIST] Selected topic: {topic_name}")

                if self._highlight_task and not self._highlight_task.done():
                    print("[TOPIC LIST] Cancelling previous task")
                    self._highlight_task.cancel()
                
                print("[TOPIC LIST] Starting delayed update task")
                self._highlight_task = asyncio.create_task(self._delayed_update())
            else:
                self.selected_topic_name = None
                print(f"[TOPIC LIST] Invalid topic name: {topic_name}")

        except Exception as e:
            print(f"[topic highlight error] {e}")
            import traceback
            traceback.print_exc()
            self.selected_topic_name = None

    async def _delayed_update(self):
        """Delayed update similar to NodeListWidget."""
        await asyncio.sleep(self._highlight_delay)
        await self._update_topic_display_async()

    async def _update_topic_display_async(self):
        """Update the topic display asynchronously."""
        if not self.selected_topic_name or not self.selected_topic_name.startswith("/"):
            return

        try:
            # Check if we're in topics mode
            if hasattr(self.app, 'current_right_pane_config') and self.app.current_right_pane_config == "topics":
                # Update Info tab directly like NodeListWidget does
                try:
                    info_widget = self.app.query_one("#topic-info-view-content")
                    info_widget.update_topic_info(self.selected_topic_name)
                    print(f"[TOPIC LIST] Updated info for: {self.selected_topic_name}")
                except Exception as e:
                    print(f"[TOPIC LIST] Error updating info: {e}")
                
                # Update Echo tab directly  
                try:
                    echo_widget = self.app.query_one("#echo-view-content")
                    echo_widget.start_echo(self.selected_topic_name)
                    print(f"[TOPIC LIST] Updated echo for: {self.selected_topic_name}")
                except Exception as e:
                    print(f"[TOPIC LIST] Error updating echo: {e}")
            else:
                print(f"[TOPIC LIST] Not in topics mode: {getattr(self.app, 'current_right_pane_config', 'unknown')}")
                
        except Exception as e:
            print(f"[TOPIC LIST] Error updating topic display: {e}")
