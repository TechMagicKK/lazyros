# This scripts defines a inputable tree widget.
# Immitates the behavior of rqt_service_caller.

from rqt_py_common.message_helpers import get_service_class
from textual.app import App, ComposeResult
from textual.widgets import Label, Input, Button, Tree, Static, TextArea
from textual.containers import Horizontal, HorizontalScroll, Vertical, VerticalScroll, Container
from textual.widgets._tree import TreeNode
import copy
import array
import rclpy
from rclpy.node import Node
from trobo_interfaces.srv import GetBoxes


class ServiceParser:
    """
    This class has following functions:
    - This class parses a ROS service message and builds a tree structure.
    - This class creates input widgets for each field in the service message.
    - Finally, this class returns the service caller widget.
    """
    CSS = """
    TreeNode {
        height: 1;
        width: 1fr;
        background: #1f1f1f;;
    }
    Input {
        height: 1;
        border: none;
        background: #1f1f1f;
    }
    Tree {
        background: #1f1f1f;
    }
    Screen {
        layout: horizontal;
        overflow-x: auto;
    }
    Label {
        layout: horizontal;
        overflow-x: auto;
    }
    """

    def __init__(self):
        self.input_widgets = []
        self.output_widgets = []
        self.flattenpath = []
        self.flattenpaths = []

    def _recursive_parse(self, message, tree, mode='input'):
        """
        Recursively parses a ROS message and builds a tree widget.

        Parameters
        ----------
        message : ROS message
            The ROS message to parse.
        tree : TreeNode
            The tree widget to be populated.
        mode : str
            'input': for request message. Added input widgets to self.input_widgets.
            'output': for response message. Added output widgets to self.output_widgets.
        """
        if hasattr(message, 'get_fields_and_field_types'):
            # individual message
            for msg_name, msg_type in message.get_fields_and_field_types().items():
                # input /output widget
                if mode == 'input':
                    self.input_widgets.append(Label(''))
                elif mode == 'output':
                    self.output_widgets.append(Label(''))

                # tree widget
                node = tree.add(f'[bold]{msg_name}[/bold] [dim]({msg_type})[/dim]')
                node.expand()
                self.flattenpath.append(msg_name)

                # recursive call
                self._recursive_parse(getattr(message, msg_name), node, mode=mode)
                if len(self.flattenpath) > 0:
                    self.flattenpath.pop()

        elif type(message) in (list, tuple) and len(message) > 0 and hasattr(message[0], 'get_fields_and_field_types'):
            # array of individual messages
            # input / output widget
            if mode == 'input':
                self.input_widgets.append(Label(''))
            elif mode == 'output':
                self.output_widgets.append(Label(''))

            # array of messages
            for index, message_in_array in enumerate(message):
                msg_name, msg_type = message_in_array.get_fields_and_field_types().items()

                # tree widget
                node = tree.add(f'[bold]{msg_name}[/bold] [dim]({msg_type})[/dim]')
                self.flattenpath.append(msg_name)

                # recursive call
                self._recursive_parse(message_in_array, tree, mode=mode)
                if len(self.flattenpath) > 0:
                    self.flattenpath.pop()
        else:
            # built-in type
            # tree widget (leaf node)
            leafnode: TreeNode = tree.add_leaf(f'{str(repr(message))}')
            node_id = '#'.join(self.flattenpath)  # flatten path (uses in input widget)
            self.flattenpath.append(str(repr(message)))
            self.flattenpaths.append(self.flattenpath.copy())
            leafnode.data = self.flattenpath.copy()
            self.flattenpath.pop()

            if mode == 'input':
                # input widget (limit user input by message type)
                if type(message) is str:
                    self.input_widgets.append(Input(value=repr(message), type='text', name=node_id))
                elif type(message) is int:
                    self.input_widgets.append(Input(value=str(message), type='integer', name=node_id))
                elif type(message) is float:
                    self.input_widgets.append(Input(value=str(message), type='number', name=node_id))
                else:
                    self.input_widgets.append(Input(value=str(message), name=node_id))
                return
            elif mode == 'output':
                # output widget (limit user input by message type)
                node_id = 'output_' + node_id
                print('in leaf. output label is added.')
                if type(message) is str:
                    # scrollable label for long string
                    self.output_widgets.append(Label('', name=node_id))
                elif type(message) is int:
                    self.output_widgets.append(Label('', name=node_id))
                elif type(message) is float:
                    self.output_widgets.append(Label('', name=node_id))
                else:
                    self.output_widgets.append(Label('', name=node_id))
                return

        return tree, self.flattenpaths

    def parse_request_and_get_tree_widget(self, service_type: str):
        """
        Returns a textual tree widget that represents the request message in service.
        """
        print(f'Parsing request message of service {service_type}...')
        loaded_service_class = get_service_class(service_type)
        if loaded_service_class is None:
            raise ValueError(f"Service class for {service_type} not found.")
        else:
            print(f'Service class for {service_type} found.')
            tree = Tree(service_type)
            root = tree.root.add('Request')
            tree.root.expand_all()
            request_msg = loaded_service_class.Request()

            # FIXME: root class is constructed by _recursive_parse() and tree class has root.
            # Therefore tree has all constructed node info and works well.
            # However, _recursive_parse() should return tree and we use it.
            # Chaninging argument in the function is not good idea.
            _, treepaths = self._recursive_parse(request_msg, root, mode='input')
            return tree, self.input_widgets, treepaths

    def parse_response_and_get_tree_widget(self, service_type: str):
        """
        Returns a textual tree widget that represents the response message in service.
        """
        loaded_service_class = get_service_class(service_type)
        if loaded_service_class is None:
            raise ValueError(f"Service class for {service_type} not found.")
        else:
            tree = Tree(service_type)
            root = tree.root.add('Response')
            tree.root.expand_all()
            response_msg = loaded_service_class.Response()

            # FIXME: root class is constructed by _recursive_parse() and tree class has root.
            # Therefore tree has all constructed node info and works well.
            # However, _recursive_parse() should return tree and we use it.
            # Chaninging argument in the function is not good idea.
            _, treepaths = self._recursive_parse(response_msg, root, mode='output')
            return tree, self.output_widgets, treepaths


class ServiceCallView(App):
    CSS = """
    Tree {
        background: #1f1f1f;
    }
    Input {
        height: 1;
        border: none;
        background: #1f1f1f;
    }
    #call_service_button {
        background: gray;
        color: white;
        text-align: center;
        height: 3;
        width: 100%;
        margin: 0 0 1 0;
        padding: 0 1 0 1;
    }
    """
    CSS_PATH = 'vertical_layout.tcss'

    def __init__(self, service_name, service_type: str, ros_node: Node):
        super().__init__()
        self.service_name = service_name
        self.service_type = service_type
        self.node = ros_node

        self.service_listview = None
        self.selected_service = None
        self.current_service = None
    
    def update_display(self):
        self.service_listview = self.app.query_one('#service-listview')
        self.selected_service = self.service_listview.selected_service if self.service_listview else None

        if self.selected_service is None:
            self.clear()
            return
        
        if self.selected_service == self.current_service:
            return
        
        self.current_service = self.selected_service
        parser = ServiceParser()
        self.request_tree_widget, self.request_message_type_widgets, self.request_treepath = parser.parse_request_and_get_tree_widget(self.service_type)
        self.response_tree_widget, self.response_message_type_widgets, self.response_treepath = parser.parse_response_and_get_tree_widget(self.service_type)
        # START HERE



    def clear(self):
        """Clear all widget contents"""
        self.request_tree_widget = Tree('Service is not selected')
        self.response_tree_widget = Tree('Service is not selected')
        self.request_message_type_widgets = [Input('')]
        self.response_message_type_widgets = [Label('')]
        return


    def compose(self) -> ComposeResult:
        parser = ServiceParser()
        self.request_tree_widget, self.request_message_type_widgets, self.request_treepath = parser.parse_request_and_get_tree_widget(self.service_type)
        self.response_tree_widget, self.response_message_type_widgets, self.response_treepath = parser.parse_response_and_get_tree_widget(self.service_type)
        yield Button(f'📞 Call {self.service_type} service', id='call_service_button')
        with Vertical():
            with Horizontal():
                with Vertical(classes='column'):
                    yield self.request_tree_widget
                with Vertical(classes='column'):
                    yield Label('')
                    yield Label('')
                    for widget in self.request_message_type_widgets:
                        yield widget
            yield Static('')  # Placeholder for layout consistency
            yield Static('')  # Placeholder for layout consistency
            with HorizontalScroll():
                with VerticalScroll():
                    yield self.response_tree_widget
                with VerticalScroll():
                    yield Static('')
                    yield Static('')
                    for widget in self.response_message_type_widgets:
                        # FIXME: long message can't showed
                        widget.styles.text_overflow_x = 'scroll'
                        yield widget

    def _set_nested_attribute(self, obj, flatten_path: str, value):
        """
        Sets a value to nested class member without expanding with flatten_path.
        e.g. flatten_path = 'class_a#b#c', value = 10 
            does the same thing as 'class_a.b.c = value'
        This function is used to set value with any depth of nested class whenever you know the flattenpath.

        Parameters
        ----------
        obj : ROS message
            The ROS message to set value.
        flatten_path : str
            The flatten path to the member to set value. i.g. 'class_a#b#c'
        value : any
            The value to set.

        Returns
        -------
        obj_copy : ROS message
            The ROS message with the value set.

        """
        parts = flatten_path.split('#')
        obj_copy = copy.deepcopy(obj)
        current_obj = obj_copy
        array_type = None

        for part in parts[:-1]:
            current_obj = getattr(current_obj, part)

        # change type of vlaue to current object type
        if hasattr(current_obj, parts[-1]):
            attr_type = type(getattr(current_obj, parts[-1]))
            if attr_type is str:
                value = str(value)
            elif attr_type is int:
                value = int(value)
            elif attr_type is float:
                value = float(value)
            elif attr_type is bool:
                value = bool(value)
            elif attr_type is array.array:
                array_type = getattr(current_obj, parts[-1]).typecode
                # convert type of value from str to array.array
                if self._is_empty_array(value):
                    value = array.array(array_type)
                else:
                    value = value.replace('[', '').replace(']', '').replace(' ', '')
                    value = value.split(',')

                if array_type == 'i':  # int
                    value = array.array(array_type, [int(v) for v in value])
                elif array_type == 'f':  # float
                    value = array.array(array_type, [float(v) for v in value])
                elif array_type == 'B':  # uint8(byte)
                    value = array.array(array_type, [int(v) for v in value])

                value = array.array(array_type, value)

        setattr(current_obj, parts[-1], value)

        return obj_copy

    def _get_nested_attribute(self, obj, flatten_path: str):
        """
        Gets a value from nested class member without expanding with flatten_path.
        e.g. flatten_path = 'class_a#b#c' 
            does the same thing as 'value = class_a.b.c'
        This function is used to get value with any depth of nested class whenever you know the flattenpath.

        Parameters
        ----------
        obj : ROS message
            The ROS message to get value.
        flatten_path : str
            The flatten path to the member to get value. i.g. 'class_a#b#c'

        Returns
        -------
        value : any
            The value of the member.
        """
        parts = flatten_path.split('#')
        current_obj = copy.deepcopy(obj)

        try:
            for part in parts:
                current_obj = getattr(current_obj, part)
            return current_obj
        except AttributeError:
            print(f"Attribute not found in {current_obj}. Returning None.")
            return None

    def _is_empty_array(self, value: str) -> bool:
        """
        if value is 'array(Any)', user does not input any value.
        So checking length is not needed.
        """
        if value.startswith('array(') and value.endswith(')'):
            return True
        else:
            return False

    def on_button_pressed(self, event: Button.Pressed) -> None:
        """
        This function has following functions:
        - Makes a service request from the input widgets.
        - Calls the service with the request.
        """
        if event.button.id == 'call_service_button':
            print(f'Button {event.button.id} pressed.')
            loaded_service_class = get_service_class(self.service_type)
            if loaded_service_class is None:
                raise ValueError(f"Service class for {self.service_type} not found.")

            # Prepare the request
            request_msg = loaded_service_class.Request()
            for widget in self.query(Input):
                # Set the value to the corresponding field in the request message
                flatten_path = str(widget.name)
                request_msg = self._set_nested_attribute(request_msg, flatten_path, widget.value)
            print(f"Filled request message: {request_msg}")

            # call the service and wait
            print(f'Calling service {self.service_type}...')
            print(f'loaded_service_class: {loaded_service_class}')
            service_client = self.node.create_client(loaded_service_class, self.service_name)
            future = service_client.call_async(request_msg)
            while rclpy.ok() and not future.done():
                print('Waiting for service response...')
                rclpy.spin_once(self.node)
            if future.result() is None:
                return
            response_msg = future.result()
            print(f'Service response: {response_msg}')

            # reflesh output widgets to display response
            output_widgets = self.query(Label)
            for widget in output_widgets:
                widget_name = str(widget.name)
                if widget_name.startswith('output_'):
                    # remove 'output_' prefix
                    flatten_path = widget_name[7:]
                    # get value from request_msg
                    value = self._get_nested_attribute(response_msg, flatten_path)

                    # update widget value
                    widget.update(str(value))
        else:
            print(f'Button {event.button.id} pressed. No action defined.')


def main():
    rclpy.init()
    ros_node = Node('service_call_view_node')

    # service_type = 'trobo_interfaces/srv/DetectConveyor'
    service_type = 'trobo_interfaces/srv/GetBoxes'
    service_name = '/get_boxes'
    app = ServiceCallView(service_name, service_type, ros_node)
    app.run()


if __name__ == "__main__":
    main()
