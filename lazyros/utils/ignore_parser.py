import re
import os

class IgnoreParser:
    def __init__(self, ignore_file_path):
        self.ignore_file_path = ignore_file_path
        self.ignore_patterns = self._load_ignore_patterns()

    def _load_ignore_patterns(self):
        patterns = {
            'node': [],
            'topic': [],
            'parameter': [],
            'service': [],
        }
        if not os.path.exists(self.ignore_file_path):
            print(f"Ignore file not found at {self.ignore_file_path}. No filtering will be applied.")
            return patterns

        current_type = None
        try:
            with open(self.ignore_file_path, 'r') as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith('#'):
                        continue # Skip empty lines and comments
                    
                    if line.startswith('node'):
                        current_type = 'node'
                    elif line.startswith('topic'):
                        current_type = 'topic'
                    elif line.startswith('parameter'):
                        current_type = 'parameter'
                    elif line.startswith('service'):
                        current_type = 'service'
                    elif current_type:
                        # Add any non-empty, non-comment line after a type header as a pattern
                        patterns[current_type].append(self._glob_to_regex(line))
        except Exception as e:
            print(f"Error parsing ignore file {self.ignore_file_path}: {e}. No filtering will be applied.")
        return patterns

    def _glob_to_regex(self, glob_pattern):
        """Converts a glob pattern to a regex pattern."""
        # Escape special regex characters
        regex = re.escape(glob_pattern)
        # Replace glob '*' with regex '.*'
        regex = regex.replace(r'\*', '.*')
        # Replace glob '?' with regex '.'
        regex = regex.replace(r'\?', '.')
        # Anchor the regex to match the whole string
        regex = f"^{regex}$"
        return regex

    def should_ignore(self, item_name, item_type):
        """Checks if an item name should be ignored based on its type."""
        if item_type not in self.ignore_patterns:
            return False
        for pattern in self.ignore_patterns[item_type]:
            if re.fullmatch(pattern, item_name):
                return True
        return False

if __name__ == "__main__":
    # Example usage
    ignore_parser = IgnoreParser('../config/display_ignore.yaml')
    print(ignore_parser.should_ignore('/move_group: move', 'parameter'))  # Adjust the test case as needed