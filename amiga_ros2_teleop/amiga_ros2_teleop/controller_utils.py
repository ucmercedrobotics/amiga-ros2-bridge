import yaml
from sensor_msgs.msg import Joy

def load_controller_config(path: str) -> dict:
    """Loads yaml config for controller.

    Format:
    buttons:
        <name>: <idx>
        ...
    axes:
        <name>: <idx>
        ...
    """
    
    with open(path, 'r') as f:
        config = yaml.safe_load(f)
    
    if "buttons" not in config or "axes" not in config:
        raise ValueError("Config must contain 'buttons' and 'axes' at the top level")

    return config

class ControllerMap:
    """Maps `Joy` msg to a class given a config.
    Example usage:
        config = {
            "buttons": {"a": 0, "b": 1},
            "axes": {"x": 0, "y": 1}
        }
        msg = Joy()
        controller = ControllerMap(msg, config)
        controller.a # access attribute
    """
    def __init__(self, msg: Joy, config: dict):
        self._buttons = config["buttons"]
        self._axes = config["axes"]
        self._msg = msg
    
    def __getattr__(self, name):
        # -- Get button
        if name in self._buttons:
            idx = self._buttons[name]
            if idx < 0 and idx >= len(self._msg.buttons):
                raise IndexError(f"Button '{name}' index '{idx}' out of range.")
            return self._msg.buttons[idx]
        
        # -- Get axes
        elif name in self._axes:
            idx = self._axes[name]
            if idx < 0 and idx >= len(self._msg.axes):
                raise IndexError(f"Axis '{name}' index '{idx}' out of range.")
            return self._msg.axes[idx]
        
        raise AttributeError(f"No such button or axis: '{name}'")
    
    def get(self, name, default=None):
        try:
            return self.__getattr__(name)
        except (IndexError, AttributeError) as e:
            return default

class ButtonTrigger:
    """Ensures that an action isn't repeated if the button `key` is held for less than `max_hold_frames`."""
    def __init__(self, key: str, max_hold_frames: int = 1):
        self.key = key
        self.max_hold_frames = max_hold_frames

        # -- State
        self.press_count = 0
        self.was_pressed = False

    def query(self, controller: ControllerMap) -> bool:
        is_pressed = controller.get(self.key, 0) == 1
        ready = False
        
        if is_pressed and not self.was_pressed:
            self.press_count += 1
            self.was_pressed = True
            ready = True
        elif is_pressed and self.was_pressed:
            self.press_count += 1
            if self.press_count >= self.max_hold_frames:
                self.press_count = 0
                self.was_pressed = False
        else:
            self.press_count = 0
            self.was_pressed = False

        return ready