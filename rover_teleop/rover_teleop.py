import os
import signal
import keyboard
import rclpy
from rover_teleop.teleop import Teleop

class RoverTeleop(Teleop):
    def __init__(self):
        super().__init__()
        self.keys_bindings = {
            "w": (self.LINEAR_MAX, 0.0),
            "s": (-self.LINEAR_MAX, 0.0),
            "a": (0.0, self.ANGULAR_MAX),
            "d": (0.0, -self.ANGULAR_MAX),
            "up": (self.LINEAR_MAX, 0.0),
            "down": (-self.LINEAR_MAX, 0.0),
            "left": (0.0, self.ANGULAR_MAX),
            "right": (0.0, -self.ANGULAR_MAX),
        }

        # Track pressed keys
        self.pressed_keys = set()

        # Register hotkeys for press and release
        for key in self.keys_bindings:
            keyboard.on_press_key(key, lambda e: self.handle_key_event(e, self.on_press))
            keyboard.on_release_key(key, lambda e: self.handle_key_event(e, self.on_release))

        # Register quit hotkey
        keyboard.on_press_key('q', lambda e: self.handle_key_event(e, lambda _: os.kill(os.getpid(), signal.SIGINT)))

        self.get_logger().info(f"""
This node takes keypresses from the keyboard and publishes them
as Twist messages.

Controls:

WASD or Arrows to move
Any other key to stop
CTRL-C or q to quit

Configuration:

Max Linear Speed: +/-{self.LINEAR_MAX} m/s
Max Angular Speed: +/-{self.ANGULAR_MAX} rad/s
""")

    def is_terminal_focused(self):
        try:
            if os.name == 'nt':  # Windows
                import win32gui
                window_title = win32gui.GetWindowText(win32gui.GetForegroundWindow()).lower()
                terminal_names = [
                    'powershell',
                    'cmd.exe',
                    'command prompt',
                    'windows terminal'
                ]
                return any(name in window_title for name in terminal_names)
            else:  # Linux/Unix
                try:
                    import subprocess
                    active_window_cmd = "xdotool getwindowfocus getwindowname"
                    active_window = subprocess.check_output(active_window_cmd, shell=True).decode().strip().lower()
                    return any(term in active_window for term in ['terminal', 'bash', 'shell'])
                except:
                    return True  # Fallback for systems without xdotool
        except:
            return True  # Fallback if window checking fails

    def handle_key_event(self, event, callback):
        if self.is_terminal_focused():
            callback(event)

    def on_press(self, event):
        key = event.name
        if key in self.keys_bindings:
            self.pressed_keys.add(key)
            self.update_movement()

    def on_release(self, event):
        key = event.name
        if key in self.keys_bindings:
            self.pressed_keys.discard(key)
            self.update_movement()

    def update_movement(self):
        linear = 0.0
        angular = 0.0

        # Calculate combined movement based on pressed keys
        for key in self.pressed_keys:
            binding = self.keys_bindings[key]
            linear += binding[0]
            angular += binding[1]

        # # Normalize diagonal movement
        # if abs(linear) > 0 and abs(angular) > 0:
        #     linear *= 0.7071  # 1/√2 to maintain consistent speed in diagonal movement
        #     angular *= 0.7071

        self.write_twist(linear, angular)

def main():
    try:
        rclpy.init()
        node = RoverTeleop()
        rclpy.spin(node)
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()