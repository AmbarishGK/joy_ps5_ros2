# TouchpadInput.msg
# Represents a single touch event on a touchpad
std_msgs/Header header
# Single touch position
float32 x
float32 y

# Multi-touch information
uint32 tracking_id       # Unique identifier for a multi-touch point
float32 mt_x             # Multi-touch position X
float32 mt_y             # Multi-touch position Y

# Touch status
bool touch_detected      # True if touch is detected (BTN_TOUCH)
bool double_tap_detected # True if double tap is detected (BTN_TOOL_DOUBLETAP)

# Button states (like left button, finger touch, etc.)
bool left_button_pressed  # Left button press status
bool finger_tool_detected # Finger tool touch detected
