# Keystroke
This package republishes local keystrokes as ROS2 messages.

## ROS API
### Launch files

- Keyboard listener `listen.launch.py`
- Translate arrow keys to rover Twist messages `teleop_twist.py`


`ros2 launch keystroke listen.launch.py`

```
C:\Users\dan\Documents\ros2_ws>ros2 launch keystroke listen.launch.py
[INFO] [launch]: process[keystroke_listen.EXE-1]: started with pid [22940]
[INFO] [keystroke_listen]: To end this node, press the escape key
[INFO] [keystroke_listen]: pressed 'a'
[INFO] [keystroke_listen]: pressed 'e'
[INFO] [keystroke_listen]: pressed 'u'
[INFO] [keystroke_listen]: pressed space (32)
[INFO] [keystroke_listen]: pressed shift_r (161)
[INFO] [keystroke_listen]: pressed f1 (112)
[INFO] [keystroke_listen]: pressed esc (27)
[INFO] [keystroke_listen]: stopping listener
[INFO] [launch]: process[keystroke_listen.EXE-1]: process has finished cleanly
```

`ros2 launch keystroke telop_twist.launch.py`

```
C:\Users\dan\Documents\ros2_ws\src>ros2 launch keystroke keyboard_teleop_twist.launch.py
[INFO] [launch]: process[keystroke_listen.EXE-1]: started with pid [24820]
[INFO] [launch]: process[keystroke_arrows_to_twist.EXE-2]: started with pid [9908]
[INFO] [arrows_to_twist]: Arrows to twist node ready - F1 for usage
[INFO] [arrows_to_twist]: Use the arrow keys to change speed.
[F1] = Show this help
[Up]/[Down] = Forward and backward
[Left]/[Right] = Clockwise and counterclockwise
[INFO] [arrows_to_twist]: Increasing forward speed
[INFO] [arrows_to_twist]: Increasing forward speed
[INFO] [arrows_to_twist]: Increasing clockwise speed
[INFO] [arrows_to_twist]: Increasing clockwise speed
[INFO] [arrows_to_twist]: resetting speed
```

### keystroke_listen Node
Run with `ros2 run keystroke kesytroke_listen`

Parameters:
- exit_on_esc (boolean, default True): Whether the escape key should cause this node to exit

Published Topics:
- /glyphkey_pressed (String) - Pressing a letter/number/symbol key
- /key_pressed (UInt32) - Pressing spacebar, ctrl, F1, etc.

### arrows_to_twist Node
Run with `ros2 run keystroke_arrows_to_twist`

Parameters:
- publish_period (float): How often to publish the speed
- linear_scale (float): How much to increase/decrease speed based on up/down arrows
- angular_scale (float): How much to increase/decrease angular speed based on left/right arrows

Subscribed topics:
- /key_pressed (Uint32) - Keypresses to translate to speed commands

Published topics:
- /cmd_vel (Twist) - Velocity (linear and angular) commands
