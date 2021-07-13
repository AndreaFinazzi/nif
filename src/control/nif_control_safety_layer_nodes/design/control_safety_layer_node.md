# Control Safety Layer
This package interfaces with the car actuators. 

### IAC specific
In the IAC case, the topic published by control safety layer are subscribed by the Raptor interface.
In particular:

- `steering_control_cmd` -> `/raptor_dbw_interface/accelerator_pedal_cmd`
- `accelerator_control_cmd` -> `/raptor_dbw_interface/brake_cmd`
- `braking_control_cmd` -> `/raptor_dbw_interface/steering_cmd`
- `gear_control_cmd` -> `/raptor_dbw_interface/gear_cmd`

Message specification:

```python
#
# Brake pressure in Pascal [Pa] from 0 to 3447379 (res: 55158)
#
float32 braking_control_cmd

#
# Accelerator percentage [%] from 0 to 100 (res: 0.1)
#
float32 accelerator_control_cmd

#
# Steering angle in Degrees from -600 to 600 (res: 1)
#
float32 steering_control_cmd

#
# Desired gear from -1 to 6
#
int8 gear_control_cmd
```