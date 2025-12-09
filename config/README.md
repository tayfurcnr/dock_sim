# Dock Station Configuration Guide

This directory contains configuration files for the dock station simulation.

## Configuration Files

### 1. `dock_config.yaml`
Controls the behavior of the dock covers (movement parameters).

#### Parameters:

**Movement Distances:**
- `open_position` (float, meters): Distance the covers move when opening
  - Default: `0.2`
  - Range: `0.0` to `0.5` (limited by joint constraints in URDF)
  - Example: Set to `0.15` for 15cm opening, or `0.4` for 40cm opening
  - To increase beyond 0.5m, edit joint limits in `urdf/dock_station.urdf.xacro`

- `closed_position` (float, meters): Position when covers are closed
  - Default: `0.0`
  - Typically should remain at `0.0`

**Movement Directions:**
- `left_direction` (float): Direction multiplier for left cover
  - Default: `1.0` (normal direction: +Y axis)
  - Set to `-1.0` to reverse direction

- `right_direction` (float): Direction multiplier for right cover
  - Default: `1.0` (normal direction: -Y axis)
  - Set to `-1.0` to reverse direction

**Speed Control (Not directly used, for reference):**
- `controller_p_gain` (float): Proportional gain
  - Default: `100.0`
  - Higher = faster response, may oscillate
  - Lower = slower, smoother movement

- `controller_i_gain` (float): Integral gain
  - Default: `0.01`
  - Eliminates steady-state error

- `controller_d_gain` (float): Derivative gain
  - Default: `10.0`
  - Reduces overshoot and oscillation

**Detection Parameters:**
- `position_tolerance` (float, meters): Tolerance for "arrived" detection
  - Default: `0.01` (1cm)
  - Used to determine when cover reached target position

- `control_rate` (integer, Hz): Update frequency
  - Default: `10`
  - How often state is published

#### Example Configuration:

```yaml
# Slower, shorter opening
dock_station:
  open_position: 0.15          # Open only 15cm instead of 20cm
  closed_position: 0.0
  left_direction: 1.0
  right_direction: 1.0
  position_tolerance: 0.02     # More lenient tolerance
  control_rate: 20             # Faster update rate
```

### 2. `dock_station_control.yaml`
Controls the ROS controllers (PID gains for actual movement speed).

#### Parameters:

**PID Gains (per controller):**
- `p` (Proportional): Main gain controlling response speed
  - Default: `100.0`
  - **Increase** for faster movement (may overshoot)
  - **Decrease** for slower, smoother movement

- `i` (Integral): Eliminates steady-state error
  - Default: `0.01`
  - Usually keep low

- `d` (Derivative): Dampens oscillation
  - Default: `10.0`
  - **Increase** to reduce overshoot
  - **Decrease** if movement is too sluggish

#### Example: Faster Movement

```yaml
dock_station:
  left_slider_position_controller:
    type: position_controllers/JointPositionController
    joint: left_slider
    pid: {p: 200.0, i: 0.05, d: 20.0}  # Doubled P and D gains

  right_slider_position_controller:
    type: position_controllers/JointPositionController
    joint: right_slider
    pid: {p: 200.0, i: 0.05, d: 20.0}
```

#### Example: Slower, Smoother Movement

```yaml
dock_station:
  left_slider_position_controller:
    pid: {p: 50.0, i: 0.01, d: 5.0}  # Halved gains

  right_slider_position_controller:
    pid: {p: 50.0, i: 0.01, d: 5.0}
```

## Changing Joint Limits in URDF

If you need to allow the covers to move beyond 0.5m, edit the URDF xacro properties:

**File**: `urdf/dock_station.urdf.xacro`

Find the properties section at the top:
```xml
<xacro:property name="joint_lower_limit" value="0.0" />
<xacro:property name="joint_upper_limit" value="0.5" />
<xacro:property name="joint_effort_limit" value="100" />
<xacro:property name="joint_velocity_limit" value="1.0" />
```

Change `joint_upper_limit` to your desired maximum:
```xml
<xacro:property name="joint_upper_limit" value="1.0" />  <!-- Allow up to 1 meter -->
```

**Model Scaling:**
- `model_scale`: Scale factor for all meshes
  - Default: `0.6` (60% of original size)
  - Example: `1.0` for 100% (full size), `0.5` for 50%, `1.5` for 150%
  - Scales visual and collision meshes uniformly

**Other configurable properties:**
- `joint_lower_limit`: Minimum position (default: 0.0m)
- `joint_upper_limit`: Maximum position (default: 2.5m)
- `joint_effort_limit`: Maximum force (default: 100N)
- `joint_velocity_limit`: Maximum speed (default: 1.0 m/s)
- `joint_damping`: Joint damping coefficient (default: 0.7)
- `joint_friction`: Joint friction (default: 0.0)

After changing URDF properties, you must restart the simulation for changes to take effect.

## Quick Tuning Guide

### To change opening distance:
Edit `dock_config.yaml`:
```yaml
open_position: 0.15  # Change from 0.2 to 0.15 for shorter opening
```

### To change movement speed:
Edit `dock_station_control.yaml`:
```yaml
pid: {p: 200.0, i: 0.01, d: 20.0}  # Increase P gain for faster movement
```

### To reverse a cover direction:
Edit `dock_config.yaml`:
```yaml
left_direction: -1.0  # Reverses left cover direction
```

### To scale the entire model:
Edit `urdf/dock_station.urdf.xacro`:
```xml
<!-- Scale to 60% of original size -->
<xacro:property name="model_scale" value="0.6" />

<!-- Or 80% -->
<xacro:property name="model_scale" value="0.8" />

<!-- Or full size -->
<xacro:property name="model_scale" value="1.0" />
```

## Applying Changes

After modifying configuration files, restart the simulation:
```bash
# Stop the current simulation (Ctrl+C in launch terminal)
# Then restart
roslaunch dock_station_sim sim_start.launch
```

Or use dynamic reconfigure (if implemented):
```bash
rosrun rqt_reconfigure rqt_reconfigure
```

## Dock Control Interface

The dock station now supports both **Action Server** (for advanced control with feedback) and **Service** (for simple debugging).

### Action Server (Recommended for Production)

The action server provides full control with real-time feedback and the ability to stop operations mid-process.

**Action Topic**: `/dock_station/lid_control`

**Commands** (from `dock_msgs/Commands`):
- `Commands.DOCK_LID_OPEN = 1`: Open the lid
- `Commands.DOCK_LID_CLOSE = 0`: Close the lid
- `Commands.DOCK_LID_STOP = 2`: Stop current motion

**Result/Feedback** includes `dock_msgs/Lid` message with:
- `state`: Current lid state (STATE_UNKNOWN, STATE_OPENING, STATE_CLOSING, STATE_OPEN, STATE_CLOSED, STATE_STOPPED, STATE_ERROR)
- `percentage`: Progress percentage (0-100, or -1 if not available)
- `ts`: ISO-8601 timestamp

**Example using Python**:
```python
import rospy
import actionlib
from dock_station_sim.msg import LidControlAction, LidControlGoal
from dock_msgs.msg import Commands

# Create action client
client = actionlib.SimpleActionClient('/dock_station/lid_control', LidControlAction)
client.wait_for_server()

# Open the lid
goal = LidControlGoal()
goal.command = Commands.DOCK_LID_OPEN
goal.transaction_id = "my_transaction_123"
goal.ts = rospy.get_rostime().to_sec()
goal.params = ""  # Optional JSON parameters

client.send_goal(goal)
client.wait_for_result()
result = client.get_result()

# result.lid contains dock_msgs/Lid with state, percentage, ts
print(f"Success: {result.success}")
print(f"State: {result.lid.state}, Percentage: {result.lid.percentage}%")
print(f"Message: {result.message}")
```

**Example using command line**:
```bash
# Open lid
rostopic pub /dock_station/lid_control/goal dock_station_sim/LidControlActionGoal "goal: {command: 1, transaction_id: 'test', ts: 'now', params: ''}"

# Close lid
rostopic pub /dock_station/lid_control/goal dock_station_sim/LidControlActionGoal "goal: {command: 0, transaction_id: 'test', ts: 'now', params: ''}"

# Stop lid
rostopic pub /dock_station/lid_control/goal dock_station_sim/LidControlActionGoal "goal: {command: 2, transaction_id: 'test', ts: 'now', params: ''}"

# Monitor feedback
rostopic echo /dock_station/lid_control/feedback

# Monitor result
rostopic echo /dock_station/lid_control/result
```

### Service Interface (For Debugging)

A simplified service wrapper is provided for easy testing and debugging.

**Service**: `/dock_station/dock_control`

**Commands**: `"open"`, `"close"`, `"stop"`

**Example using command line**:
```bash
# Open lid
rosservice call /dock_station/dock_control "command: 'open'"

# Close lid
rosservice call /dock_station/dock_control "command: 'close'"

# Stop lid
rosservice call /dock_station/dock_control "command: 'stop'"
```

**Example using Python**:
```python
import rospy
from dock_station_sim.srv import DockControl

rospy.wait_for_service('/dock_station/dock_control')
dock_control = rospy.ServiceProxy('/dock_station/dock_control', DockControl)

# Open the lid
response = dock_control('open')
print(f"Success: {response.success}, State: {response.state}, Message: {response.message}")
```

## Testing

Test the dock after configuration changes:
```bash
# Using service (easiest for debugging)
rosservice call /dock_station/dock_control "command: 'open'"
rosservice call /dock_station/dock_control "command: 'close'"
rosservice call /dock_station/dock_control "command: 'stop'"

# Monitor state
rostopic echo /dock_station/state

# Monitor joint positions
rostopic echo /dock_station/joint_states

# Monitor lid telemetry (conforms to DOCK_STATION_TELEMETRY.md)
rostopic echo /dock/rx/devices/telemetry/lid
```
