# SdkEventBridge

Intercept Unitree SDK2 sport commands over DDS, handle them in your own program, and republish them to ROS2 simulation topics (`/cmd_vel`, `/cmd_ctl`).

When a sport client (for example `b2_sport_client` or `go2_sport_client`) sends a command, Unitree SDK2 publishes on these internal DDS topics:

- `rt/api/sport/request`
- `rt/api/sport/response`

SdkEventBridge listens for those commands, handles them locally, and bridges events to ROS2 simulation topics.

## Features

- Intercept sport API requests before they reach the real robot service
- Reply immediately to the SDK client with status code `0` (success)
- Handle sport events by API ID:
  - DAMP
  - MOVE
  - BALANCESTAND
  - STOPMOVE
  - STANDUP
  - STANDDOWN
  - RECOVERYSTAND
  - FREEWALK
  - SWITCHMOVEMODE
  - SPEEDLEVEL
  - SWITCHGAIT
- Publish ROS2 `geometry_msgs/msg/Twist` to `/cmd_vel` on MOVE and STOPMOVE
- Publish ROS2 `std_msgs/msg/Int32` to `/cmd_ctl` for FSM / simulation commands
- Passive mode for observation-only use

## Requirements

- C++17 compiler
- [Unitree SDK2](https://github.com/unitreerobotics/unitree_sdk2) installed (`unitree_sdk2`, CycloneDDS)
- ROS 2 Humble (`rclcpp`, `geometry_msgs`, `std_msgs`)

## Build

```bash
source /opt/ros/humble/setup.bash

cd SdkEventBridge
mkdir -p build && cd build
cmake .. -DCMAKE_PREFIX_PATH="/opt/ros/humble"
cmake --build .
```

## Run

```bash
source /opt/ros/humble/setup.bash
./sdk_event_bridge_demo <networkInterface> [domainId] [mode]
```

Examples:

```bash
# Intercept mode on eth0 (default)
./sdk_event_bridge_demo eth0

# Local test without robot
./sdk_event_bridge_demo lo 0 intercept

# Passive observe-only mode
./sdk_event_bridge_demo eth0 0 passive
```

### Typical workflow

Terminal 1:

```bash
source /opt/ros/humble/setup.bash
./sdk_event_bridge_demo eth0
```

Terminal 2:

```bash
ros2 topic echo /cmd_vel geometry_msgs/msg/Twist
ros2 topic echo /cmd_ctl std_msgs/msg/Int32
```

Terminal 3:

```bash
./b2_sport_client eth0
```

Test examples:

- `5` or `move` → `/cmd_vel`
- `3` or `stand_down` → `/cmd_ctl` data=`10002`
- `0` or `damp` → `/cmd_ctl` data=`10003`
- stand up / recovery stand → `/cmd_ctl` data=`10001`

## Modes

| Mode | Description |
|------|-------------|
| `intercept` (default) | Acts as the sport server. Commands are handled locally and answered immediately. |
| `passive` | Subscribes to DDS topics and logs traffic without replying as the sport server. |

## ROS2 `/cmd_vel` mapping

On MOVE, the bridge parses the Unitree move JSON parameter and publishes a `Twist`:

| Unitree MOVE param | ROS2 field |
|--------------------|------------|
| `x` (vx) | `linear.x` |
| `y` (vy) | `linear.y` |
| `z` (vyaw) | `angular.z` |

Example Unitree parameter:

```json
{"x": 0.5, "y": 0.0, "z": 0.0}
```

Equivalent ROS2 message:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

On STOPMOVE, the bridge publishes zero velocity to `/cmd_vel`.

## SwitchMoveMode (continuous MOVE)

`SwitchMoveMode(bool flag)` switches how MOVE is bridged to `/cmd_vel`:

| `flag` | Behavior |
|--------|----------|
| `true` | Continuous response mode: latest MOVE velocity is kept until a new command or explicit stop |
| `false` | Default mode: if no new MOVE arrives within 1 second, `/cmd_vel` is published as zero (auto stop) |

Parameter JSON uses `{"data": true}` or `{"data": false}` (also accepts `flag`).

STOPMOVE, STANDDOWN, and DAMP always stop immediately regardless of mode.

## SpeedLevel (max linear speed)

`SpeedLevel(int level)` limits MOVE linear velocity magnitude on `/cmd_vel`:

| `level` | Mode | Max linear speed |
|---------|------|------------------|
| `-1` | Slow (default) | 1.5 m/s |
| `1` | Fast | 3.5 m/s |

Parameter JSON: `{"data": -1}` or `{"data": 1}`.

Linear velocity `(vx, vy)` is scaled down if its magnitude exceeds the current max. Angular `vyaw` is not clamped.

## Joint lock / unlock (MOVE gating)

MOVE commands are only published to `/cmd_vel` when joints are **unlocked**. Default state is **locked**.

| Event | Lock state | Effect |
|-------|------------|--------|
| STANDUP | Lock | Joints locked, MOVE ignored |
| STANDDOWN | Lock | Stop + lock, MOVE ignored |
| RECOVERYSTAND | Lock | Joints locked |
| DAMP | Lock | Stop + lock |
| BALANCESTAND | **Unlock** | MOVE allowed |
| SWITCHGAIT `0` | Lock | Locked standing |
| SWITCHGAIT `1-4` | **Unlock** | Movable gaits (walk modes) |

Typical flow: `STANDUP` (locked) → `BALANCESTAND` (unlocked) → `MOVE` works.

## ROS2 `/cmd_ctl` mapping (FSM / simulation)

Publishes `std_msgs/msg/Int32` with a 5-digit command code in `data`:

| Sport event | `/cmd_ctl` data | Meaning |
|-------------|-----------------|---------|
| STANDUP | `10001` | Get up → locomotion |
| BALANCESTAND | `10001` | Get up → locomotion |
| RECOVERYSTAND | `10001` | Get up → locomotion |
| STANDDOWN | stop `/cmd_vel` (zero) + `10002` on `/cmd_ctl` | Stop, then get down |
| DAMP | stop `/cmd_vel` (zero) + `10003` on `/cmd_ctl` | Stop, then passive (damped motors) |

Example:

```bash
ros2 topic pub /cmd_ctl std_msgs/msg/Int32 "{data: 10001}"
```

## Status codes

| Code | Meaning |
|------|---------|
| `0` | Success |
| `3104` | Client API timeout (robot did not respond in time) |
| other | See Unitree SDK internal / sport error definitions |

In intercept mode, the bridge responds with `0` immediately so the SDK client does not wait for the real robot.

## Important notes

1. Use the same network interface as your sport client (`eth0`, `lo`, etc.).
2. DDS is pub/sub. If the real robot is online on the same network and domain, it may still receive published requests.
3. To fully block the robot during development, keep it off the same DDS network or use an isolated interface such as `lo`.
4. `b2_sport_client` test move uses `Move(0.0, 0.0, 0.5)`, which maps to `angular.z = 0.5`, not forward motion. Use `Move(0.5, 0.0, 0.0)` for forward `/cmd_vel.linear.x = 0.5`.

## Library usage

```cpp
#include "sdk_event_bridge.hpp"
#include "cmd_vel_publisher.hpp"

SdkEventBridgeClass bridge(0, "eth0");
bridge.SetMode(BridgeMode::Intercept);
bridge.Init();

bridge.RegisterMoveHandler([](const SportEventResult& result) {
    if (result.IsSuccess()) {
        // handle move event
    }
});

bridge.Start();
```

Register handlers:

- `RegisterDampHandler()`
- `RegisterMoveHandler()`
- `RegisterBalanceStandHandler()`
- `RegisterStopMoveHandler()`
- `RegisterStandUpHandler()`
- `RegisterStandDownHandler()`
- `RegisterRecoveryStandHandler()`
- `RegisterFreeWalkHandler()`
- `RegisterSwitchMoveModeHandler()`
- `RegisterSpeedLevelHandler()`
- `RegisterSwitchGaitHandler()`

Or use generic handlers:

- `RegisterSportEventHandler(apiId, handler)`
- `SetSportRequestHandler(handler)`
- `SetSportResponseHandler(handler)`

## Project structure

```
SdkEventBridge/
├── CMakeLists.txt
├── README.md
├── include/
│   ├── sdk_event_bridge.hpp
│   ├── cmd_vel_publisher.hpp
│   └── cmd_ctl_publisher.hpp
└── src/
    ├── sdk_event_bridge.cpp
    ├── cmd_vel_publisher.cpp
    ├── cmd_ctl_publisher.cpp
    └── main.cpp
```

## License

Use and modify as needed for your robot integration project.
