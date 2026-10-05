# Remote Control Protocol

## Connection

Run `ros2 run mecanum_gateway gateway_node`.

- TCP server: `0.0.0.0:8765` by default.
- UTF-8 newline-delimited JSON: one JSON object followed by `\n` per message.
  TCP reads may contain partial lines or multiple lines; clients must buffer accordingly.
- One connected client at a time. Additional connections are closed.
- Disconnecting does not stop the gateway. A new client can reconnect.
- Command results and telemetry share the same stream; distinguish them by `type`.
- No authentication or encryption is implemented; use a trusted network.

Startup ROS parameters:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `bind_address` | `0.0.0.0` | IPv4 listening address |
| `port` | `8765` | TCP port |
| `telemetry_rate` | `10.0` | State publication rate, Hz |
| `odom_timeout` | `1.0` | Maximum age of valid received odometry, seconds |
| `map_tf_future_tolerance` | `1.5` | Maximum allowed future offset of latest dynamic map TF, ROS seconds; accommodates AMCL transform_tolerance |
| `map_tf_stale_timeout` | `1.0` | Maximum age of latest dynamic map TF, ROS seconds; independent of odometry reception timeout |
| `linear_motion_threshold` | `0.01` | Moving threshold for planar speed, m/s |
| `angular_motion_threshold` | `0.01` | Moving threshold for absolute yaw rate, rad/s |
| `map_dir` | `~/mecanum_maps` | Map directory; `~` expands to the gateway user home |
| `map_save_timeout` | `30.0` | Map saver deadline in seconds before termination |
| `max_manual_linear_mps` | `0.30` | Symmetric clamp for each of vx/vy, m/s |
| `max_manual_angular_rps` | `1.00` | Symmetric clamp for wz, rad/s |
| `manual_cmd_timeout` | `0.30` | Nonzero manual command deadline, steady seconds |

Example: `ros2 run mecanum_gateway gateway_node --ros-args -p bind_address:=127.0.0.1 -p port:=8766`

## Robot state

The gateway subscribes to `/odometry/filtered` (`nav_msgs/msg/Odometry`) and sends
`robot_state` at approximately 10 Hz, including when no odometry is available.

```json
{"type":"robot_state","timestamp":0.0,"mode":"BASE","base_ready":false,"pose":{"x":0.0,"y":0.0,"yaw":0.0},"velocity":{"vx":0.0,"vy":0.0,"wz":0.0},"motion_state":"UNKNOWN","error":"odometry unavailable"}
```

- `timestamp`: telemetry generation time in ROS clock seconds.
- `pose`: odometry x/y in meters and quaternion-derived yaw in radians.
- `velocity`: odometry vx/vy in m/s and wz in rad/s.
- `map_pose`: optional map-frame `{x, y, yaw}` for the GUI overlay; the original
  `pose` remains in the odometry message frame. The gateway uses a nonblocking TF
  latest-TF lookup from that frame to `map`, without requiring TF history at the
  odometry message timestamp. Dynamic TF age must be within
  `[-map_tf_future_tolerance, map_tf_stale_timeout]`; future-dated AMCL transforms
  within this range are valid. Missing TF, stale odometry, TF outside these bounds,
  or BASE gives `map_pose:null`. Static TF is timeless. Old clients can ignore this field.
- `base_ready`: valid odometry was received within `odom_timeout`; freshness uses
  monotonic reception time, independently of the TCP connection.
- `motion_state`: `MOVING` when either speed threshold is exceeded, otherwise
  `STOPPED`. Missing or stale odometry gives `UNKNOWN`.
- Before the first valid odometry sample, pose/velocity are zero placeholders.
  Stale data retains the last values; clients must check `base_ready`.
- `error`: odometry availability/staleness and, if present, an unexpected mode-process
  exit. A process error remains until the next successful mode start; normal stop
  does not create a process error. Non-finite samples and invalid quaternions do
  not refresh readiness.
- `mode`: `BASE`, `MAPPING`, or `NAVIGATION`, initially `BASE`. Mode start success
  means process creation, not SLAM/Nav2 readiness. Mode returns to `BASE` after
  the managed process group has exited.

There is no separate heartbeat, ping, or pong. The Control PC determines connection
health from the last valid `robot_state`, with a 1.5-second reception timeout.
This is distinct from `base_ready`, which describes ROS data freshness.

## Commands

`get_status` acknowledges the request; current
robot details arrive through the periodic `robot_state` stream.

Request:
```json
{"command":"get_status","request_id":1}
```

Response:
```json
{"type":"command_result","request_id":1,"command":"get_status","success":true,"message":"ok"}
```

Unknown command:
```json
{"type":"command_result","request_id":2,"command":"unknown","success":false,"message":"unknown command"}
```

`request_id` is echoed and may be an integer, string, or null (omitted means null).
Malformed JSON or command objects return `success:false` with an explanatory
message; unavailable command/request identifiers are null. No state change occurs.
Commands are limited to 16 KiB per line. Excessive queued commands or unsent data
cause disconnection; telemetry is not retained across disconnected sessions.

## Mode commands

The base system must already be running. The gateway manages only
`mecanum.mapping.launch.py` and `mecanum.navigation.launch.py`, with
`use_sim_time:=false`. It never starts or stops the base launch.

### start_mapping

Allowed only in `BASE` with no previous operation still being cleaned up.

```json
{"command":"start_mapping","request_id":10}
{"type":"command_result","request_id":10,"command":"start_mapping","success":true,"message":"mapping started"}
```

Starting mapping or navigation while another mode is active returns
`success:false`, `message:"another mode is already running"`.

### start_navigation

Loads `<map_dir>/<map_name>.yaml`. The file must exist before launch starts.

```json
{"command":"start_navigation","request_id":11,"map_name":"room_01"}
{"type":"command_result","request_id":11,"command":"start_navigation","success":true,"message":"navigation started"}
```

A missing YAML returns `success:false`, `message:"map not found"`.

### stop_mode

```json
{"command":"stop_mode","request_id":12}
{"type":"command_result","request_id":12,"command":"stop_mode","success":true,"message":"mode stopped"}
```

The result arrives after process-group cleanup. Shutdown sends SIGINT, allows
3 seconds, then SIGTERM for 2 seconds, then SIGKILL if necessary. Telemetry
continues during the wait. An active map save is cancelled and receives a failure
result. Stopping idle `BASE` succeeds with `message:"already in BASE"`.
Gateway shutdown also cleans up managed groups. TCP disconnect alone does not
stop a mode; reconnect to observe it through `robot_state`.

### save_map

Allowed only in `MAPPING`, with one save at a time. Runs `nav2_map_server`
`map_saver_cli -f <map_dir>/<map_name>` and creates the directory if necessary.

```json
{"command":"save_map","request_id":13,"map_name":"room_01"}
{"type":"command_result","request_id":13,"command":"save_map","success":true,"message":"map saved"}
```

The result is sent after completion. Failure contains the exit code/signal or
cancellation/timeout reason and captured stderr when available. Telemetry and
other commands remain responsive. A second save receives
`success:false`, `message:"map save already in progress"`.

Names must contain 1–200 ASCII characters from `A-Z`, `a-z`, `0-9`, `_`, `-`, `.`;
`.` and `..` alone, path separators, and symbolic-link output files are rejected.
Use names without the `.yaml` extension. Default map saver output is YAML/PGM.
Results retain the original `request_id` and go only to the requesting TCP session.


## Manual velocity

Allowed only while MAPPING is running (not while stopping). BASE/NAVIGATION
returns `success:false`, `message:"manual velocity is only allowed in MAPPING mode"`.

```json
{"command":"manual_velocity","request_id":20,"vx":0.10,"vy":0.00,"wz":0.00}
{"type":"command_result","request_id":20,"command":"manual_velocity","success":true,"message":"manual velocity applied"}
```

All three fields must be finite JSON numbers; strings, booleans, null and missing
values are rejected. Non-finite JSON literals are invalid JSON. The gateway clamps
vx/vy individually to ±`max_manual_linear_mps` and wz to ±`max_manual_angular_rps`,
then publishes `geometry_msgs/msg/Twist` on `/cmd_vel`; all other components are 0.
Positive vx is forward, positive vy is left strafe, positive wz is left rotation.

Send while holding the button at about 10 Hz. Each nonzero command refreshes a
steady-clock deadline. After `manual_cmd_timeout` without a new nonzero command,
the gateway publishes one zero Twist and disarms the timeout. An explicit zero
also disarms it. MAPPING stop, client disconnect and normal Gateway shutdown
publish zero; shutdown publishes before the ROS context is closed. Old-session
velocity commands are rejected after disconnect. This does not periodically
publish zeros into NAVIGATION. Mode changes still use the existing commands.

## Map stream

`/map` (`nav_msgs/msg/OccupancyGrid`, reliable/transient-local) is retained as the
latest snapshot. A separate worker compresses changed map content/metadata at
most once per second, only in MAPPING/NAVIGATION. Header timestamp-only changes
are ignored. A new client receives the cached map once while that mode is active.
BASE clears the cached outgoing map; a frame already being sent may finish.

```json
{"type":"map","sequence":15,"resolution":0.05,"width":400,"height":400,"origin":{"x":-10.0,"y":-10.0,"qz":0.0,"qw":1.0},"encoding":"zlib+base64:int8","data":"<base64-encoded zlib bytes>"}
```

Decode base64, then zlib, to exactly `width * height` signed int8 bytes in ROS
row-major order (`index = y * width + x`). Byte 255 means -1/unknown; 0 means free;
100 means occupied. Values 1–99 are grayscale occupancy. Origin yaw comes from
qz/qw; the GUI flips image Y and applies origin rotation/translation and resolution
to `map_pose`. Only planar maps in the `map` frame are supported.

Each map is one NDJSON frame, separate from the 10 Hz `robot_state` stream. Keep
buffering until newline; do not assume a recv call contains the full map. Maps
are limited to 1,048,576 cells and clients allow up to 2 MiB per incoming frame.
The send path keeps one latest pending map rather than map history. Commands and
telemetry are queued ahead of a new map, but cannot interrupt an in-flight map
frame; a slow link may therefore trigger the client's telemetry timeout.
