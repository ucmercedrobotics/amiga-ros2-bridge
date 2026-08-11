# Behavior Tree ROS2

BT.CPP mission execution for the Amiga, plus the TCP intake and orchard
lookup that feed it. Three nodes, one launch file:

```
              TCP mission           /mission/xml            action servers
netcat/planner ────────────► tcp_demux_node ──────────► bt_runner ──────────► (nav, arm, ...)
                                     │                                            │
                                     ▼ /orchard/tree_info_json                    ▼
                            orchard_management_node ◄──── /orchard/get_tree_info ─┘
                              (amiga_interfaces/srv/GetTreeInfo)
```

`tcp_demux_node` owns the TCP port; `bt_runner` never touches a socket, it only
subscribes to `mission_topic`. Bring all three up together:

```bash
ros2 launch amiga_ros2_behavior_tree bt.launch.py
```

## Quick demo

1. Launch with a single unframed XML frame expected (no orchard JSON, no
   length prefix) — the simplest way to hand-feed a mission:
   ```bash
   ros2 launch amiga_ros2_behavior_tree bt.launch.py \
       expect_json:=false payload_length_included:=false
   ```
2. From your host machine, send `amiga_ros2_behavior_tree/examples/sample_leafs.xml`
   over TCP:
   ```bash
   nc 0.0.0.0 12346 < amiga_ros2_behavior_tree/examples/sample_leafs.xml
   ```
   (or `nc 0.0.0.0 12346` and paste the XML by hand, then `CTRL-C` the shell to
   flush the socket and close the connection.)
3. Watch the tree execute:
   ```bash
   (.venv) root@marcos-pc:/amiga-ros2-bridge# ros2 launch amiga_ros2_behavior_tree bt.launch.py expect_json:=false payload_length_included:=false
   [INFO] [tcp_demux]: tcp_demux: Waiting on port 12346...
   [INFO] [tcp_demux]: Published XML (2298)
   [INFO] [bt_runner]: Starting mission execution...
   [INFO] [bt_runner]: Mission finished with status: SUCCESS
   ...
   ```

With the defaults (`expect_json:=true`, `payload_length_included:=true`), a
mission is two length-prefixed frames on the same connection — XML then JSON
(`order:=json_then_xml` to swap) — and `orchard_management_node` also comes up
to serve the JSON frame back out over `/orchard/get_tree_info`. `nc` isn't
enough to produce length-prefixed frames by hand; use a real client (the fleet
planner, or `scripts/demo_llm_auction.sh`) to exercise that path.

> Note, we use XML syntax validation via XSD for our behavior tree XML (see
> `xml_validation`). This isn't standard practice, but we find it makes our
> pipeline more secure. You can edit the `bt_runner` node however you see fit
> for BT.CPP support.

## Action mocks

For testing tree designs without real navigation/arm/perception behind them:

| action (`action_name`) | mock executable |
| --- | --- |
| `navigate_to_pose` | `dummy_nav2_ntp_server` |
| `follow_gps_waypoints` | `dummy_wp_server` |
| `follow_tree_id_waypoint` (`MoveToTreeID`) | `dummy_tree_id_server` |
| `segment_leaves` (`SampleLeaf`) | `dummy_segment_leaves_server` |

Run whichever ones the mission needs, e.g. `ros2 run amiga_ros2_behavior_tree dummy_wp_server`.

## TCP mission intake (`tcp_demux_node`)

Owns the mission TCP port. One connection carries either a single XML frame or
(with `expect_json:=true`) an XML frame and a JSON orchard frame, and the node
publishes each onto a topic — it never parses either payload itself.

### Configure
- `port` (int): TCP server port (default `12346`)
- `payload_length_included` (bool): frames are 4-byte big-endian length-prefixed (default `true`)
- `order` (string): `xml_then_json` or `json_then_xml` (default `xml_then_json`)
- `expect_json` (bool): expect a second (orchard) frame after the XML (default `true`); requires `payload_length_included:=true`
- `default_frame_size` (int): cap in bytes when `payload_length_included:=false` (default `65536`)
- `mission_topic` / `orchard_topic` (string): where the two frames are published (default `mission/xml`, `orchard/tree_info_json`, both relative — see `bt.launch.py`'s namespace note)

### Fleet planner discovery

If a fleet planner is configured, `tcp_demux_node` registers with it, sends
periodic heartbeats, deregisters on clean shutdown, and writes a length-prefixed
JSON ack back on the same TCP connection once a mission is accepted or
rejected. The wire contract (register/heartbeat/deregister/ack bodies) lives in
[`include/amiga_ros2_behavior_tree/planner_client.hpp`](include/amiga_ros2_behavior_tree/planner_client.hpp)
and is implemented over a plain HTTP/1.1 socket in
[`src/planner_client.cpp`](src/planner_client.cpp) — no HTTP client dependency
for three tiny JSON calls on the local network.

- `planner_host` (string, default `""`): fleet planner host. **Empty disables
  discovery entirely** — a dev box driven by `nc` with no planner configured
  behaves exactly as if this feature didn't exist. `bt.launch.py` defaults this
  to `100.88.70.65`; pass `planner_host:=""` to opt out.
- `planner_port` (int, default `8003`)
- `robot_id` (string, default `""`): stable id sent to the planner. Empty falls
  back to the robot's namespace, and if that's also empty, to `hostname()`.
- `robot_name` (string), `schema` (default `amiga_btcpp`), `schema_sha256`,
  `capability_actions`, `has_manipulator`, `battery_pct`: registration fields,
  mostly optional (unset ones are simply omitted from the request body).
- `heartbeat_interval_s` (int, default `5`)
- `min_battery_pct_for_mission` (double, default `-1.0`, disabled): reject an
  incoming mission (ack `accepted: false`) below this battery level.

A 404 on heartbeat means the planner restarted and lost its roster; the node
re-registers automatically. Discovery runs on its own thread so a slow register
or heartbeat call never stalls TCP mission intake.

## Orchard lookup (`orchard_management_node`)

- **Service:** `/orchard/get_tree_info` (`amiga_interfaces/srv/GetTreeInfo`)
- **Behavior:** subscribes to the orchard JSON topic (`json_topic`, fed by
  `tcp_demux_node`'s second TCP frame) and caches the latest payload. A
  request names an `index_type` (tree/row/col/aisle) and a list of `indicies`
  to look up in the cached JSON, optionally filtered to `request.fields`
  (`tree_index`, `row`, `col`, `lat`, `lon`, `row_waypoints`); the response is
  a JSON string. A fixed metric offset (`x_offset`/`y_offset`, meters) is
  applied to any lat/lon pair returned. No JSON received yet -> an empty
  result, not an error.
- This is what `follow_tree_id_waypoint` (`MoveToTreeID`) resolves a tree index
  or aisle to a GPS point through.

### Configure
- `json_topic` (string): topic to read orchard JSON from (default
  `/orchard/tree_info_json`)
- `x_offset` / `y_offset` (double, meters, default `0.0`): added to
  east/west and north/south respectively on any returned lat/lon

```bash
ros2 run amiga_ros2_behavior_tree orchard_management_node --ros-args -p json_topic:=/orchard/tree_info_json
```

## Seeing the whole pipeline

`bt.launch.py` is one robot's mission layer. To watch it drive the full
replanning/coordination/auction stack across a simulated fleet, with a real
LLM behind both reasoning points:

```bash
export AGENT_MODEL=...
export AGENT_API_BASE=...
make llm-demo
```

See [`scripts/demo_llm_auction.sh`](../scripts/demo_llm_auction.sh) for what it
does and why, and
[`amiga_ros2_coordinator/docs/coordinator.md`](../amiga_ros2_coordinator/docs/coordinator.md)
for the pipeline it exercises.
