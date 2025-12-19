# Behavior Tree ROS2
To use BT.CPP with ROS2, we've created a small demo to showcase utility.

1. Once inside the container run `ros2 run amiga_ros2_behavior_tree dummy_wp_server`. This is a mock GPS waypoint navigator for the purposes of testing out your tree design.
2. In another shell run, `ros2 run amiga_ros2_behavior_tree bt_runner`.
3. Send a valid BT.CPP XML over TCP to the runner node. You can accomplish this from your host machine by running `nc 0.0.0.0 12346` and pasting in your XML text.
4. Use `amiga_ros2_behavior_tree/examples/sample_leafs.xml` as an example and paste it in the `netcat` shell.
5. `CTRL-C` the `netcat` shell to flush the socket and you'll see the tree execute:
```bash
(.venv) root@marcos-pc:/amiga-ros2-bridge# ros2 run amiga_ros2_behavior_tree bt_runner
[INFO] [1760349524.136186461] [bt_runner]: Waiting for mission on TCP port 12346...
[INFO] [1760349673.183670841] [bt_runner]: Received mission (2298 bytes)
[INFO] [1760349673.193118021] [bt_runner]: Starting mission execution...
[INFO] [1760349673.193161498] [bt_runner]: Moving to GPS location: (37.26644032, -120.42069862)
[INFO] [1760349673.893580403] [bt_runner]: Navigation finished successfully: 4
...
```

> Note, we use XML syntax validation via XSD for our behavior tree XML. This isn't standard practice, but we find it makes our pipeline more secure. You can edit the `bt_runner` node however you see fit for BT.CPP support.

## Mocks
- `/navigate_to_pose` mock -> dummy_nav2_ntp_server
- `/navigate_to_pose_in_frame` mock -> dummy_ntp_server
- `follow_gps_waypoints` mock -> dummy_wp_server

## Orchard Management Service
- **Service:** `/orchard/get_tree_placements` (`example_interfaces/srv/Trigger`)
- **Behavior:** On request, the node opens a TCP server on the configured `port` (default 12346), reads a length-prefixed JSON payload, and returns the raw JSON string in `response.message`.
- **JSON Format:** List of dictionaries with keys: `tree_index`, `row`, `col`, `lat`, `lon`.

### Run
```bash
ros2 run amiga_ros2_behavior_tree orchard_management_service_node
```

### Call
```bash
ros2 service call /orchard/get_tree_placements example_interfaces/srv/Trigger
```

### Configure
- `port` (int): TCP port to listen on (default 12346)
- `payload_length_included` (bool): If `true`, expects a 4-byte big-endian length before the JSON payload (default true)

Example with parameters:
```bash
ros2 run amiga_ros2_behavior_tree orchard_management_service_node --ros-args -p port:=12347 -p payload_length_included:=true
```
