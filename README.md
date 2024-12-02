Topics published by amigra_streams node: 

/canbus/twist

/filter/state

/gps/pvt

/oak0/imu

/oak0/left


Topics published by twist_control node: 

/amiga/cmd_vel

/amiga/vel


important: change the path to service_config.json in amiga_streams node to your local path. 


To setup the environment, you'll be working in Docker on your local machine. In theory, this should also work on the Amiga, but I haven't tested it yet.
It will communicate to the Amiga via Tailscale using gRPC.

To start, make your local Docker network to connect your VNC client, local machine, and Amiga together. You'll use this later when remote controlling the Amiga.
```bash
make network
```

After, build your ROS1 container:
```bash
make build-image
```

Next, standup the VNC container to forward X11 to your web browser. You can see this at `localhost:8080`.
```bash
make vnc
```

Finally, standup your ROS1 container:
```bash
make ros-bridge
```

This last step makes the assumption you have installed Tailscale and configured it with FarmNG.
Once you have the environment working, you'll follow FarmNG instructions below to run their 2 default nodes to expose telemetry and command and control.
Read https://github.com/farm-ng/amiga-ros-bridge and follow the instructions once in the container.
