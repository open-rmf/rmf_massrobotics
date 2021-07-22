# RMF - Massrobotics integration efforts

![](../media/bridge.gif)

To build, clone this repository into your RMF workspace

```bash
cd rmf_ws/src
git clone ssh://git@github.com/open-rmf/rmf_massrobotics

cd ../
source /opt/ros/foxy/setup.bash
colcon build --packages-up-to rmf_massrobotics
```

To run,

```bash
# Start the MassRobotics Interop server, according to the repository instruction
npm run start

# Start the demo in one terminal
cd rmf_ws
source install/setup.bash
ros2 launch rmf_demos office.launch.xml

# Start the bridge on another
cd rmf_ws
source install/setup.bash
ros2 launch rmf_massrobotics bridge.launch.xml use_sim_time:=true
```

TODOs
* Refactor each json into it's own class, properly fill up each field.
* Refactor all the websocketpp components and methods into their own class.
* Since the Interop server recognizes each websocket as a new robot, refactor the bridge such that each robot starts a new socket connection.
