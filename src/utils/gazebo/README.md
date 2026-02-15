# SITL Gazebo Workspace

Canonical project location for Gazebo SITL runtime artifacts.

- Scripts:
  - `src/utils/gazebo/run_gz_takeoff_once.sh`
  - `src/utils/gazebo/apply_sitl_diff.py`
  - `src/utils/gazebo/mspapi2_takeoff_example.py`
- Diff file:
  - `src/utils/gazebo/gz_sitl_diff.txt`
- Runtime state:
  - `src/utils/gazebo/state/eeprom.bin`
  - `src/utils/gazebo/state/build/`
  - `src/utils/gazebo/state/logs/`

Primary launcher:

```bash
cmake/run_sitl_gazebo.sh
```

Build and launch:

```bash
unset LD_LIBRARY_PATH; 
cmake/docker_build_sitl.sh clean
./cmake/run_sitl_gazebo.sh --world forest --model x500_mono_camera --fcproxy --serialport=/dev/ttyACM0 --baudrate=115200
```



# Problems:
- pitch is reversed
- yaw is reversed
- ie: they're not reversed in RC or motor order, angle and acro mode fly OK, but poshold mode gives the wrong way commands
- every sign flipped in every combination possible, no effect
- changing imu signs just deathspins
- quaternion shit idk
- motor order: questionnable
- gazebo gui/ui: no idea how any of it works, it's awful, cant change views
- drone actual flight model (motors, props, weight): dont even get me started
- "world" is a joke, smaller than my backyard lost in a white void, deranged level design makes adding anything impossible
- have you people never played a fucking VIDEO GAME