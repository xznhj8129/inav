# SITL Gazebo Workspace

Canonical project location for Gazebo SITL runtime artifacts.

- Scripts:
  - `src/utils/gazebo/run_gz_testflight.sh`
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
unset LD_LIBRARY_PATH; cmake/docker_build_sitl.sh
cmake/run_sitl_gazebo.sh
```

./cmake/run_sitl_gazebo.sh --world forest --model x500_mono_camera --fcproxy --serialport=/dev/ttyACM0 --baudrate=115200