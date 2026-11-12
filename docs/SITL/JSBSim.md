# JSBSim in SITL

SITL can fly a [JSBSim](https://github.com/JSBSim-Team/jsbsim) aircraft inside its own process. JSBSim is the open-source flight dynamics model behind FlightGear and many research projects. INAV's real code (sensors, AHRS, flight modes, PID, mixer, navigation, failsafe) flies the airframe, and its motor and servo outputs drive JSBSim's controls.

There is no other program to run: no X-Plane or RealFlight, no graphics, no network simulator link. A script drives the flight the way a pilot and a ground station would. It sends the sticks, switches modes, uploads missions, and checks where the vehicle really is against where INAV thinks it is. Runs are deterministic and usually much faster than real time, so they suit automated tests of firmware changes.

## What you need

- A JSBSim installation with its C++ library and `JSBSim.pc` (pkg-config). The Python package from pip does not include the C++ library; build JSBSim from source:

  ```bash
  git clone https://github.com/JSBSim-Team/jsbsim.git
  cd jsbsim
  cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DBUILD_PYTHON_MODULE=OFF -DBUILD_DOCS=OFF
  cmake --build build --target libJSBSim -j4
  sudo cmake --install build --component devel
  ```

  To install somewhere else, add `-DCMAKE_INSTALL_PREFIX=<dir>` and point `PKG_CONFIG_PATH` at `<dir>/lib/pkgconfig` when configuring INAV.
- JSBSim's aircraft data: a directory with `aircraft/`, `engine/` and `systems/`, such as the JSBSim source checkout or the directory of the `jsbsim` Python package.

## Building

Build SITL as usual. When CMake finds JSBSim it prints `SITL JSBSim backend enabled`. Without it, SITL builds exactly as before and `--sim=jsbsim` reports that JSBSim is missing.

## Starting

```bash
inav_SITL --sim=jsbsim --simcfg=quad.xml --useimu --path=quad.bin
```

- `--simcfg` names the vehicle file (below).
- `--useimu` lets INAV's own AHRS estimate the attitude from the simulated gyro, accelerometer and compass. Without it SITL hands INAV the true attitude, and the AHRS is not tested.
- SITL boots on wall time with the vehicle at its start condition. Physics stands still until the first step frame arrives, then runs only on the stepped clock.

Configure INAV for the vehicle as you would a real one. The example profiles set the simulated sensors: `receiver_type = SIM (SITL)`, `acc_hardware`, `baro_hardware` and `mag_hardware = FAKE`, `gps_provider = FAKE`, and `align_mag = CW0` (with `DEFAULT`, INAV turns the compass `CW270FLIP`, the usual external mounting). Add `rangefinder_hardware = FAKE` or `pitot_hardware = FAKE` to use those sensors.

## Vehicle file

`src/utils/jsbsim/` has two examples with matching INAV profiles: `quad.xml` (JSBSim's DJI F450, on the ground) and `plane.xml` (JSBSim's Cessna 172, a full-size aircraft, in level flight at 450 m).

```xml
<inav_vehicle>
  <model root="jsbsim" dt="0.001">F450</model>
  <initialize>quad_ground.xml</initialize>
  <property name="atmosphere/wind-north-fps">10</property>
  <motor index="0">fcs/cmdAR_ext_nd</motor>
  <servo index="2" invert="true">fcs/steer-cmd-norm</servo>
  <gps rate="10"/>
  <rangefinder max="10"/>
  <known_error>INCOMPATIBLE CFG FILE</known_error>
</inav_vehicle>
```

| Element | Meaning |
| --- | --- |
| `model` | The JSBSim aircraft. `root` is the JSBSim data directory, relative to the vehicle file or absolute. `dt` is the physics step in seconds. |
| `initialize` | A standard JSBSim initialization file: position, altitude, attitude, speed, running engines (`<running>`), trim (`<trim>ground</trim>` for a start on the gear). Give latitudes as `type="geodetic"`, as GPS reports them. |
| `property` | Any JSBSim property set before the start, for example wind or mixture. |
| `motor`, `servo` | One INAV output driving one JSBSim property. `index` is INAV's motor or servo number from 0 (motor 1 = 0). Motors go out as 0..1, servos as -1..1. `invert="true"` negates it; one output may drive several properties. |
| `gps` | GPS update rate in Hz. Required. |
| `rangefinder` | Optional. Range in m of a rangefinder looking along the body's down axis at flat ground. |
| `known_error` | Optional. JSBSim error messages containing this text are not printed. |
| `hold` | Optional. Holds an airborne start where it is until released: physics does not run and the accelerometer reads gravity at the start attitude, so you can arm, pick modes and set the throttle first. `release` and `delay` as for `catapult`. |
| `catapult` | Optional. The vehicle sits on a catapult at its start condition (give the rail's pitch as `theta`, its height as `altitudeAGL` and `vc` 0 in the initialization file), attitude locked, until released; then a perfect rail along the nose pushes it at constant acceleration to `speed` (m/s) over `length` (m), and it flies free from the rail's end. |

`release` says what lets a `hold` or `catapult` go, `delay` (s, default 0) how long after that holds:

- `release="launch_ready"`: INAV's NAV LAUNCH shows READY TO LAUNCH, i.e. it is armed, the throttle is up, any idle motor has spun up and it is waiting for the throw. The catapult then gives INAV the acceleration it detects as a launch (`nav_fw_launch_accel`, about 1.9 g by default: 15 m/s over 2 m is about 5.7 g). Never fires without NAV LAUNCH.
- `release="arm"`: INAV is armed. For launches without NAV LAUNCH (ACRO, MANUAL), a held airborne start, or a held multirotor.

SITL prints when it releases, and the speed at the catapult's end.

If the vehicle goes into the ground (below its gear), SITL prints the time and height and exits with an error.

## Driving a flight

Each tick the script sends one `MSP_SIMULATOR` frame (format 3) with the flags `HITL_SITL_MODE` and `HITL_STEPPED_CLOCK`:

| Field | Type | |
| --- | --- | --- |
| version | `uint8` | 3 |
| flags | `uint16` | `HITL_SITL_MODE` (bit 14) + `HITL_STEPPED_CLOCK` (bit 15); `HITL_SIM_RC_INPUT` (bit 11) if RC follows; `HITL_FAILSAFE_TRIGGERED` (bit 13) for receiver failsafe |
| RC | 8 × `uint16` | AETR + AUX1..4 in µs, only with `HITL_SIM_RC_INPUT` |
| RSSI | `uint16` | only with `HITL_SIM_RC_INPUT` |
| step | `uint32` | µs of simulated time to run |

SITL runs the physics and the flight controller through the step, then replies. The reply is the usual `MSP_SIMULATOR` reply (INAV's stabilised commands, debug value, its attitude estimate, OSD data), followed by 34 bytes of ground truth at the end of the step: sim time, latitude, longitude, altitude MSL, height above ground, roll, pitch, heading, velocity north/east/down, airspeed. The fields are in `docs/development/msp/README.md` (`truth*`).

Everything else (arming, modes through the AUX channels, settings, missions, telemetry) uses normal MSP or MAVLink between steps. A multirotor needs steps of about 2.5 ms (400 Hz); a fixed-wing flies well with 10 ms.

Wind and every other start condition are fixed for the run; GPS always has a fix.
