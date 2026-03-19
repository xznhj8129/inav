# ProjectAirSim

ProjectAirSim SITL support lets INAV run against a prebuilt ProjectAirSim environment with the simulator integration living inside INAV SITL.

This backend is for **ProjectAirSim**, not legacy Microsoft AirSim.
It uses a **prebuilt** ProjectAirSim environment and does **not** require building ProjectAirSim or UE5 from source.

## Current status

The current backend supports quadcopter SITL bring-up with:
- native `--sim=pas` backend inside INAV SITL
- scene loading owned by INAV
- normal Configurator connection

## Requirements

Install the native SITL transport dependencies:

```bash
sudo apt install libnng-dev libmsgpack-dev
```

You also need:
- a prebuilt ProjectAirSim environment
- a working ProjectAirSim setup appropriate for your machine and scene
- your own INAV SITL configuration workflow

## Build INAV SITL

From the INAV repo:

```bash
unset LD_LIBRARY_PATH
cmake -S . -B build_SITL -DSITL=ON
cmake --build build_SITL --target SITL.elf -j"$(nproc)"
```

The SITL binary is:
- `build_SITL/bin/SITL.elf`

## What needs to be started

ProjectAirSim SITL requires two things to be running:
- a ProjectAirSim prebuilt environment
- INAV SITL started with `--sim=pas`

Typical order:
1. Start ProjectAirSim.
2. Start INAV SITL with `--sim=pas`.
3. Connect Configurator or another MSP client.

## Start ProjectAirSim

Launch your ProjectAirSim environment in the way appropriate for your installation.

Useful ProjectAirSim launch arguments include:
- `-windowed`
- `-fullscreen`
- `-RenderOffScreen`
- `-nullrhi`
- `-ResX=<width> -ResY=<height>`
- `-NoVSync`
- `-nosound`

Windowed example:

```bash
unset LD_LIBRARY_PATH
<your-projectairsim-launcher> -windowed -ResX=1280 -ResY=720 -NoVSync -nosound
```

Offscreen example:

```bash
unset LD_LIBRARY_PATH
<your-projectairsim-launcher> -RenderOffScreen -NoVSync -nosound
```

## Start INAV SITL

Start INAV SITL with the ProjectAirSim backend:

```bash
unset LD_LIBRARY_PATH
build_SITL/bin/SITL.elf --sim=pas --path <your-eeprom.bin>
```

The `--path` file is your SITL EEPROM/config state.
How you create it, persist it, or preconfigure it is part of your own workflow.

## Configurator

For normal manual use, connect INAV Configurator to:
- `SITL`

Or use:
- `TCP`
- host `127.0.0.1`
- port `5760`

Additional MSP clients can use the other TCP-backed UARTs as configured in SITL.

## Configuration

ProjectAirSim SITL does not define your INAV setup for you.
You are responsible for:
- choosing or creating the EEPROM/config file you want to use
- applying any CLI setup or diffs you need
- deciding how your baseline configuration is stored
- deciding whether you want to use scripts, Configurator, CLI, or MSP tooling

As with other SITL workflows, the simulated vehicle will only behave correctly if the INAV configuration matches the airframe and enabled sensors.

## Typical workflow summary

1. Install `libnng-dev` and `libmsgpack-dev`.
2. Build `build_SITL/bin/SITL.elf`.
3. Start your ProjectAirSim environment.
4. Start `build_SITL/bin/SITL.elf --sim=pas --path <your-eeprom.bin>`.
5. Connect Configurator on `SITL` or `TCP 127.0.0.1:5760`.
6. Configure, test, and fly using your normal SITL workflow.

## Notes

- The ProjectAirSim environment starts empty. INAV loads the scene at runtime.
- ProjectAirSim launch method, scene choice, EEPROM management, and auxiliary tooling are user-specific.
- Wrapper scripts can be useful, but they are not required by the backend itself.
