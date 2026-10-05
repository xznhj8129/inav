# Autopilot control

INAV can fly without RC. `control_mode = AUTOPILOT` replaces the RC link with a
telemetry-link heartbeat as the lifeline, restricts the aircraft to navigation
modes, and adds command surfaces so a GCS or companion computer can select modes
and, behind an explicit gate, drive attitude/rate/throttle setpoints.

This document covers the operating model, the command and setpoint surfaces,
the settings and the procedures.

See also: [Mavlink.md](Mavlink.md), [Settings.md](Settings.md),
`docs/development/msp/msp_messages.json` (MSP payload source of truth).

---

## 1. What Autopilot adds

1. **Pilot vs Autopilot control mode** — one setting, two operating regimes; in Autopilot the telemetry link replaces RC as the lifeline.
2. **Telemetry-link heartbeat failsafe** — an RC-independent liveness source with its own timeout, reusing `failsafe_procedure`.
3. **Unblocked mode selection** — channels and commands may select any flight mode in either control mode; a remote pilot or GCS may be on the controls.
4. **Command-based mode selection** — `MAV_CMD_DO_SET_MODE` (full reverse ArduPilot table) and `MSP2_INAV_SET_MODE`, with switch-like semantics.
5. **Offboard setpoints** — flight-axis angle/rate overrides plus a throttle member, gated by CONTROL OVERRIDE with a 200 ms dead-man timer.

---

## 2. Control modes

`control_mode` — `PILOT` (default) | `AUTOPILOT`, configuration-time, not inferred from `receiverType`.

```
                      PILOT                            AUTOPILOT
  lifeline .......... RC link                          telemetry link
  arming flag ....... RX (RC_LINK)                     TELEMLINK
  flight modes ...... all boxes                        all boxes (channels and commands)
  sticks ............ full RC authority                AER decay to centre in 200 ms;
                                                       throttle/aux remain sticky
  stick gestures .... gyro cal, profile, stick-arm     disabled
  prearm switch ..... active                           ignored
  arm-box disarm .... active (reversible motors)       unavailable
  direct control .... channels; MSP overrides gated    channels; offboard setpoints behind
                      by CONTROL OVERRIDE               CONTROL OVERRIDE
```

**Channel model in Autopilot.** Channels are real, not zeroed:

- initial values: AER = 1500 µs, throttle = `rx_min_usec`, aux = `rx_min_usec` (below every mode range);
- roll/pitch/yaw decay to centre when no valid channel update arrived within 200 ms (`PERIOD_RXDATA_FAILURE`); throttle and aux stay where they were last written;
- channels can be written by an RC stream (MSP RC / MAVLink RC on a serial-RX port) and aux channels CH13–CH32 over `MSP2_INAV_SET_AUX_RC`;
- NAV mode adjustments (POSHOLD nudges, ALTHOLD climb, FW course/altitude) work whenever channels are being written.

---

## 3. Lifelines, failsafe and arming

### 3.1 Telemetry liveness (Autopilot)

Two acceptable sources, no setting to pick one:

- **MSP:** any inbound message on a physical MSP port (USB/bench sessions excluded).
- **MAVLink:** a `HEARTBEAT` from a GCS or onboard controller identifies the peer (per system/component); after that, *any* traffic addressed to us from that peer refreshes liveness. Heartbeats from other vehicles never count.

```
 telemetry link (AUTOPILOT)                        RC link (PILOT)
       │                                                 │
  fresh│ stale > failsafe_telem_timeout (default 5 s)   │ stale > failsafe_delay
       ▼                                                 ▼
    FAILSAFE ──────────► configured failsafe_procedure: RTH | LAND | DROP | NONE
       │
       │ fresh data for failsafe_recovery_delay
       ▼
    normal operation
```

- Setpoint/command silence is *not* link loss: only the liveness clocks above trigger failsafe.
- Recovery uses the same hysteresis as the RC link (`failsafe_recovery_delay`).

### 3.2 Arming

```
 PILOT:      RX (RC_LINK) ──┬── other checks (ANGLE, CAL, GPS, NAV safety, ...) ──► ARMED
 AUTOPILOT:  TELEMLINK ─────┘   (RC_LINK is not evaluated)
```

- In Autopilot `TELEMLINK` is the only link interlock: it follows `failsafeIsReceivingControlLinkData()` and `RC_LINK` is explicitly cleared, even if an RC stream is present (RC is not the control link there).
- In Pilot `TELEMLINK` is cleared and `RC_LINK` follows the RC link as before.
- Position-dependent NAV mode preselected → `ARMING_DISABLED_NAVIGATION_UNSAFE` (see §4.3).

---

## 4. Mode model

### 4.1 Selectable modes and IDs

| Mode | box id | permanent id | MAVLink custom mode (FW / MC) |
|---|---|---|---|
| ANGLE | 1 | 1 | FW FBWA=5 / MC STABILIZE=0 |
| HORIZON | 2 | 2 | FW STABILIZE=2 / MC STABILIZE=0 |
| NAV ALTHOLD | 3 | 3 | FW FBWB=6 / MC ALT_HOLD=2 |
| NAV RTH | 8 | 10 | FW RTL=11 / MC RTL=6 |
| NAV POSHOLD | 9 | 11 | FW LOITER=12 / MC LOITER=5, POSHOLD=16, BRAKE=17 |
| MANUAL | 10 | 12 | FW MANUAL=0 / — |
| NAV LAUNCH | 14 | 36 | FW TAKEOFF=13 |
| NAV WP | 19 | 28 | FW AUTO=10 / MC AUTO=3 |
| GCS NAV | 22 | 31 | via FW/MC GUIDED (15/4) |
| NAV COURSE HOLD | 35 | 45 | — (GUIDED pairing) |
| NAV CRUISE | 44 | 53 | FW CRUISE=7 / — |
| ANGLE HOLD | 55 | 64 | — (reports as STABILIZE) |
| CONTROL OVERRIDE (offboard gate) | 41 | 50 | — |

`MSP2_INAV_SET_MODE` takes the **permanent id**; the CLI `aux` command also takes the
permanent id; MAVLink takes the ArduPilot custom mode.

### 4.2 Availability

There is no control-mode mode-blocking: channels and mode commands may select
any flight mode in either control mode, because a remote pilot or GCS may be on
the controls (RC stream, aux writes, offboard setpoints). What still restricts
selection:

- **target availability** — the box must exist in this build (e.g. NAV LAUNCH is absent when the FW launch feature is enabled, so `PLANE_MODE_TAKEOFF` is `DENIED`);
- **command whitelist** — commands accept flight modes, not auxiliary switches (beeper, cameras, user boxes, ...);
- **arming safety** (§4.3) still blocks arming while a position-dependent NAV mode is selected.

### 4.3 Arming safety

Position-dependent modes block arming while disarmed (`NAVIGATION_UNSAFE`):

- blocked: RTH, WP, POSHOLD, COURSEHOLD, CRUISE; on fixed wing additionally ALTHOLD and AUTOSPEED;
- exempt: GCS NAV, NAV LAUNCH (routinely active on the ground).

So the headless flow is **arm first, then select a NAV mode**; or select GCS NAV /
NAV LAUNCH pre-arm. A position-dependent mode left selected at disarm keeps
blocking the next arm until replaced (e.g. `MSP2_INAV_SET_MODE` GCS NAV, permanent 31).

---

## 5. Command-based mode selection

### 5.1 Surfaces

- **MAVLink:** `MAV_CMD_DO_SET_MODE` — `param1` must carry `MAV_MODE_FLAG_CUSTOM_MODE_ENABLED` (bit 0, value 1); `param2` is the ArduPilot custom mode. QGC's mode dropdown and Pause button both work.
- **MSP:** `MSP2_INAV_SET_MODE` (8738 / `0x2222`), payload `U8 permanent_id`.

### 5.2 Custom-mode tables

- **Fixed wing:** MANUAL 0, STABILIZE 2, FBWA 5, FBWB 6, CRUISE 7, AUTO 10, RTL 11, LOITER 12, TAKEOFF 13, GUIDED 15, AUTOLAND 26.
- **Multirotor:** STABILIZE 0, ALT_HOLD 2, AUTO 3, GUIDED 4, LOITER 5, RTL 6, LAND 9, POSHOLD 16, BRAKE 17.

Special decodes: `LAND`/`AUTOLAND` are not selectable modes — they run the normal
landing path (`activateForcedLanding`). `GUIDED` selects POSHOLD + GCS NAV.
Rovers and boats are not supported and receive `UNSUPPORTED`.
`TAKEOFF` maps to NAV LAUNCH, which only exists on fixed wing when the FW launch
feature is off; otherwise the box is unavailable and the request is `DENIED`.

### 5.3 Semantics

A mode command is a **virtual mode switch**:

```
 GCS command                                  MSP2_INAV_SET_MODE(0x2222, U8 id)
 MAV_CMD_DO_SET_MODE(param1 bit0, param2)     │
        │                                     │
        ▼                                     ▼
  plane/copter table                       box permanent id
        └───────────────┬─────────────────────┘
                        ▼
          navigationSelectModesByCommand()
            · selectable flight-mode box?
            · available on this target?
                        │
          fail ─────────┴────────► DENIED / UNSUPPORTED / MSP error
                        │ ok
                        ▼
              commanded mode mask ──────► effective box mask ──► nav FSM ──► flight mode
                        ▲
      channel flight-mode change / newer command ──► release, no command pending
```

- accepted while **disarmed** (nothing flies until armed);
- **retained across arm/disarm** like a channel-held switch;
- **released** when a channel-driven flight-mode selection changes, or when a newer command replaces it;
- **supersedes** an active one-shot `ACTIVATE_RTH` / `ACTIVATE_LANDING` override;
- forced failsafe procedures are untouched — the nav FSM keeps its priority and the command only moves the selection.

### 5.4 ACK behaviour

| Situation | MAVLink | MSP |
|---|---|---|
| mode selected | `ACCEPTED` | ACK |
| policy/state rejection (unavailable box, not selectable) | `DENIED` | error (`!`) |
| unknown mode id / no INAV equivalent | `UNSUPPORTED` | error (unknown id) |

Note: MAVLink mode *telemetry* (`HEARTBEAT.custom_mode`) reports the flying/nav
state, not an idle selection — a mode selected on the ground is visible through
MSP active modes / the FSM state once it flies, not through the heartbeat.

---

## 6. Offboard control

### 6.1 The gate

**CONTROL OVERRIDE** (`BOXMSPRCOVERRIDE`, permanent id 50) is the offboard gate:

- in **both** control modes the setpoint commands do nothing unless the box is active;
- in **Pilot** the RC link must also be healthy;
- in **Autopilot** it is selectable like any other box (channel-driven boxes are unfiltered);
- select it like any mode box: a mode range on an aux channel (`aux <slot> 50 <aux ch> <start> <end>`) driven by a switch, or by an RC stream, or by writing the channel over `MSP2_INAV_SET_AUX_RC` (CH13–CH32).

### 6.2 Setpoints

`MSP2_INAV_FLIGHT_AXIS_ANGLE_OVERRIDE` (8726 / `0x2216`) and
`MSP2_INAV_FLIGHT_AXIS_RATE_OVERRIDE` (8727 / `0x2217`):

```
 overrideMask : U8     bit 0 roll | bit 1 pitch | bit 2 yaw | bit 3 throttle
 roll         : I16    angle [deci-deg]  or rate [deg/s, ±2000]
 pitch        : I16    same
 yaw          : I16    same
 throttle     : I16    PWM us
```

- all five fields are part of the message; mask bit 3 enables the throttle override;
- send **one** of the two commands (angle XOR rate) per setpoint — mixing them is undefined;
- must be refreshed within **200 ms (5 Hz)**; expiry clears everything, including throttle;
- `throttle` is clamped to `idle..max` at the mixer and outranks the programming-framework throttle override.

```
 setpoints ≥5 Hz :  ANGLE_OVERRIDE(0x2216) XOR RATE_OVERRIDE(0x2217)
        │
        ▼  gate: BOXMSPRCOVERRIDE active  +  ≤200 ms fresh
 ┌─────────────────────────────────────────────────────────┐
 │ offboard layer                                          │
 │   angle/rate targets ──► PID (flight/pid.c)             │
 │   throttle ──► mixer input (flight/mixer.c)             │
 └─────────────────────────────────────────────────────────┘
        │ setpoints expire / gate drops        │ telemetry link dies
        ▼                                      ▼
 underlying mode (selected by command;        telemetry failsafe
 no default mode yet — falls to IDLE/rate)    (failsafe_procedure)
```

The OSD source text already shows `MSP` while the gate is active. There is no
dedicated "offboard active" element yet.

### 6.3 Caveats

- The gate is currently chosen over the aux surface; a mode range must exist
  (command-selecting `BOXMSPRCOVERRIDE` is a recorded option, not implemented).
- The gate and the setpoints have no bench readback — gating behaviour must be
  confirmed in flight or on a sim with an armed vehicle.

---

## 7. Settings reference

| Setting | Values / range | Default | Notes |
|---|---|---|---|
| `control_mode` | `PILOT`, `AUTOPILOT` | `PILOT` | selects regime (§2) |
| `failsafe_telem_timeout` | 0–200 deciseconds | 50 (5 s) | Autopilot telemetry-link loss guard; 0 = no guard time |

Related existing settings that Autopilot reuses:

- `failsafe_procedure` — RTH / LAND / DROP / NONE, used by both lifelines;
- `failsafe_recovery_delay` — recovery hysteresis for both lifelines;
- `rx_min_usec` — the value Autopilot initialises throttle/aux channels to.

CLI example: `set control_mode = AUTOPILOT` + `save`.

---

## 8. Procedures

### 8.1 Headless arm and mode change (Autopilot)

```
 1. set control_mode = AUTOPILOT ; save
 2. telemetry link up, GCS heartbeats flowing      → TELEMLINK clears
 3. arm:     MSP2_INAV_ARM_DISARM(0x2227, 01)      or MAV_CMD_COMPONENT_ARM_DISARM
 4. select:  MSP2_INAV_SET_MODE(0x2222, id)        or MAV_CMD_DO_SET_MODE(1, mode)
 5. change:  send another selection, or move a mode channel (release rule)
```

### 8.2 Offboard setpoints

```
 1. configure a mode range for the gate:  aux <slot> 50 <aux ch> 1700 2100
 2. select the gate:  MSP2_INAV_SET_AUX_RC  (write CH13..CH32)
    — Pilot additionally needs a healthy RC link
 3. stream setpoints ≥5 Hz: 0x2216 XOR 0x2217, mask bit 3 + throttle
 4. stop streaming  → setpoints expire, gate stays; the selected mode resumes
    link dies        → telemetry failsafe
```