# Eyas examples

The examples use Eyas the way a host does: only the public headers in
`include/eyas/`, linked through `eyas::eyas` (D-020). They are built by
default (`EYAS_BUILD_EXAMPLES`) and smoke-tested by `ctest`.

## `drive/`: scripted test driver

Flies an aircraft from a control history and writes every `AircraftState`
field at each host frame:

```bash
./build/release/examples/eyas_drive --config data/uh60a/uh60a.yaml \
    --controls examples/drive/doublet_10s.csv --trim-speed 60 --out doublet.csv
```

It trims in level flight at `--trim-speed` kt (default hover) and
`--altitude` m, then flies at `--rate` Hz (default 60) with integer-nanosecond
frames. The control file holds increments from the trimmed controls, held
from each row's time until the next; see the comment at the top of
`drive/eyas_drive.cpp` for the format and exit codes. `--origin` and
`--ground` set the geodetic origin (D-044) and a level ground, so a pilot
station recording replays exactly.

## `adapter/`: reference adapter

The host application codes against its own flight-model interface (spec
§6.10, "Adapter pattern"); an adapter per flight model translates. Here
`iflight_model.hpp` is a toy host interface in host units (feet, knots,
degrees, °C, pedal positive left), and `EyasAdapter` implements it on
`eyas::Model`:

- `initialize()` loads the config and trims (init-time; Eyas throws
  `eyas::InitError`, which the adapter turns into an error string);
- `trim_inputs()` gives the host the trimmed controls, to start its cockpit
  controls there;
- `update()` maps the inputs and calls `advance()` once per host frame,
  synchronously on the host's thread; it allocates nothing and never throws;
- `pose()`, `air_data()` and `rotor()` convert units and sign conventions;
- `healthy()` turns `AdvanceStatus` flags (non-finite step, capped or
  rejected frame) into one flag for the host.

`main.cpp` is host code: it names the flight model in one place
(`make_flight_model()`), so swapping adapters changes that line only.

```bash
./build/release/examples/eyas_adapter_example data/uh60a/uh60a.yaml
```

Host-side guidance from the decisions log: pass integer frame times and
prefer a nominal frame period for reproducible runs (D-006); keep the
simulation thread's priority and CPU placement on the host side, since Eyas
creates no threads and its timing budget assumes the thread isn't preempted
(D-005, D-042).

## `pilot_station/`: desktop pilot station (M8b, D-044)

Flies an aircraft by hand at a fixed host rate, paced to the wall clock,
in its own window (GLFW and OpenGL 2.1; built when both are found, otherwise
`--headless` only) and optionally in FlightGear.

```bash
./build/release/examples/eyas_pilot_station --config data/uh60a/uh60a.yaml \
    --input mouse --start hover --record flight.csv
```

- **Inputs** (`--input`): `gamepad` (GLFW gamepad; right stick cyclic, left
  stick Y a rate collective, left stick X pedals, A trim, B brakes, Y trim
  reset), `joystick` (GLFW raw axes: roll, pitch, twist, throttle), `mouse`
  (hold the left button to fly the cyclic; W/S collective, Z/X pedals, T or
  the right button trim, B brakes, C trim reset), `fg` (FlightGear's
  joystick, below), `script` (an `eyas_drive` control file, `--controls`) or
  `none`. `--print-map` writes the mapping in use; edit it and pass it back
  with `--map` to change axes, signs, deadband, expo, absolute or rate mode
  and gains.
- **Trim:** pressing and releasing the trim button keeps the cyclic and
  pedals where they are while the stick springs back (force-trim release).
  The HUD shows the stick (dot) and the trim point (cross).
- **Keys:** Esc quit, P pause, R restart, 1–4 start on the pad, in a hover,
  at 60 kt or at 120 kt (100 ft above the ground; `--hover-height`), V view
  (chase, eye at the CG, ground observer), H help.
- **Recording:** `--record` writes an `eyas_drive` control file, timed from
  the start point, whose header gives the exact `eyas_drive` command that
  replays the flight from an airborne start (bit for bit). `eyas_drive` has
  no brake input, so if the brakes change the file says from when it no
  longer replays. `--out` writes the state history.
- The bare UH-60A has no AFCS yet (M8c), so it is unstable and hard to hover.

### FlightGear as the out-the-window view

Tested with FlightGear 2024.1 on Windows, Eyas in WSL2. FlightGear also reads
the stick on the Windows side and forwards it, so a USB device needs no
passthrough into WSL2.

1. Copy `examples/pilot_station/flightgear/eyas_fdm.xml` and `eyas_ctrl.xml`
   into FlightGear's `$FG_ROOT/Protocol/` (with the Windows launcher this is
   `fgdata_<version>/Protocol` in its download directory). Copy
   `eyas_joystick.xml` anywhere on the Windows side: FlightGear doesn't
   publish raw stick values, and this file, loaded with `--config`, binds
   joystick 0's axes and buttons to the properties `eyas_ctrl` sends
   (replacing FlightGear's own bindings for that device for the session).
2. Find the addresses. In WSL2's default NAT networking, Windows is WSL's
   default gateway (`ip route show default`) and WSL is its `eth0` address
   (`hostname -I`). With `networkingMode=mirrored` in `.wslconfig`, both are
   `127.0.0.1`.
3. Start FlightGear (Windows, one line), at the origin you will give Eyas,
   for example near San Francisco International:

   ```
   fgfs --fdm=null --aircraft=<any helicopter model> --lat=37.6188 --lon=-122.3756
        --config=<path to>\eyas_joystick.xml
        --generic=socket,in,60,,5501,udp,eyas_fdm
        --generic=socket,out,60,<WSL address>,5502,udp,eyas_ctrl
   ```

   Allow `fgfs` through the Windows firewall for UDP when asked.
4. Start the station with the same origin (latitude, longitude in degrees,
   ground altitude in metres MSL):

   ```bash
   ./build/release/examples/eyas_pilot_station --config data/uh60a/uh60a.yaml \
       --origin 37.6188,-122.3756,4 --fg <Windows address>:5501 \
       --input fg --listen 5502
   ```

The HUD's top line counts the control lines received (`FG out in N`). The
`fg` map assumes an Xbox pad as FlightGear numbers it on Windows (axes 0 and 1
the left stick, 3 and 4 the right stick's Y and X). FlightGear's built-in web
server (`--httpd=5480`) shows the raw values at
`/json/devices/status/joysticks/joystick` as you move the sticks; correct the
map with `--map` if the numbers differ. If FlightGear lists more than one
joystick, the stick must be its joystick 0. Eyas's ground is flat at the origin's altitude, so where
FlightGear's terrain isn't flat the aircraft can look slightly sunk or
afloat.
