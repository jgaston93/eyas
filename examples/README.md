# Eyas examples

Both examples use Eyas the way a host does: only the public headers in
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
`drive/eyas_drive.cpp` for the format and exit codes.

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
