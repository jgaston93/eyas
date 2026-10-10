# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project status

Eyas is a real-time, piloted rotorcraft flight dynamics model (C++20 library), intended to replace an incumbent commercial flight-dynamics package ("the reference model") in a host simulator. Reference vehicle: UH-60A (GenHel, NASA CR-166309).

The design spec is `docs/EYAS_DESIGN.md`. Review findings are in `docs/SPEC_REVIEW.md`, approved departures in `docs/DECISIONS.md`, and data issues in `docs/DATA_GAPS.md`. Implementation status: M0 (scaffolding) and M1 (6-DOF rigid body core) done (see `docs/VALIDATION.md`); next is M2.

## How to treat the spec

`docs/EYAS_DESIGN.md` is a **v0.1 draft and the starting point, not gospel**.
- Before implementing any equation, sign convention, numeric threshold or acceptance criterion, verify it against the cited reference (Padfield, Leishman, Johnson, GenHel, the inflow and stall papers).
- When the spec looks wrong, inconsistent or underspecified:
  - say so;
  - add an entry to `docs/SPEC_REVIEW.md`;
  - propose a fix.

  Don't silently implement a flawed spec, and don't silently deviate from it either.
- Resolved departures go in `docs/DECISIONS.md`.
- Check `docs/SPEC_REVIEW.md` for known open problems before working on an affected area.

## Reference library

The reference documents live outside this repo, in a local reference directory. **`CLAUDE.local.md` (git-ignored) gives the location and file names**; read it first if it exists, and if it doesn't, ask the user. The documents are copyrighted: never copy them, or large excerpts from them, into this repo. Cite the editions below (DECISIONS D-001).

| Reference | Notes |
|---|---|
| Padfield, *Helicopter Flight Dynamics*, **3rd ed.** (2018) | Has a text layer. Ch. 3 covers the simulation model, Ch. 4 trim and stability, Ch. 10 tiltrotors. |
| Johnson, *Rotorcraft Aeromechanics* (2013) | Has a text layer. The Mathematics of Rotating Systems chapter (pp. 545–581) covers multiblade coordinates; §11.3 covers dynamic inflow. |
| Leishman, *Principles of Helicopter Aerodynamics*, **2nd ed.** (2006) | Scanned, with an imperfect OCR layer. Dynamic inflow is in §10.8.3–10.9; the hover blade-element closed forms are in §3.2. |
| Pitt & Peters, 6th ERF Paper 47 (1980), the same paper as *Vertica* 5(1) 1981 | The primary Pitt–Peters source. Weak OCR, so read equations from images. Final L and M matrices are in Table 4, p. 47-13. Open 4× apparent-mass conflict with Johnson (D-001). |
| Pitt & Peters, 9th ERF Paper 55 (1983) | Supporting only; it covers a different, 5-state model. |
| Howlett, GenHel UH-60A, NASA CR-166309 (1981) | Public NASA report, scanned with a poor OCR layer. **Read numeric data from rendered page images, never from extracted text.** An OCR misread is invented data. |

**Reading scans:** poppler isn't installed, so the Read tool can't render PDFs. Use PyMuPDF (`python3 -I`, `import fitz`) to extract text or render a page with `page.get_pixmap(dpi=...)` (use `prerotate(90)` for rotated pages), then Read the PNG. For DjVu: search with `djvutxt --page=N`, render with `ddjvu -format=tiff -page=N`, convert to PNG with PyMuPDF.

**Human review of uncertain readings.** These are scans, so don't guess. Whenever text, a figure, an equation, a sign or a number is ambiguous, ask the user to check it. Typical causes: OCR disagrees with the image, a digit or subscript is blurry, a table cell is unclear, or the source contradicts another reference.
- Give the file, the book page and PDF page, what you read, and what is in doubt.
- Point to the rendered image you looked at.
- Don't use the value in code or config until the user confirms it.
- If it can't be resolved right away, log it in `docs/DATA_GAPS.md` marked `UNVERIFIED — needs human review`.
- Answers from AI assistants (including you) are not sources. Verify any definition or value against the primary document before using it.


## Hard constraints (not up for debate)

- **Clean-room rule for the reference model (§2.1).** Use only the public references in §2.
  - Never read, copy, paraphrase or port the reference model's source, docs, manuals, model or config files, or parameter values, and never ask for them.
  - Reference-model *outputs* are a comparison target only. They live in an external, access-controlled data directory, never in this repo.
  - Tests that need reference-model outputs must skip, not fail, when that directory is absent.
  - If a task seems to need the reference model's internals, stop and ask.
- **This repo is public.** Never name the reference model's vendor or product, people, the organization, or specific future aircraft programs in tracked files, commits or PR text. Use the generic terms ("the reference model", "a tiltrotor"). Private specifics go in `CLAUDE.local.md` only.
- Controlled or proprietary vehicle data stays out of the repo. `data/uh60a/` holds public data only.
- Never invent aircraft data. Mark a missing config value with the `!placeholder` YAML tag (a whole table with `# status: placeholder`), log it in `docs/DATA_GAPS.md`, and continue (D-012). Every table needs a `# source:` line; every config quantity needs a unit; write `lbf`/`lbm`, never `lb`.
- Cite the equation source in a code comment, using the D-001 editions, e.g. `// Padfield 3rd ed. Eq. 3.xx` or `// Leishman 2nd ed. Eq. 3.25`.
- The §6 interfaces (`DynamicComponent`, `ForceModel`, `ComponentDerivs`, `Wrench`, `SimContext`, library API, `AircraftState`) are the contract between modules. Propose a change in `DECISIONS.md` and ask before making it. Several are already known to need changes (see SPEC_REVIEW).

## Process

- Work one milestone at a time (§9). Don't start M(N+1) until M(N)'s acceptance tests pass. The spec's acceptance criteria were written loosely, so any of them may be replaced by a better test anchored in the literature, with the replacement recorded in DECISIONS (D-007 is the model). Never bend a test to make it pass. Prefer layered verification:
  - an independent exact-equation reference that exercises the unmodified production code path (no test-only switches in hot loops);
  - a closed-form or textbook anchor with an explained tolerance;
  - experimental or published data as a validation entry.
- Write tests first: turn acceptance criteria into GoogleTest cases.
- Each milestone ends with an entry in `docs/VALIDATION.md`, with plots.

## Build & test

```bash
cmake --preset debug                 # configure; also: release, relwithdebinfo, asan (ASan+UBSan)
cmake --build --preset debug
ctest --preset debug                 # all tests
ctest --preset debug -R 'Axis\.'     # tests matching a regex (each GoogleTest case is a ctest test)
./build/debug/tests/eyas_unit_tests --gtest_filter='Config.*'   # run the binary directly
clang-format --dry-run --Werror src/**/*.{cpp,hpp} tests/**/*.{cpp,hpp}
clang-tidy -p build/debug src/util/*.cpp src/sim/*.cpp
```

- **Dependencies:** fetched by FetchContent at configure time, pinned by version and SHA-256 in `cmake/EyasDependencies.cmake` (Eigen 3.4.0, yaml-cpp 0.8.0, GoogleTest 1.15.2). Offline machines: `-DFETCHCONTENT_SOURCE_DIR_<NAME>=<path>`.
- **Flags (D-010):** set in `cmake/EyasCompilerFlags.cmake` through `eyas_target_defaults(target)`, which **every new target must call**. The flags are strict warnings plus `-Werror`, `-ffp-contract=off` and no fast-math (configure fails if fast-math flags appear). Third-party includes are SYSTEM.
- **Before calling a change done:** build and test with both GCC and Clang. Clang catches warnings GCC misses (e.g. `NAN` is a float under `-Wdouble-promotion`), so use `std::numeric_limits<double>::quiet_NaN()`.
- **Target platform:** a Linux server running RHEL 8 or RHEL 9 (D-005). Verified in Rocky Linux containers: RHEL 9's system GCC 11 works; RHEL 8 needs `gcc-toolset-13` (configure rejects GCC < 11 with instructions). CMake there is 3.26–3.31. Pin the compiler per build, since determinism is per build and compiler.
- **CI:** `.github/workflows/ci.yml` (Ubuntu GCC/Clang/ASan, Rocky 8 and 9 containers, format and tidy). To reproduce a Rocky job locally: `docker run --rm -v "$PWD":/src:ro rockylinux:9 ...`, configuring out-of-tree with `cmake -S /src -B /tmp/build`.
- **Real-time test hook:** `tests/support/alloc_counter.hpp` (`AllocationCounter`) counts heap allocations in a scope through replaced global `operator new`. Use it to assert zero allocations for anything reachable from `advance()`.
- **Layout:** `src/util/` holds init-time helpers (units, axis and tables, CSV and YAML loaders); `src/core/` the shared types, the §6 component interfaces, the rigid body and RK4; `src/sim/` the executive and recorder. `include/eyas/` will hold the public API (from M2, with `AircraftState`); `bench/` the timing benchmark; `tools/` offline Python (`tools/requirements.txt`, run from a git-ignored `.venv`); `tests/validation/` generators for VALIDATION.md data.

## Architecture (intended design per spec §5–6)

- **One global state vector.** Every dynamic component declares its own states and computes their derivatives: the 6-DOF body, per-blade flap, lag and elastic states, inflow, and later rotor speed and engine. The integrator (RK4 by default, swappable) advances all of them together. The goal is that adding fidelity means adding states to a component, not restructuring.
- **Rotor–body coupling solved within each derivative evaluation.** Components return derivatives and loads affine in the body acceleration a = [v̇_b; ω̇_b]: `ẋ = ẋ0 + G·a`, `load = load0 + H·a`. The executive solves one 6×6 system `(M_rb − ΣH) a = Σload0 + Σforces + gravity − gyro` and back-substitutes. `M_rb` is the airframe without the modeled blades, loads are about the airframe CG, and each component includes gravity on its own mass (D-002).
- **The derivative function is meant to be a pure function of (t, x, inputs).** Trim and linearization Jacobians and `snapshot()`/`restore()` rely on this. The only hidden state allowed is a cached table-search index.
- **Executive (§6.10).** `advance(controls, env, dt_host_ns)` (integer nanoseconds, D-006) runs fixed `step()` sub-steps (default 1000 Hz).
  - Time is kept in integer nanoseconds: `t = n_steps × dt`, with the remainder carried between calls and the number of sub-steps per call capped.
  - Eyas is single-threaded and synchronous, with no threads, IPC or transport inside it. Host adapters live in the host codebase.
- **API:** `init(config)`, `trim(condition)`, `advance(...)`, `outputs() → AircraftState`, `reset`, `snapshot`, `restore`. `AircraftState` grows in tiers, and existing fields never change meaning, units or frame.
- **Data-driven:** aircraft are defined by YAML and CSV loaded at `init()`. Imperial data is converted to SI at load time.
- Known open architectural problems are tracked in `docs/SPEC_REVIEW.md`:
  - linearization of the periodic (LTP) rotor system;
  - no data path between components;
  - the stateless gear and stabilator models;
  - a timing budget defined against the wrong call.

## Real-time rules (§7), applying to everything reachable from `advance()`

- No heap allocation, enforced by a counting `operator new` test.
- No exceptions and no I/O.
- Bounded iterations, each with a convergence flag.
- Bit-identical determinism.
- Stay within the timing budget (see SPEC_REVIEW #5 on how it should be defined).
- Table lookups use hunt search with a cached index.
- Use `double` everywhere, fixed-size Eigen types and `std::span` in hot paths, and no raw `new`/`delete`.

## Conventions (§4)

- Padfield conventions. Frames: `n` NED inertial, `b` body FRD at CG, `h` hub non-rotating, `r` rotating hub, `bl` blade.
- Every vector carries its frame suffix (`v_b`, `F_h`, `omega_b`). A rotation matrix `C_xy` maps frame y into frame x.
- Attitude is a unit quaternion `q_nb`, renormalized every step. Euler angles are output only.
- `ψ` always means rotor azimuth (ψ = 0 points aft). Euler heading is `heading` (ψ_h). Rotor direction is a config flag (UH-60: CCW from above).
- Every public function documents its units and frames.
