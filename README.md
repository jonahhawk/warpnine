# WarpNine — CAM + CNC control in Rust

One desktop app that turns 2D/3D geometry into toolpaths, simulates them, posts G-code,
and drives the machine. Windows first; everything except the UC100 driver is cross-platform.

## Building

One-time setup: install Rust from https://rustup.rs (choose the default MSVC toolchain; the
installer offers the Visual Studio C++ build tools if they are missing).

For Rhino import, also install the Visual Studio C++ CMake tools and fetch the pinned SDK once:

```powershell
.\scripts\fetch-opennurbs.ps1
```

For STEP import, also build the pinned static Open CASCADE toolkits once (20-40 minutes, about 530 MB of libraries under `vendor/occt-install`, git ignored):

```powershell
.\scripts\fetch-occt.ps1
```

After every change:

1. **Close WarpNine** if it is running (Windows locks the running `.exe`, so the build fails).
2. In a terminal:
   ```
   cd D:\dev\warpnine
   cargo build --release
   ```
   or double-click **`build.bat`** (it checks that WarpNine is closed first).
   `build.bat run` builds and then launches the app.
3. Run `D:\dev\warpnine\target\release\warpnine.exe`.

The first build takes a few minutes; later builds take about 1–2 minutes and only recompile what changed.
`cargo run --release` builds and launches in one step. `cargo test --workspace` runs all tests.

## Workspace

| Crate | Purpose |
|---|---|
| `warp-import-3dm` | Pinned native openNURBS reader, stable Rhino identity and diagnostics |
| `warp-core` | Geometry (polylines, bounding boxes, arc tessellation, curve chaining), DXF and STL import |
| `warp-tools` | Tool library and holders (13 tool shapes, metric and imperial divisions with starter sets, JSON persistence, profile function per shape) |
| `warp-gcode` | G-code parser + RS274 interpreter (G0–G3 IJK/R, G4, G10 L2/L20, G17–19, G20/21, G28/30, G43/49, G53–59, G73/G80–G89, G90/91, G90.1/91.1, G92, G98/99, M0–M9/M30) |
| `warp-cam` | Operations → toolpaths, polygon offsetting, exact mesh contact and Z-map drop cutter, post-processors |
| `warp-sim` | Height-field material-removal simulation with rapid-collision detection |
| `warp-machine` | Motion device trait, real-time simulator device, program streamer, jog/MDI/WCS, UC100 driver |
| `warp-app` | egui GUI with four tabs: CAM, Simulate, Tool library, Machine |

M1 adds persistent geometry selections, setup coordinates, a geometry/layer browser, undo/redo and bounded background generation. See [M1 testing instructions](docs/baselines/M1_WORKFLOW.md).

M2 adds native Rhino `.3dm` import for curves, points, meshes and nested blocks, with source reimport. See [M2 testing and limits](docs/baselines/M2_RHINO_IMPORT.md) and the [solid meshing decision](docs/decisions/3dm-meshing.md).

M3 adds curve-driven 2.5D operation types, validated regions and links, mesh-fixture envelope checks, and checked posting. See [M3 test job, evidence and supported limits](docs/baselines/M3_25D_CAM.md).

## What works now

**Operations, with every parameter editable**
- 2D: Facing, Profile (inside/outside/on, finish passes, spring pass, arc lead-in/out, tabs),
  Pocket (offset or zigzag, islands, finish pass), Drill (G81/82/83/73/85/89, tip compensation,
  breakthrough), Engrave/follow, V-carve (medial axis: the bit stays tangent to both walls and rises into narrow
  gaps and corners), Helical bore, Thread mill (internal/external, RH/LH), Chamfer.
- 3D (any tool shape via its profile): Z-level roughing, Parallel, Waterline, Spiral, Radial, Project curves.
- Common: heights (clearance/retract/feed/top/bottom), feeds and speeds, coolant, stepdown, wall/floor
  stock, climb/conventional, plunge or ramp entry, tolerance, surface sampling step and query mode, curve selection
  (click curves in the 3D view).

**Posts:** Mach3/Mach4, UCCNC, LinuxCNC, GRBL, TurboCNC. Each has editable settings: units, decimals,
line numbers, arcs vs. linearised, canned cycles, tool change, G43, preamble, and program end.

**Simulation:** animated stock removal, tool glyph, removed volume, run-time estimate, and warnings
when a rapid move cuts material. You can also simulate any external G-code file.

**Machine:** DRO (work and machine coordinates), G54–G59, zero buttons, continuous/step/keyboard jog,
MDI, cycle start / feed hold / stop / run-from-line, E-stop, feed override, soft limits, tool-change
pauses, M0/M1, dwell, and spindle/coolant output pins. All of this is tested against the simulator device.

## UC100 + Gecko G540

WarpNine drives the UC100 through CNC Drive's `UC100.dll` (`C:\UCCNC\API\DLL`, installed with UCCNC;
no licence needed). The bindings in `crates/warp-machine/src/uc100.rs` are transcribed from the
vendor's `API\Interface\UC100.h` and checked against `UC100_wrapper.cs`. A test runs them against
the DLL's built-in demo controller: struct layouts round-trip, moves land on target with the right
line ID, and jog directions are correct.

The DLL also needs FTDI's 64-bit `ftd2xx.dll`. WarpNine loads it from System32 (installed by the FTDI
USB driver) or from `C:\UCCNC`.

**Machine → Machine setup:**
1. Choose **UC100 (USB)**, then **Probe DLL** to check it loads.
2. Tick **Demo mode** to try everything without the board.
3. Click **Connect**. The board starts in reset, so press **RESET E-STOP** before jogging.

Close UCCNC or Mach3 first, since only one program can use the board.

The G540 preset puts step/dir on DB25 pins 2–9 (X, Y, Z, A). Steps/mm assume 10 microsteps and
5 mm-pitch screws (400 steps/mm); set yours. E-stop, spindle and charge-pump pins depend on your
wiring, so take them from the G540 manual.

**First run on the real machine:** keep a hand on the physical E-stop. Check jog directions and
distances with small moves. Then check output pins (spindle/coolant relays) with a meter before
trusting them.

## Roadmap (gaps vs. RhinoCAM / Mach4 / LinuxCNC)

See the [RhinoCAM-style CAM implementation plan](docs/RHINOCAM_FEATURE_PLAN.md) for phased
2.5D/3-axis strategy coverage, Rhino/openNURBS import, geometry selection, sheet nesting,
high-speed machining, and release acceptance gates.

The [CAM build plan](docs/CAM_BUILD_PLAN.md) turns that scope into ordered implementation
tasks, code ownership, dependencies, test fixtures, and release gates.

User-facing features and changes by date are logged in the [changelog](docs/CHANGELOG.md);
the current position and open work are in [SESSION_STATUS.md](docs/SESSION_STATUS.md).

M0 is complete: [baseline results and known limitations](docs/baselines/M0_BASELINE.md).
Run `cargo test -p warp-cam --test baseline` for the synthetic fixtures, or
`./scripts/run-cam-baseline.ps1` to record generation/simulation metrics and environment hashes.

- **CAM:** adaptive/trochoidal clearing, rest machining, pencil and scallop finishing,
  4th-axis wrapping/indexing, holder/shank collision checking, cutter-comp output
  (G41/42), toolpath editing and reordering of individual cuts, STEP/IGES import.
- **Simulation:** GPU rendering (wgpu), dexel/voxel stock for undercuts, gouge check vs. the model,
  comparison against the design.
- **Control:** homing and limit-switch sequences, probing (G31/G38.x), tool
  length setter, VFD/PWM spindle control, macros and M-code scripting, backlash compensation,
  pendant/MPG input, a trajectory planner for boards that need host-side planning.
- **App:** undo/redo, inch display units, layers, saved post-processor profiles, macOS/Linux packaging.
