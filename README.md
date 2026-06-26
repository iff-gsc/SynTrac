# SynTrac — Flight Dynamics Model

Physics-based flight dynamics simulation for Project A05 (TRR 364 CRC), developed at TU Braunschweig, Institut für Flugführung.

The model covers three reference configurations of the CRC:
- **Config 1a** — Two highly integrated fans at the rear fuselage with thrust vectoring
- **Config 1b** — Pylon-mounted rear propulsors with thrust vectoring
- **Config 2a** — 10 distributed pull propellers along the main wing *(primary focus)*

The implementation uses MATLAB/Simulink and builds on the open-source [LADAC](https://github.com/iff-gsc/LADAC) framework (Library for Aircraft Dynamics and Control).

---

## Prerequisites

- MATLAB R2023b or later with Simulink
- LADAC submodule (included under `modules/LADAC/`, see [Submodules](#submodules))
- trimmod submodule (included under `trimmod/`, provides `jj_trim` and `jj_lin`)

### Submodules

After cloning the repo, initialise the submodules:

```bash
git submodule update --init --recursive
```

---

## Repository Structure

```
syntrac/
├── configuration_2a/        # Config 2a entry point, params, and data
│   ├── init_config2a.m      # Main entry point — run this first
│   ├── trimConfig2a.m       # Trim and linearize at a flight condition
│   ├── params/              # Aircraft geometry, airfoil, and actuator parameters
│   ├── data/
│   │   ├── xprop_data.txt   # Raw XROTOR propeller data (source of truth)
│   │   ├── xprop_data.mat   # Preprocessed lookup tables (generated, do not edit)
│   │   └── aero_validation/ # CFD reference distributions (DLR TAU)
│   ├── xprop_build_matfile.m  # Step 1 of propeller data pipeline
│   ├── xprop_load_table.m     # Step 2 of propeller data pipeline
│   ├── plot_aircraft_geometry.m  # Plot wing, HTP, and VTP lifting-line geometry in one figure
│   └── prop_wing_validation.m   # Compare spanwise lift and drag distributions (LLT vs CFD reference data)
├── functions/               # Standalone MATLAB functions
│   ├── propeller_aero.m     # BEM propeller force/moment evaluation
│   ├── propeller_model_sfcn.m # Simulink S-function wrapper
│   ├── propeller_derivatives.m
│   ├── inducedSwirl.m       # Swirl velocity field from BEM
│   ├── eval_Swirl.m
│   └── wingApplySlipstream.m  # Apply propeller-induced velocity to wing
├── library/
│   ├── syntrac_lib.slx        # Custom Simulink block library
│   ├── aircraftSyntracCreate.m  # Build the aircraft parameter struct
│   └── aircraftSyntracGetState.m
├── models/
│   ├── sim_config2a.slx       # Main simulation model
│   ├── sim_config2a_trim.slx  # Trim variant (used by trimConfig2a)
│   └── sim_BEM_propeller.slx  # Stand-alone propeller model
├── FlightGear/
│   ├── SynTrac2a/                # Aircraft model for FlightGear
│   │   ├── SynTrac2a-set.xml     # Aircraft configuration and camera setup
│   │   └── Model/                # 3D geometry (.ac) and animation definition (.xml)
│   ├── runfg_SynTrac2a.sh        # Launch script (Linux)
│   └── runfg_SynTrac2a.bat       # Launch script (Windows)
└── modules/
    └── LADAC/               # LADAC framework submodule
```

---

## FlightGear Visualization

[FlightGear](https://www.flightgear.org/) is used as a real-time 3D visualization companion to the Simulink simulation. FlightGear runs with its own flight dynamics disabled (`--fdm=null`); all motion data come from Simulink via a UDP socket:

- **Simulink → FlightGear:** flight state (position, attitude, velocity) on port 5502
- **FlightGear → Simulink:** control surface positions on port 5503

### Installation

The `SynTrac2a/` aircraft model folder needs to be accessible to FlightGear. The provided launch scripts handle this automatically by setting the `FG_AIRCRAFT` environment variable to the `FlightGear/` folder inside the repo — no manual copy is required if you use them.

If you prefer to install the model permanently, copy the `FlightGear/SynTrac2a/` folder to your FlightGear aircraft directory:

| OS | Path |
|---|---|
| Linux | `~/.fgfs/Aircraft/` |
| Windows (modern, recommended) | `C:\Users\<username>\FlightGear\Aircraft\` |
| Windows (older installs) | `C:\Program Files\FlightGear <version>\data\Aircraft\` |

Alternatively, use the FlightGear Launcher: open **Add-ons → Additional aircraft folders**, add the `FlightGear/` folder from this repo, and FlightGear will find the model without any copying.

### Starting FlightGear

Launch FlightGear **before** pressing Run in Simulink. From a terminal inside the `FlightGear/` folder:

**Linux:**
```bash
./runfg_SynTrac2a.sh
```

**Windows:**
```bat
runfg_SynTrac2a.bat
```

On Windows the script uses `fg_install_searcher.bat` (provided in `modules/LADAC-Examples-Data/FlightGear/`) to locate the FlightGear installation automatically. Make sure the submodules are initialised so that file is present.

Once FlightGear has loaded and the aircraft is visible, start the Simulink simulation as usual. FlightGear will then follow the simulated flight in real time.

---

## Quick Start

All steps are performed from inside the `configuration_2a/` directory. The simplest way is to open MATLAB, navigate there, and run the init script:

```matlab
cd configuration_2a
init_config2a
```

This script performs the following steps in order:

1. **Add paths** — calls `addPathConfig2a()`, which adds `functions/`, `library/`, `models/`, `modules/`, `params/`, and `data/` to the MATLAB path.
2. **Create aircraft struct** — calls `aircraftSyntracCreate()`, which assembles all sub-structs (rigid body, wing geometry, propeller positions, propeller lookup table, actuator dynamics) into a single `aircraft` struct.
3. **Load environment** — calls `envirLoadParams` for ISA atmosphere at 22 000 ft.
4. **Trim and linearize** — calls `trimConfig2a(aircraft, 125, h)` for straight horizontal flight at 125 m/s (see [Trim and Linearization](#trim-and-linearization)).
5. **Open Simulink model** — opens `sim_config2a.slx` with the trimmed initial conditions already in the workspace.

After `init_config2a` completes, the workspace contains:

| Variable | Description |
|---|---|
| `aircraft` | Full aircraft parameter struct |
| `envir` | Atmosphere / environment parameters |
| `ic` | Trimmed initial conditions (states) |
| `u0` | Trimmed control inputs |
| `lin` | Full linearized state-space model (A, B, C, D) spanning all continuous states — rigid-body motion plus control surface and propeller pitch actuator dynamics (e.g. elevator, aileron, rudder). |
| `lin_red` | Reduced state-space with actuator states eliminated, retaining only the rigid-body motion states. `lin_red.D` is the control effectiveness matrix used as input to the control allocation in Project A04. |
| `xprop_tbl_bus` | Simulink Bus object for the propeller lookup table |

Press **Run** in `sim_config2a.slx` to start the simulation.

---

## Propeller Data Pipeline

Propeller performance data originates from XROTOR calculations provided by project partners. The data goes through a two-step offline preprocessing before it reaches the Simulink model.

### Step 1 — Build the lookup table (offline, run once per data update)

```matlab
cd configuration_2a
xprop_build_matfile('data/xprop_data.txt', 'data/xprop_data.mat')
```

**Input format** (`xprop_data.txt`) — plain-text, whitespace-delimited, with a header line:

```
pitch_delta  V  Ct  Cp  eff  J
-15  40.0  0.2341  0.4812  0.65  1.23
...
```

- `pitch_delta` — blade pitch increment relative to the reference pitch (54.7° at r/R = 0.7), in degrees
- `J` — propeller advance ratio, dimensionless. Defined as J = V / (n · D), where V is the freestream velocity [m/s], n is the rotational speed in revolutions per second [rev/s] = RPM / 60, and D is the propeller diameter [m]. At the reference cruise condition (V = 110 m/s, 1225 RPM, D = 2.35 m): n = 1225 / 60 ≈ 20.4 rev/s → J = 110 / (20.4 × 2.35) ≈ 2.29. Note: do not confuse this with the helicopter inflow ratio λ = (V sinα + v_i) / (ΩR), which uses tip speed ΩR as reference and includes the induced velocity v_i. The propeller J uses n·D = ΩD/(2π) = ΩR/π as its reference, so J = πμ relative to the helicopter advance ratio μ = V/(ΩR).
- `Ct`, `Cp` — thrust and power coefficients
- Lines starting with `%` or `#` are treated as comments

The script:
1. Parses all data rows and assembles C_T and C_P grids indexed by (Δβ, J)
2. Pre-computes derivative grids (∂C_T/∂β, ∂C_P/∂β, ∂C_T/∂J, ∂C_P/∂J) via central finite differences — no derivatives are computed at runtime
3. Fills ragged table boundaries by linear extrapolation to cover the full J range for every pitch level
4. Saves the result to `xprop_data.mat`

When replacing data (e.g. new XROTOR run or different RPM), place the new text file at `data/xprop_data.txt` and re-run the command above. Then commit the updated `.mat` file.

### Step 2 — Load into MATLAB and Simulink (automatic, called by `aircraftSyntracCreate`)

```matlab
prop_table = xprop_load_table('data/xprop_data.mat');
```

This loads the `.mat` file, replaces NaN padding with zeros (required for Simulink bus signals), and registers a `Simulink.Bus` object named `xprop_tbl_bus` in the base workspace. The Simulink MATLAB Function block receives the full table as a typed bus signal and performs bilinear interpolation at runtime.

You do **not** need to call `xprop_load_table` manually — `aircraftSyntracCreate` (called from `init_config2a`) does this automatically.

---

## Trim and Linearization

`trimConfig2a` handles both trimming and linearization in one call:

```matlab
[ic, u0, info, lin, lin_red] = trimConfig2a(aircraft, V, h)
```

| Argument | Description |
|---|---|
| `aircraft` | Aircraft struct from `aircraftSyntracCreate()` |
| `V` | Trim airspeed in m/s |
| `h` | Altitude in m |

**Trim condition:** straight horizontal flight (zero sideslip, zero climb rate). The free trim variables are elevator deflection and collective propeller pitch. `jj_trim` iterates until force/moment equilibrium is reached.

**Linearization:** once trim converges, `jj_lin` computes the full Jacobian numerically using finite differences around the trim point. The outputs are:

- `lin` — full state-space model spanning all continuous states: rigid-body motion plus control surface and propeller pitch actuator dynamics (e.g. elevator, aileron, rudder).
- `lin_red` — reduced model with actuator states eliminated via model reduction, retaining only the rigid-body motion states. `lin_red.D` is the control effectiveness matrix, which feeds into the control allocation algorithm in Project A04.

**Trimming a different flight condition:**

```matlab
[ic, u0, ~, lin, lin_red] = trimConfig2a(aircraft, 110, ft2m(22000));
```

The resulting `ic` and `u0` can be used directly as initial conditions in `sim_config2a.slx`.

---

## Changing Aircraft Parameters

All configuration-specific parameters live in `configuration_2a/params/`:

| File | Contents |
|---|---|
| `wing_params_2a.m` | Main wing geometry (span, chord, sweep, twist, dihedral, flap layout) |
| `wing_params_htp2a.m` | Horizontal tail geometry |
| `wing_params_vtp2a.m` | Vertical tail geometry |
| `airfoilAnalyticSimple_params_wing2a.m` | 2D airfoil model parameters for the main wing |
| `actuators_params_syntrac.m` | Actuator dynamics (time constants, rate limits) |

Propeller positions and rotation directions are set directly in `aircraftSyntracCreate.m`. Mass and inertia for MTOM and OEM cases are also defined there.

After changing any parameter file, re-run `init_config2a` (or at minimum re-run `aircraftSyntracCreate`) before simulating.
