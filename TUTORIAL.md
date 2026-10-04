# TUM-CONTROL Tutorial

This tutorial explains how the repository is organized, how one closed-loop simulation runs, how the three MPC controllers differ, and how the safe weights-varying MPC learning pipeline fits around them.

The repository is a research framework for autonomous-vehicle motion control. It combines a nonlinear single-track vehicle model, CasADi expressions, ACADOS nonlinear MPC solvers, trajectory data, simulation/logging utilities, and optional Bayesian-optimization and reinforcement-learning tools.

## 1. The shortest useful mental model

A normal simulation follows this loop:

```text
YAML configuration
      |
      v
MPC_Sim -------------------- reference trajectory and track
      |                       |
      v                       v
vehicle simulator <--- MPC controller <--- local reference horizon
      |                       |
      +-------- state, input, predictions, diagnostics
                         |
                         v
                       Logger
```

At each control iteration:

1. The planner emulator finds the part of the reference trajectory near the vehicle.
2. The controller solves an ACADOS optimal-control problem over the next `Tp` seconds.
3. The first control action is applied.
4. The vehicle model advances one simulation step, or the next MPC prediction is used directly.
5. The measured/estimated state becomes the next MPC initial condition.
6. States, references, controls, predictions, and solver statistics are logged.

The primary entry point is [main.py](main.py). The active controller is selected by changing the imported controller class near the top of that file.

## 2. Installation and prerequisites

### Python environment

The checked-in [requirements.txt](requirements.txt) contains the core scientific, CasADi, plotting, and YAML dependencies. The project was originally tested with Python 3.7--3.9.

Create and activate an environment from the repository root, then install the requirements:

```powershell
py -3.9 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

On Linux or macOS, the activation command is:

```bash
source .venv/bin/activate
```

### ACADOS

The MPC formulations generate and load ACADOS solver code. Install ACADOS separately by following the official ACADOS installation instructions, then install its Python template interface:

```bash
pip install -e <acados_root>/interfaces/acados_template
```

Set the ACADOS environment variables before running the simulation. On Windows, use the equivalent PowerShell environment variables for your ACADOS installation; on Linux/macOS the typical form is:

```bash
export ACADOS_SOURCE_DIR=<acados_root>
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:<acados_root>/lib
```

The learning pipeline additionally expects packages that are not listed in the root requirements file, including PyTorch, Stable-Baselines3, Gymnasium, BoTorch, GPyTorch, and scikit-learn.

Run commands from the repository root. Many paths in the code are intentionally relative, for example `Config/`, `Trajectories/`, `Logs/`, and `Learning_To_Adapt/`.

## 3. First run

Before the first run, inspect [Config/EDGAR/sim_main_params.yaml](Config/EDGAR/sim_main_params.yaml) and [Config/EDGAR/MPC_params.yaml](Config/EDGAR/MPC_params.yaml).

The supplied simulation configuration uses:

```yaml
simMode: 0
track_file: track_monteblanco.json
ref_traj_file: reftraj_monteblanco_edgar.json
Ts: 0.02
T: 100.0
Tp: 3.04
Ts_MPC: 0.08
live_visualization: 0
save_logs: true
```

The MPC horizon has:

```text
N = int(Tp / Ts_MPC) = int(3.04 / 0.08) = 38
```

Start the simulation with:

```powershell
python main.py
```

The default configuration selects the reduced robustified controller and has `enable_WMPC: True`. Therefore, a clean checkout also needs the trained WMPC model under the configured path and the ML dependencies. For a basic static-weight controller experiment, set `enable_WMPC: False` in `MPC_params.yaml`.

For a quick smoke test, temporarily reduce `T` and keep `live_visualization: 0`. ACADOS solver generation and compilation can dominate the first run. Later runs may reuse generated solver artifacts when the corresponding build flags are disabled.

## 4. What `main.py` does

The entry point is deliberately small. Its important operations are:

```python
sim = MPC_Sim(sim_main_params, logs_path)
vehicle = PassengerVehicleSimulator(config_path, sim.sim_main_params, sim.Ts)
X0_sim = sim.set_Vehicle(vehicle)
MPC = Model_Predictive_Controller(config_path, MPC_params_file,
                                   sim.sim_main_params, sim.X0_MPC)
logger = Logger(sim, X0_sim, vehicle.sim_constraints.alat, MPC)
```

The controller alias is chosen by commenting/uncommenting imports. The current source selects:

```python
from Model_Predictive_Controller.Reduced_Robustified_NMPC. \
    Reduced_Robustified_NMPC_class import \
    Reduced_Robustified_Nonlinear_Model_Predictive_Controller as Model_Predictive_Controller
```

The simulation loop then calls:

```python
current_ref_idx, current_ref_traj = PlannerEmulator(...)
u0, pred_X, MPC_stats = MPC.solve(current_ref_traj)
x_next, x_next_MPC, x_next_sim, x_next_sim_disturbed = \\
    sim.sim_step(i, x, u0, pred_X)
x_next = sim.StateEstimation(x_next)
MPC.set_initial_state(x_next)
logger.logging_step(...)
```

If ACADOS returns a nonzero status, `main.py` asks the controller to reinitialize its solver. The loop tracks maximum and total solve time, then `Logger.evaluation()` produces plots and optionally saves a compressed log.

## 5. Configuration files

### Simulation configuration

[Config/EDGAR/sim_main_params.yaml](Config/EDGAR/sim_main_params.yaml) controls:

- `simMode`: `0` for controller-in-the-loop with an independent simulator, `1` for MPC-in-the-loop using the MPC prediction.
- Reference and track files.
- Simulator and prediction-model parameter files.
- `Ts`, `T`, `Tp`, and `Ts_MPC`.
- Live visualization and GIF generation.
- State-estimation and derivative disturbance generation.
- Log output settings.

The default files describe the EDGAR Volkswagen T7 model and the Monteblanco circuit. To use another supplied circuit, change both `track_file` and `ref_traj_file` consistently, for example to the Modena or LVMS files in [Trajectories](Trajectories).

### MPC configuration

[Config/EDGAR/MPC_params.yaml](Config/EDGAR/MPC_params.yaml) controls:

- Cost-function type: `NONLINEAR_LS` or `EXTERNAL`.
- ACADOS code generation/build flags.
- Static tracking and input weights.
- Optional WMPC model and update period.
- Combined acceleration-limit shape.
- Robust and stochastic uncertainty parameters.

The default tracking cost uses the diagonal weights:

```text
Q = diag(q_lon, q_lat, q_yaw, q_vel)
R = diag(r_jerk, r_steering_rate)
```

Scaling values `s_*` are applied by the controller before the matrices are built. `L1_pen` and `L2_pen` are soft-constraint penalties.

### Vehicle and tire configuration

- [veh_params_pred.yaml](Config/EDGAR/veh_params_pred.yaml) configures the prediction model.
- [veh_params_sim.yaml](Config/EDGAR/veh_params_sim.yaml) configures the independent simulator.
- [pacejka_params.yaml](Config/EDGAR/pacejka_params.yaml) configures the tire model.
- [ggv.csv](Config/EDGAR/ggv.csv) supplies speed-dependent longitudinal and lateral acceleration limits.

Keeping simulator and prediction parameters separate is useful: it allows controlled model mismatch in controller-in-the-loop experiments.

## 6. State, input, and reference conventions

### MPC prediction state

The prediction model in [pred_model_dynamic_stm_pacejka.py](Prediction_Models/pred_model_dynamic_stm_pacejka.py) uses an eight-element state:

```text
x_MPC = [x_pos, y_pos, yaw, v_long, v_lat, yaw_rate, delta_f, acceleration]
```

Its two inputs are:

```text
u_MPC = [jerk, steering_rate]
```

The acceleration state is integrated from jerk, and the steering state is integrated from steering rate.

### Independent simulator state

The model in [sim_model_dynamic_stm_pacejka.py](Vehicle_Simulator/sim_model_dynamic_stm_pacejka.py) uses a seven-element state:

```text
x_sim = [x_pos, y_pos, yaw, v_long, v_lat, yaw_rate, delta_f]
```

Its inputs are:

```text
u_sim = [acceleration, steering_rate]
```

In CiL mode, the first MPC action contains jerk and steering rate. The simulator receives the predicted acceleration from the first prediction step together with the steering rate. The simulator result is then extended with acceleration to form the next eight-element MPC state.

This difference is intentional, but important: the prediction model integrates jerk while the independent simulator is driven by acceleration. It creates the model mismatch that CiL experiments can expose.

### Reference trajectory

`PlannerEmulator()` in [MPC_sim_utils.py](Utils/MPC_sim_utils.py) finds the closest reference point, collects a horizon of points, wraps around circuit boundaries, and interpolates the result for the MPC grid. Each reference horizon contains at least:

```text
pos_x, pos_y, ref_yaw, ref_v
```

Yaw values are unwrapped before interpolation so that crossing the `0`/`2*pi` boundary does not create a false large rotation.

## 7. Vehicle model

Both models are nonlinear dynamic single-track models with Pacejka lateral tire forces. They include vehicle kinematics, yaw dynamics, longitudinal resistance, aerodynamic drag, axle loads, steering dynamics, and combined-slip scaling.

The simulator integrates the CasADi model in [VehicleSimulator.py](Vehicle_Simulator/VehicleSimulator.py). The prediction model is assembled for ACADOS from [pred_model_dynamic_stm_pacejka.py](Prediction_Models/pred_model_dynamic_stm_pacejka.py).

The combined acceleration constraint is based on:

$$
\left(\frac{a_{lon}}{a_{x,max}}\right)^2 +
\left(\frac{a_{lat}}{a_{y,max}}\right)^2 \leq 1,
$$

where the implementation uses the acceleration state for $a_{lon}$ and approximately $v_{long} \cdot yaw\_rate$ for $a_{lat}$. The lookup table in `ggv.csv` supplies speed-dependent limits.

The model is a research model, not a complete production vehicle model. The source identifies missing or provisional pieces, including longitudinal Pacejka forces, downforce, and some parameter validation. The prediction model also hardcodes zero banking while the simulator reads a banking parameter.

## 8. The MPC controller variants

All controller classes expose the same core lifecycle:

```text
construct -> set_initial_state -> solve(reference) -> return u0/predictions/stats
```

### Nominal NMPC

[Model_Predictive_Controller/Nominal_NMPC/NMPC_class.py](Model_Predictive_Controller/Nominal_NMPC/NMPC_class.py) solves the standard nonlinear tracking problem. The ACADOS formulations are in:

- [NMPC_STM_acados_settings.py](Model_Predictive_Controller/Nominal_NMPC/NMPC_STM_acados_settings.py)
- [NMPC_STM_acados_settings_dev_lonlat.py](Model_Predictive_Controller/Nominal_NMPC/NMPC_STM_acados_settings_dev_lonlat.py)

The usual `NONLINEAR_LS` cost tracks position, yaw, velocity, jerk, and steering rate through `yref`. The `EXTERNAL` form passes the reference through model parameters.

### Stochastic NMPC

[SNMPC_class.py](Model_Predictive_Controller/Stochastic_NMPC/SNMPC_class.py) augments the prediction with uncertainty samples. [pred_model_dynamic_disc.py](Model_Predictive_Controller/Stochastic_NMPC/pred_model_dynamic_disc.py) and [stochastic_mpc_utils.py](Model_Predictive_Controller/Stochastic_NMPC/stochastic_mpc_utils.py) build the polynomial-chaos representation.

The key parameters are:

- `n_samples`: uncertainty collocation samples.
- `expansion_degree`: Hermite polynomial expansion degree.
- `gamma`: risk/chance-constraint parameter.
- `uncertainty_propagation_horizon`: how far uncertainty is propagated.
- `stds` and `disturbance_type`: assumed disturbance model.

The stacked stochastic state contains one nominal eight-state trajectory plus one eight-state trajectory per uncertainty sample.

### Reduced robustified NMPC (R2NMPC)

The active default is [Reduced_Robustified_NMPC_class.py](Model_Predictive_Controller/Reduced_Robustified_NMPC/Reduced_Robustified_NMPC_class.py). It solves a nominal-size OCP, extracts local linearized dynamics from the ACADOS SQP-RTI step, and propagates an ellipsoidal uncertainty matrix outside the OCP. The resulting steering and acceleration backoffs tighten constraints over the configured propagation horizon.

The robust formulation is implemented around:

- `P_propagation()` in `Robust_NMPC_pred_model_utils.py`.
- `pred_stm_disturbed()` for disturbed dynamics.
- ACADOS setup in `Reduced_Robustified_NMPC_acados_settings.py`.

R2NMPC requires the SQP-RTI solver type. Its disturbance channels principally affect yaw, longitudinal velocity, lateral velocity, and yaw rate.

## 9. Simulation modes and disturbances

### CiL mode (`simMode: 0`)

The independent seven-state simulator advances at `Ts`. This is the more realistic mode for testing prediction/simulator mismatch. It can add:

- Derivative disturbances to the simulated dynamics.
- State-estimation disturbances before the next MPC solve.
- Disturbance playback from a previous log.

### MPC-in-the-loop mode (`simMode: 1`)

The next state is taken directly from `pred_X[1, :]`. This is useful for isolating controller behavior, but it bypasses independent vehicle integration and therefore does not test model mismatch in the same way.

The filtering logic is in `MPC_Sim.StateEstimation()`. Disturbance generation and tracking-error helpers are in [MPC_sim_utils.py](Utils/MPC_sim_utils.py).

## 10. Logging and results

[Logging_Plotting.py](Utils/Logging_Plotting.py) owns arrays for the simulation state, independent simulator state, controls, references, solver diagnostics, tracking errors, and disturbance realizations.

The solver-debug row is:

```text
[cost, total_solver_time, sqp_iterations, max_qp_iterations, status]
```

When `save_logs: True`, evaluation writes a `full_logs.npz` file containing fields such as:

```text
MPC_SimX, CiLX, simU, simREF, simSolverDebug,
a_lat, dev_lat, dev_long, dev_vel, dev_yaw, t
```

The logger also creates performance and track plots. Use the standalone scripts in [Papers_Plots/ACC24_SNMPC](Papers_Plots/ACC24_SNMPC) for historical paper figures; those scripts expect particular old log layouts and are not a general plotting command.

## 11. Enabling WMPC

Weights-varying MPC is optional at runtime. When enabled, a controller:

1. Loads a Stable-Baselines3 PPO model from `<WMPC_model>/best_model/best_model`.
2. Loads the corresponding `rl_config.yaml`.
3. Reads a CSV containing candidate seven-parameter weight sets.
4. Builds an observation from current tracking and look-ahead information.
5. Predicts a discrete action every `weights_update_period` MPC steps.
6. Calls `update_cost_function_weights()` with the selected parameter set.

To run a trained policy, set in `MPC_params.yaml`:

```yaml
enable_WMPC: True
WMPC_model: Learning_To_Adapt/SafeRL_WMPC/_models/<model_directory>
```

The model path is relative to the repository root. The model directory must contain the expected Stable-Baselines3 checkpoint, `rl_config.yaml`, and action-parameter CSV.

For training, disable WMPC and disable ACADOS solver build/code generation first. Parallel training workers otherwise compete for generated solver code and the policy would try to load itself during training.

## 12. Safe WMPC training pipeline

The learning extension is sequential:

```text
Bayesian optimization -> Pareto parameter sets -> PPO policy -> online WMPC
```

### Step 1: Bayesian optimization

Run [bo_optimize.py](Learning_To_Adapt/SafeRL_WMPC/bo_optimize.py) after configuring [bo_config.yaml](Learning_To_Adapt/SafeRL_WMPC/_config/bo_config.yaml). The optimizer evaluates static cost weights and searches for a Pareto tradeoff between lateral tracking and velocity tracking.

The optimized vector is:

```text
[q_xy, q_yaw, q_vel, r_jerk, r_steering_rate, L1, L2]
```

The BO implementation is distributed across [bayesian_optimization.py](Learning_To_Adapt/SafeRL_WMPC/BO_WMPC/bayesian_optimization.py), [objective_function.py](Learning_To_Adapt/SafeRL_WMPC/BO_WMPC/objective_function.py), surrogate/acquisition modules, and track segmentation utilities.

### Step 2: Pareto postprocessing

Run [bo_postprocess_parameters.py](Learning_To_Adapt/SafeRL_WMPC/bo_postprocess_parameters.py) to filter feasible trials, find nondominated points, reduce each front with KMeans, and export representative CSV parameter sets.

Check the identifiers and output paths in the script before running it. The current script contains hardcoded run-selection values rather than a fully generalized command-line interface.

### Step 3: PPO training

Run [rl_training.py](Learning_To_Adapt/SafeRL_WMPC/rl_training.py) with [rl_config.yaml](Learning_To_Adapt/SafeRL_WMPC/_config/rl_config.yaml) configured for the exported action CSV. The environment in [environment.py](Learning_To_Adapt/SafeRL_WMPC/RL_WMPC/environment.py) treats each action as an index into the finite set of safe weight vectors.

For every RL action, the environment:

1. Updates the MPC cost weights.
2. Runs a configured number of MPC steps.
3. Computes lateral and velocity tracking errors.
4. Produces a reward and normalized observation.

Observations are generated by [observation.py](Learning_To_Adapt/SafeRL_WMPC/RL_WMPC/observation.py) and include current lateral/velocity deviations plus sampled future reference velocities and yaw rates. Rewards are generated by [reward.py](Learning_To_Adapt/SafeRL_WMPC/RL_WMPC/reward.py).

## 13. How to change the experiment

### Switch controllers

In [main.py](main.py), replace the active controller import with one of the other controller classes and comment out the current import. The controller APIs are designed to be interchangeable, but each variant has its own ACADOS formulation and generated-code expectations.

### Change the circuit

Edit `track_file` and `ref_traj_file` together in `sim_main_params.yaml`. The planner emulator is designed primarily for loop circuits and wraps around the end of the reference.

### Add model mismatch

Keep the simulator and MPC vehicle parameter files different, or enable derivative/state-estimation disturbances in the simulation YAML. Start with a short run and inspect `simSolverDebug` and the tracking-error plots before increasing the disturbance magnitude.

### Tune the cost

Modify `q_lon`, `q_lat`, `q_yaw`, `q_vel`, `r_jerk`, and `r_steering_rate` in `MPC_params.yaml`. For WMPC, the runtime action vector maps to the seven values used by `update_cost_function_weights()` rather than directly changing the controller class.

### Change acceleration constraints

Use `combined_acc_limits`:

```text
0 = separate longitudinal/lateral limits
1 = diamond-shaped combined limit
2 = circular combined limit
```

The speed-dependent limits come from `ggv.csv`.

## 14. Troubleshooting checklist

### ACADOS import or shared-library error

Check that `ACADOS_SOURCE_DIR` points to the ACADOS checkout, the native libraries are on the runtime library path, and `acados_template` is installed in the active Python environment.

### Missing WMPC model or `stable_baselines3`

Either install the learning dependencies and provide the configured model directory, or set `enable_WMPC: False` for a static-weight run.

### Missing generated ACADOS JSON/solver artifacts

The controller settings request generated artifacts for the selected formulation. The repository includes `acados_ocp_SNMPC.json`, but the nominal and R2 code paths may request formulation-specific artifacts that are not checked in. Enable generation/build on the first compatible run, then disable it only when those artifacts already exist.

### Solver status is nonzero

Inspect the printed ACADOS status, `simSolverDebug`, and the current reference. Try a shorter horizon, smaller initial disturbances, a shorter test duration, or a less aggressive cost/constraint setup. Also verify that the reference contains at least `N + 1` usable points.

### Relative paths fail

Run from the repository root. The code frequently concatenates paths such as `Config/` and `Learning_To_Adapt/...` rather than resolving paths relative to the source file.

### Training conflicts between workers

For BO/RL training, disable WMPC and ACADOS build/code generation as described above. Generated solver code is shared filesystem state and is not designed for multiple workers to rebuild it simultaneously.

## 15. Suggested reading order in the source

For a first code tour, read these files in order:

1. [main.py](main.py)
2. [SimulationMode_main_class.py](Utils/SimulationMode_main_class.py)
3. [VehicleSimulator.py](Vehicle_Simulator/VehicleSimulator.py)
4. [sim_model_dynamic_stm_pacejka.py](Vehicle_Simulator/sim_model_dynamic_stm_pacejka.py)
5. [MPC_sim_utils.py](Utils/MPC_sim_utils.py)
6. The selected controller class, starting with [Reduced_Robustified_NMPC_class.py](Model_Predictive_Controller/Reduced_Robustified_NMPC/Reduced_Robustified_NMPC_class.py)
7. The matching ACADOS settings module
8. [Logging_Plotting.py](Utils/Logging_Plotting.py)
9. The relevant YAML files in [Config/EDGAR](Config/EDGAR)
10. The learning entry points under [Learning_To_Adapt/SafeRL_WMPC](Learning_To_Adapt/SafeRL_WMPC)

That path follows the actual runtime ownership: orchestration, state progression, dynamics, reference generation, optimization, and results.

## 16. Current repository caveats

This is research software with several historical or work-in-progress edges:

- The root requirements file does not include every dependency used by the learning pipeline.
- Some learning READMEs contain older paths such as `Python/...` that no longer match this checkout.
- The plotting scripts target historical experiments and fixed output layouts.
- No automated test suite is present; validation is primarily through simulation and generated plots.
- The non-loop planner branch is unfinished; circuit references are the supported path.
- Some model parameters and physical effects are provisional.
- `simMode: 1` is useful for controller isolation but does not reproduce independent simulator bookkeeping exactly like CiL mode.

When extending the code, preserve the state-dimension distinction and verify both the first returned control action and the first predicted state. Those are the two contracts shared by the main loop and the controller implementations.

## 17. Repository map

The repository is organized by responsibility rather than by Python package. The following map is the quickest way to find the implementation behind a feature.

| Path | Responsibility |
| --- | --- |
| [main.py](main.py) | Application entry point; loads YAML, constructs the simulator/controller/logger, runs the closed loop, and evaluates results. |
| [Config/](Config) | Vehicle, tire, MPC, disturbance, visualization, and simulation configuration. |
| [Trajectories/](Trajectories) | Track geometry and reference trajectories for Monteblanco, Modena, and LVMS. |
| [Prediction_Models/](Prediction_Models) | CasADi prediction model used by the MPC and ACADOS. |
| [Vehicle_Simulator/](Vehicle_Simulator) | Independent seven-state vehicle model, integrator, disturbance generation, and simulator wrapper. |
| [Model_Predictive_Controller/Nominal_NMPC/](Model_Predictive_Controller/Nominal_NMPC) | Nominal nonlinear MPC class and its ACADOS formulations. |
| [Model_Predictive_Controller/Stochastic_NMPC/](Model_Predictive_Controller/Stochastic_NMPC) | Polynomial-chaos uncertainty model, stochastic MPC class, and stochastic utilities. |
| [Model_Predictive_Controller/Reduced_Robustified_NMPC/](Model_Predictive_Controller/Reduced_Robustified_NMPC) | R²NMPC class, ACADOS setup, covariance propagation, and constraint back-offs. |
| [Utils/](Utils) | Planner emulator, simulation-mode orchestration, logging, plotting, and color helpers. |
| [Learning_To_Adapt/SafeRL_WMPC/](Learning_To_Adapt/SafeRL_WMPC) | Bayesian optimization, Pareto-set reduction, PPO training, evaluation, and WMPC runtime support. |
| [Papers_Plots/](Papers_Plots) | Reproduction scripts for historical paper figures and solver-time experiments. |
| `codegen_ocp_*/` and `acados_*.json` | Generated ACADOS metadata and C/Cython solver artifacts. These are build outputs, not controller source. |
| [inspection.ipynb](inspection.ipynb) | Interactive inspection notebook; it is not required for a normal simulation. |

The small README files in individual directories contain formulation-specific background. This tutorial is the operational reference; when a local README contains an old path such as `Python/...`, use the paths in this map.

## 18. Runtime ownership and call graph

The normal execution path is:

```text
main.py
  -> MPC_Sim.__init__()
       -> load track/reference and derive N, Nsim, initial states
  -> PassengerVehicleSimulator(...)
       -> load simulator vehicle/tire parameters and create integrator
  -> Model_Predictive_Controller(...)
       -> load prediction model, constraints, weights, and ACADOS solver
  -> Logger(...)
  -> repeat Nsim times
       -> PlannerEmulator(...)
       -> MPC.solve(reference_horizon)
       -> MPC_Sim.sim_step(...)
       -> MPC_Sim.StateEstimation(...)
       -> MPC.set_initial_state(...)
       -> Logger.logging_step(...)
       -> Logger.step_live_visualization(...)
  -> Logger.evaluation(...)
```

The important contracts between these objects are:

* `PlannerEmulator` returns a dictionary of reference arrays with at least `pos_x`, `pos_y`, `ref_yaw`, and `ref_v`.
* `solve()` returns `(u0, pred_X, MPC_stats)`. `u0` is the first control action, `pred_X` is the predicted state trajectory, and the last element of `MPC_stats` is the ACADOS status.
* `sim_step()` returns both the next state used by the MPC and the simulator bookkeeping states used by the logger.
* `set_initial_state()` updates the equality constraint at the first shooting node; it does not rebuild the solver.
* `Logger.evaluation()` is the post-processing boundary. Simulation data should be added to `logging_step()` before adding new plots or metrics.

## 19. Configuration reference

### `sim_main_params.yaml`

| Key | Meaning |
| --- | --- |
| `simMode` | `0` runs the independent simulator (CiL); `1` advances from the MPC prediction (MPC-in-the-loop). |
| `trajectory_path`, `track_file`, `ref_traj_file` | Relative paths to the track and reference data. Change the track and reference together. |
| `idx_ref_start` | Initial reference index. |
| `veh_params_file_simulator`, `tire_params_file_simulator` | Parameters for the independent simulator. |
| `veh_params_file_MPC`, `tire_params_file_MPC` | Parameters for the prediction model. Keeping these different creates model mismatch. |
| `live_visualization` | `0` disables plots, `1` shows track/vehicle motion, `2` adds tracking and acceleration diagnostics. |
| `live_plot_freq` | Number of simulation frames skipped between live-plot updates. |
| `GIF_animation_generation`, `GIF_file_name` | Optional GIF capture; it requires live visualization and slows the run. |
| `save_logs`, `file_logs_name` | Whether and how the compressed log is written. |
| `Ts`, `T`, `Tp`, `Ts_MPC` | Simulator step, simulation duration, prediction horizon, and MPC grid step. `N = int(Tp / Ts_MPC)`. |
| `disturbance_playback`, `playback_log_file` | Reuse a previous disturbance realization instead of sampling a new one. |
| `simulate_state_estimation`, `w_*` | Add measurement/state-estimation noise before the next MPC solve. |
| `simulate_disturbances`, `w_*_dot` | Add disturbances to state derivatives in the independent simulator. |

### `MPC_params.yaml`

| Key | Meaning |
| --- | --- |
| `costfunction_type` | ACADOS cost mode: `NONLINEAR_LS` or `EXTERNAL`. |
| `solver_build`, `solver_generate_C_code` | Build and generate ACADOS artifacts. Enable them for the first run of a formulation; disable them only when compatible artifacts already exist. |
| `enable_WMPC`, `WMPC_model`, `weights_update_period` | Enable a trained PPO policy, select its model directory, and choose how often weights are changed. |
| `s_*` | Scaling factors applied before constructing cost weights. |
| `q_lon`, `q_lat`, `q_yaw`, `q_vel` | State/reference tracking weights. |
| `r_jerk`, `r_steering_rate` | Input regularization weights. |
| `L1_pen`, `L2_pen` | Linear and quadratic soft-constraint penalties. |
| `lookuptable_gg_limits` | Relative path to the speed-dependent acceleration-limit table. |
| `combined_acc_limits` | `0`: separate limits; `1`: diamond; `2`: circular combined acceleration limit. |
| `stds`, `uncertainty_propagation_horizon` | Disturbance magnitudes and propagation depth used by stochastic/robust controllers. |
| `n_samples`, `gamma`, `expansion_degree`, `disturbance_type` | Polynomial-chaos and chance-constraint settings used by SNMPC. |

The cost vector used by WMPC has seven entries:

```text
[q_xy, q_yaw, q_vel, r_jerk, r_steering_rate, L1_pen, L2_pen]
```

The exact interpretation of `q_xy` is controller-specific; inspect the active controller's `update_cost_function_weights()` method before introducing a new action CSV.

## 20. Data formats and outputs

### Track and reference JSON

Track JSON files contain the 2D circuit geometry used by visualization. Reference JSON files contain the arrays consumed by `PlannerEmulator`. The reference must provide enough consecutive points for the complete horizon and must use the same circuit as the track file. Looping is enabled by the current `main.py` call, so the planner wraps from the end of a circuit to its beginning.

### Vehicle and tire YAML

`veh_params_pred.yaml` and `veh_params_sim.yaml` hold model parameters for different purposes. `pacejka_params.yaml` contains tire coefficients shared by both models. Do not silently merge these files: the separation is how CiL model mismatch is controlled.

### GGV CSV

`ggv.csv` supplies speed-dependent acceleration limits. It is read by the controller formulation and is therefore part of the optimization problem, not merely a plotting input.

### Simulation logs

With `save_logs: True`, logs are written below `Logs/` as compressed NumPy archives. The most useful fields are:

| Field | Contents |
| --- | --- |
| `MPC_SimX` | MPC prediction/in-loop state history. |
| `CiLX` | Independent simulator state history. |
| `simU` | Applied controls. |
| `simREF` | Reference horizon/current reference history. |
| `simSolverDebug` | Cost, solver time, SQP iterations, QP iterations, and status. |
| `a_lat`, `dev_lat`, `dev_long`, `dev_vel`, `dev_yaw` | Acceleration and tracking diagnostics. |
| `t` | Simulation time. |

Load a log with:

```python
import numpy as np

log = np.load("Logs/<run>/full_logs.npz", allow_pickle=True)
print(log.files)
print(log["simSolverDebug"].shape)
```

## 21. Running the learning tools

The learning scripts are Python entry points, not import-time library APIs. Run them from the repository root so their relative paths resolve.

```powershell
# Inspect available options/configuration in the script before a long run.
python Learning_To_Adapt/SafeRL_WMPC/bo_optimize.py --help

# Run Bayesian optimization using _config/bo_config.yaml.
python Learning_To_Adapt/SafeRL_WMPC/bo_optimize.py

# Reduce a completed BO Pareto front to a finite action set.
python Learning_To_Adapt/SafeRL_WMPC/bo_postprocess_parameters.py

# Train or continue the PPO policy configured in _config/rl_config.yaml.
python Learning_To_Adapt/SafeRL_WMPC/rl_training.py
python Learning_To_Adapt/SafeRL_WMPC/rl_training.py -cont <model_identifier>
```

The current BO post-processing and baseline scripts contain configuration values in source rather than a complete stable CLI. Read the top-level constants and commented argument parser before running them. The expected dependency set extends beyond [requirements.txt](requirements.txt): the learning pipeline imports PyTorch, Gymnasium, Stable-Baselines3, BoTorch, GPyTorch, scikit-learn, and related packages.

The safe order is:

1. Configure and run BO.
2. Inspect feasibility and Pareto plots.
3. Post-process the selected BO run into an action CSV under `_parameters/`.
4. Point `rl_config.yaml` at that CSV and train PPO.
5. Verify the model directory contains the checkpoint, `rl_config.yaml`, and action parameters.
6. Set `enable_WMPC: True` and `WMPC_model` in the simulation configuration.
7. Run a short CiL simulation before a long benchmark.

Never run training with WMPC enabled. During BO/RL training, also disable ACADOS code generation/build or isolate each worker's generated-code directory; generated solver files are shared mutable state.

## 22. Extending the repository safely

### Add a controller

Implement the lifecycle used by `main.py`: constructor, `solve(reference)`, `set_initial_state(state)`, and (if needed) `reintialize_solver(state)`. Add the matching ACADOS settings module, use the existing state/input convention, and expose `nx`, `model`, and `constraint` because the logger and evaluation code use them. Then switch only the import alias in `main.py` for an experiment.

### Add a track

Add one track JSON and one compatible reference JSON under [Trajectories](Trajectories). Update `sim_main_params.yaml`, run with a short `T`, and verify the first planner horizon has at least `N + 1` points and that yaw remains continuous after interpolation.

### Add a disturbance

Keep disturbance generation in the simulator/simulation utilities, add its configuration keys to `sim_main_params.yaml`, and log the realization. If the disturbance affects the prediction model's uncertainty assumptions, update `MPC_params.yaml` as well; otherwise the robust/stochastic controller is being evaluated with inconsistent assumptions.

### Add a metric or plot

Store per-step values in `Logging_Plotting.Logger.logging_step()`, aggregate them in `evaluation()`, and keep plotting side effects behind the existing visualization/logging flags. Do not derive metrics from only the live plot because live plotting can be disabled or frame-skipped.

## 23. Validation checklist

There is currently no automated test suite. Use this checklist for every controller or configuration change:

1. Run a syntax/import check in the selected environment.
2. Run a short simulation (`T` of a few seconds, `live_visualization: 0`).
3. Confirm `N == int(Tp / Ts_MPC)` and that the reference is long enough.
4. Confirm ACADOS status remains zero and inspect `simSolverDebug`.
5. Check that `u0` has two inputs and `pred_X` has eight state columns.
6. Compare CiL and MPC-in-the-loop runs when changing dynamics.
7. Inspect tracking plots and the saved `.npz` before increasing `T`.
8. Only then enable disturbances, WMPC, live visualization, or long benchmark runs.

This workflow catches the most common integration failures: mismatched state dimensions, stale generated solvers, insufficient reference horizons, wrong relative working directories, and WMPC artifacts that do not match their action parameter CSV.
