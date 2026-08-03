# PX4 Quadrotor Three-Motor Emergency Landing SITL Experiment

> **Experimental, SITL only.** Do not use these control changes on hardware.
> The result is a degraded emergency landing after one motor stops, not power
> redundancy or production-ready fault-tolerant flight.

This directory is the reproducibility package for a PX4 quadrotor experiment:
inject one stopped motor, remove that motor from control allocation, enter Land
mode, release yaw-heading control, and attempt to reach the ground using the
three remaining motors.

The formal matrix was flown on branch
`dev/power-redundancy-protection` at firmware revision
`f5bba53c2a53c0f7d7820341c19e67e6c6f92819`. Raw ULogs are intentionally not
tracked. Derived CSV, JSON, SHA-256 manifests, and PNG figures are included.

## What is on the branch

The experiment is split into reviewable commits:

- `2b24c9f163`: degraded three-motor landing implementation;
- `4911da2819`: control-allocation, failsafe, and MAVSDK test coverage;
- `64d471f428`: matrix execution, ULog extraction, validation, and plotting;
- `10b4c7be7e`: standard MAVLink parameter and command transport;
- `341fe61da9`: same-session SITL recovery support;
- `f5bba53c2a`: fresh visible jMAVSim session for every matrix case.

The implementation is active only in the experiment configuration used here:
quadrotor rotor count, control-allocation failure mode enabled, and actuator
failure action set to Land. It is not Flycore-board-specific and can affect any
quadrotor that enables the same parameters.

One experimental constant remains hard-coded: degraded yaw torque is capped at
`0.15`. It has not been tuned across airframes and is a major reason this
branch must remain SITL-only.

## Get the personal experiment branch

For a new clone:

```sh
git clone https://github.com/amovlgf/PX4-Autopilot.git
cd PX4-Autopilot
git fetch origin dev/power-redundancy-protection
git switch --track -c dev/power-redundancy-protection \
  origin/dev/power-redundancy-protection
git submodule update --init --recursive
```

For exact comparison with the committed matrix artifacts, use the tested
firmware revision:

```sh
git checkout f5bba53c2a53c0f7d7820341c19e67e6c6f92819
```

The final branch tip may contain documentation and generated-result commits
after this tested revision, but no later flight-control change is implied.

## Formal matrix

The matrix contains every combination of:

- stopped motor: 1, 2, 3, 4;
- takeoff altitude: 2.5 m, 10 m, 20 m;
- `COM_FAIL_ACT_T`: 0 s, 5 s.

This produces 24 cases. Before injecting the failure, the runner requires the
vehicle to remain within 0.6 m of the target altitude and below 1 m/s total
speed for two seconds.

Each case uses these key parameters:

```text
SYS_FAILURE_EN=1
FD_ACT_EN=0
MC_AIRMODE=1
CA_FAILURE_MODE=1
CA_ROTOR_COUNT=4
COM_ACT_FAIL_ACT=2
```

`MIS_TAKEOFF_ALT` and `COM_FAIL_ACT_T` are set from the current matrix case.
Failure injection uses `MAV_CMD_INJECT_FAILURE`, motor unit `101`, and failure
type `OFF`.

## Run the matrix

Requirements:

- a Linux desktop session with `gnome-terminal`;
- PX4 jMAVSim dependencies;
- Python 3 with `pymavlink`;
- QGroundControl is optional for observation, but may remain connected.

From the repository root:

```sh
python3 Tools/simulation/single_motor_failure_experiment/run_isolated_matrix.py \
  --fail-fast
```

Every case opens a visible PX4 console and jMAVSim GUI, waits for
`Preflight check: OK`, runs one flight, finalizes its ULog, and closes only
that PX4/jMAVSim session. QGroundControl is left running.

This isolation is important. A three-motor landing can leave the jMAVSim
vehicle tilted, causing later arming checks to depend on the previous impact.
If execution is interrupted after completed cases, continue with:

```sh
python3 Tools/simulation/single_motor_failure_experiment/run_isolated_matrix.py \
  --resume --fail-fast
```

The progress file is `generated/matrix/run_summary.json`. A non-complete last
case is rerun; completed prefix cases are retained.

`run_matrix.py` can also drive one already-running visible SITL session. Use it
for a representative case, not for the full matrix:

```sh
python3 Tools/simulation/single_motor_failure_experiment/run_matrix.py \
  --motors 1 --heights 2.5 --delays 0 --fail-fast
```

The `--force-arm` option exists only to investigate SITL recovery behavior. It
must never be treated as a hardware reproduction step.

## Analyze and validate

Analysis requirements:

- `pyulog`;
- `numpy`;
- `matplotlib`.

After producing the 24 local ULogs:

```sh
bash Tools/simulation/single_motor_failure_experiment/run_analysis.sh

python3 Tools/simulation/single_motor_failure_experiment/validate_matrix.py \
  Tools/simulation/single_motor_failure_experiment/generated/matrix/results.json \
  --expected-sha "$(git rev-parse HEAD)"
```

The extractor recovers the motor number, parameters, branch, firmware SHA, and
injection time from each ULog. It does not infer test conditions from
filenames. The validator requires:

- all 24 unique matrix combinations;
- the expected firmware SHA and unique ULog hashes;
- exact stopped/handled motor masks;
- finite allocator output;
- stopped output for the failed motor;
- allocator and Land-mode response within 0.5 s.

The separate exploratory safety classification additionally requires:

- absolute roll and pitch no greater than 60 degrees;
- down speed no greater than 3 m/s;
- horizontal drift no greater than 5 m;
- a logged landed state.

These thresholds are experiment labels, not hardware safety certification.
Natural disarm and the runner's pre-disarm touchdown observation are reported
separately.

## Formal result at the tested revision

| Takeoff altitude | Cases | Threshold passes | Natural disarms | Forced disarms |
|---|---:|---:|---:|---:|
| 2.5 m | 8 | 8 | 8 | 0 |
| 10 m | 8 | 0 | 8 | 0 |
| 20 m | 8 | 0 | 6 | 2 |
| **Total** | **24** | **8** | **22** | **2** |

Across all 24 ULogs:

- exact handled motor mask: 24/24;
- finite allocator reporting: 24/24;
- stopped failed-motor output: 24/24;
- maximum allocator handling response: 0.196 s;
- maximum Land-mode response: 0.120 s.

The low-altitude result is promising but narrow. Every 10 m and 20 m case
failed at least one dynamics threshold. Observed maxima were 47.98 m/s down
speed, 43.94 m horizontal drift, 179.93 degrees absolute roll, and 89.47
degrees absolute pitch.

The two forced-disarm cases were both Motor 4 at 20 m, with
`COM_FAIL_ACT_T=0 s` and `5 s`. The runner did not observe touchdown within
90 seconds. A later ULog land-detector state after forced disarm does not turn
these into successful powered landings.

## Tracked evidence

- `matrix_logs.txt`: local paths for the formal 24 ULogs; the files themselves
  are ignored;
- `generated/matrix/run_summary.json`: runner observations, including forced
  disarm;
- `generated/matrix/results.csv` and `results.json`: ULog-derived metrics;
- `generated/matrix/logs_manifest.csv`: filename, size, SHA-256, source SHA,
  and matrix identity;
- `generated/plots/`: figures generated from the formal matrix;
- `environment.txt`: pinned source and tool environment;
- `validation.md`: commands, outcomes, and known validation blockers;
- `forum_draft.md`: tracked English PX4 forum article draft;
- `forum_draft_zh.md`: tracked Chinese PX4 forum article draft.

The synchronized English and Chinese forum drafts are tracked so they can be
reviewed and recovered. Other draft variants and raw `.ulg` files remain
outside the Git history.
