# Local Validation Record

Date: 2026-07-29
Branch: `dev/power-redundancy-protection`
Formal matrix firmware:
`f5bba53c2a53c0f7d7820341c19e67e6c6f92819`

## Static and build validation

- `make px4_sitl_default -j2`
  - Passed before the formal matrix.
- `make px4_sitl_default jmavsim`
  - Passed for every fresh visible matrix session at the tested firmware SHA.
- `make check_format`
  - Passed after the matrix and documentation preparation.
- `git diff --check`
  - Passed.
- `python3 -m py_compile
  Tools/simulation/single_motor_failure_experiment/*.py`
  - Passed, including the MAVLink runner, isolated-session runner, extractor,
    validator, and plot generator.
- `bash -n
  Tools/simulation/single_motor_failure_experiment/run_analysis.sh`
  - Passed.

## Runner validation

The first same-session experiments exposed a simulator-state dependency:

1. Two consecutive 2.5 m cases completed.
2. After the second three-motor impact, the vehicle remained at approximately
   16.5 degrees roll and 31.2 degrees pitch.
3. A third takeoff was rejected because preflight checks failed.
4. jMAVSim SPACE reset was not used in the final workflow because resetting the
   vehicle inside a lockstep session stopped reliable time progression.

`run_isolated_matrix.py` was added to start a fresh visible PX4 console and
jMAVSim GUI for each case while leaving QGroundControl running.

An isolated-session smoke matrix covering Motor 1 and Motor 2 at 2.5 m passed:

- each session reached `Preflight check: OK`;
- both cases reached stable hover before injection;
- both produced a ULog and natural disarm;
- exact mask, finite allocation, stopped output, response-time, and dynamics
  criteria passed in both ULogs;
- `--resume` recognized the two completed cases without relaunching them.

## Formal 24-case matrix

Execution command:

```sh
python3 Tools/simulation/single_motor_failure_experiment/run_isolated_matrix.py \
  --fail-fast
```

Matrix:

- motors: 1, 2, 3, 4;
- takeoff altitudes: 2.5 m, 10 m, 20 m;
- `COM_FAIL_ACT_T`: 0 s, 5 s;
- total: 24 cases.

Execution result:

- completed runner cases with ULogs: 24/24;
- missing logs: 0;
- runner errors: 0;
- natural disarms: 22/24;
- forced disarms: 2/24.

Both forced-disarm cases were Motor 4 at 20 m. One used
`COM_FAIL_ACT_T=0 s`; the other used `5 s`. Neither produced a touchdown
observation during the 90-second window. The later ULog land-detector state
after forced disarm is not counted as a successful powered landing.

## ULog analysis and strict validation

Analysis:

```sh
bash Tools/simulation/single_motor_failure_experiment/run_analysis.sh
```

Strict validation:

```sh
python3 Tools/simulation/single_motor_failure_experiment/validate_matrix.py \
  Tools/simulation/single_motor_failure_experiment/generated/matrix/results.json \
  --expected-sha f5bba53c2a53c0f7d7820341c19e67e6c6f92819
```

Validator result:

```json
{
  "errors": [],
  "landed_count": 24,
  "logs": 24,
  "natural_disarm_count": 22,
  "safe_threshold_pass_count": 8
}
```

Source and mechanism checks:

- ULogs with the expected full firmware SHA: 24/24;
- unique matrix combinations: 24/24;
- unique ULog SHA-256 hashes: 24/24;
- exact stopped/handled motor mask: 24/24;
- finite allocator reporting: 24/24;
- failed motor output stopped: 24/24;
- maximum allocator handling response: 0.196 s;
- maximum Land-mode response: 0.120 s;
- response criterion: 0.5 s.

Exploratory dynamics classification:

| Height | Cases | Threshold passes | Natural disarms | Forced disarms |
|---|---:|---:|---:|---:|
| 2.5 m | 8 | 8 | 8 | 0 |
| 10 m | 8 | 0 | 8 | 0 |
| 20 m | 8 | 0 | 6 | 2 |

Observed extrema:

- maximum down speed: 47.980 m/s;
- maximum horizontal drift: 43.940 m;
- maximum absolute roll: 179.935 degrees;
- maximum absolute pitch: 89.469 degrees.

Number of cases exceeding each threshold:

- down speed above 3 m/s: 16;
- horizontal drift above 5 m: 12;
- absolute roll above 60 degrees: 11;
- absolute pitch above 60 degrees: 11;
- allocator response above 0.5 s: 0;
- Land-mode response above 0.5 s: 0.

The safety-threshold result is therefore limited to the tested 2.5 m cases.
Natural disarm at 10 m or 20 m does not imply a safe landing.

## Generated evidence inspection

Regenerated from the formal 24 ULogs:

- `generated/matrix/results.csv`;
- `generated/matrix/results.json`;
- `generated/matrix/logs_manifest.csv`;
- `generated/matrix/run_summary.json`;
- `matrix_pass_heatmap.png`;
- `metrics_by_height.png`;
- `response_delays.png`;
- `representative_comparison.png`;
- `data_flow.png`.

The matrix heatmap, dynamics summary, response-delay scatter, representative
time series, and data-flow diagram were visually inspected after regeneration.
The pre-fix baseline, baseline comparison, raw ULogs, and forum drafts are not
part of the personal-branch evidence commit.

## Blocked by unavailable dependencies

- `make tests TESTFILTER=failsafe_test -j2`
  - Blocked because `test/fuzztest` is not initialized and the execution
    environment cannot update the repository's `.git/config`.
  - No submodule synchronization was performed.
- `make px4_sitl_default mavsdk_tests -j2`
  - Blocked because MAVSDK C++ 3.11.2 is not installed.
  - No package installation was performed.

These are validation gaps. The added unit and MAVSDK test sources compiled only
as part of their unavailable targets and must not be reported as executed.
