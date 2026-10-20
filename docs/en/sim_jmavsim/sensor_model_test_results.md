# jMAVSim Sensor Model Validation

Validated on 2026-10-19 on Linux with OpenJDK 13.0.7 and Ant 1.10.7.
Branch: `codex/position-source-switch-test`, based on PX4 `7293640e6ac84ca6274af7a26a9d4438053a8e8a`.
The implementation changes were uncommitted during these tests.
jMAVSim is pinned to `665b26d62d36a33b6bf01c5931c589beb812c6a2`, and jMAVlib to `c08380767b21938391bec676a43b36b837af9ba4`.

## Build And Model Checks

| Check | Result |
| --- | --- |
| Linux `px4_sitl_default` build from an initially absent build directory | Passed |
| Incremental PX4 rebuild after bridge changes | Passed |
| Clean Java staging, patch application and compilation in the build directory | Passed; 78 Java source files compiled |
| Incremental `build_jmavsim` with unchanged inputs | Passed; no work required |
| `test_jmavsim_sensors` | Passed: production model selection, geometry, covariance, delays, rates, independent random streams and MAVLink 2 extensions |
| C++ sensor-control and odometry GoogleTests | All 5 passed |
| Changed C/C++ formatting, Python syntax and `git diff --check` | Passed |
| Upstream jMAVSim and jMAVlib working trees | Clean |

Java boundary checks include due-north heading encoding, unavailable heading with a vertical antenna baseline, tilted slant range, upward/out-of-range beams, 200 ms GNSS delay and 40 ms visual delay.
The production multicopter factory is exercised directly so that selecting the old sensor model fails the test.

## Runtime Inputs And Controls

The visible jMAVSim aircraft window, a separate PX4 terminal and the local QGroundControl Debug build were launched.
Automated console tests used MAVLink `SERIAL_CONTROL`, the same transport as QGC's MAVLink Console; individual QGC UI clicks were not automated.

| Observation | Observed Result |
| --- | --- |
| IMU1 and IMU2 | Both update at 250 Hz; no third active IMU |
| Barometer and magnetometer | One active input each |
| GNSS | One input at approximately 20 Hz; RTK fixed (`fix_type=6`) |
| GNSS uncertainties | Horizontal 0.02 m, vertical 0.03 m, speed 0.05 m/s |
| Dual-antenna heading | Approximately 20 Hz through `sensor_gnss_relative` and `vehicle_gnss_heading` |
| GNSS position and heading delay | Both processed timestamps were exactly 200000 microseconds before publication |
| Downward range | One input at approximately 20 Hz; body-down orientation and 0.1–30 m limits |
| Visual odometry | One input at 25 Hz; NED position/velocity, positive finite covariance, quality 100 |
| Visual delay configuration | `EKF2_EV_DELAY=40`; Java delayed-sample timing test passed |

Every input was disabled and restored independently, with stopped and resumed publication confirmed by topic timestamps.
Both IMU1-only and IMU2-only operation retained a live clock and console.
GNSS position and heading continued independently when the other input was disabled.
The console rejected disabling the last available IMU; direct IMU masks `0`, `4` and `8` rolled back to the preceding valid value.
Disabling all six non-IMU input categories together also kept the simulation running.
Reset restored masks `3` and `63`.

## Source Profiles And Flights

Each profile was configured on the ground, saved, and checked after restarting the same instance.
All six source-selection parameters persisted, while the sensor masks returned to `3` and `63`.
Checks required matching EKF control flags, fresh accepted `estimator_aid_src_*` observations and valid local position/velocity.

| Profile | Accepted Fusion | Validation |
| --- | --- | --- |
| RTK default | GNSS position, velocity, height and heading | Takeoff, hover, landing and disarming passed |
| Vision | Visual position, velocity, height and heading | Takeoff, hover, landing and disarming passed |
| RTK plus vision | GNSS position/velocity/height/heading plus visual position/velocity | Ground checks passed |
| Barometer and magnetic heading | GNSS position/velocity, barometric height and magnetic heading | Ground checks passed |
| Range height | Visual position/velocity/heading and range height | Ground checks passed |

The two flights used normal arming, local offboard setpoints at 3 m above the initial estimated position, an 8-second wall-clock hover check, and normal landing.
During the sampled hover, maximum estimated-position deviations from the commanded setpoint were:

| Profile | Horizontal | Vertical |
| --- | ---: | ---: |
| RTK | 0.087 m | 0.135 m |
| Vision | 0.053 m | 0.206 m |

These are tracking deviations in the estimated frame, not independent measurements of absolute positioning accuracy.
Range-height fusion on the ground can use PX4's minimum-range fallback; this test does not establish flight performance using range as the height reference.
Tilted and invalid range geometry was checked in the Java model tests.
No in-flight source reconfiguration or new automatic source-selection strategy was tested.

The final running session was restored to the RTK default profile with all inputs enabled and the vehicle disarmed.
See [the usage guide](sensor_model.md) for configuration blocks and repeatable test commands.
