# AMOV RID/UOM protocol

The XML, generator and protocol tests are shared byte-for-byte by PX4-Autopilot,
QGroundControl and ArduRemoteID. Generated headers remain build artifacts; do not
edit upstream MAVLink definitions or commit generated headers.

## Version negotiation and compatibility

A ground station creates a random nonzero connection `session_id` and requests
identity with `protocol_version=2`. The response must match the requesting
system/component, session and nonzero request ID, return `OK`, version 2 and
both `LOCAL_READINESS_V2` and `FLIGHT_CONTEXT` capability bits. Capabilities in
an unsolicited, failed or version 1 response do not authorize version 2 use.
A session cannot change its negotiated version; use a new session instead.
Identity invalidation, autopilot reboot or connection replacement requires a
new session. Status messages must match the session, autopilot boot, RID boot,
identity revision/digest and source, and advance the uint32 sequence (including
wrap). Old-session messages cannot refresh readiness.

Version 1 requests and status retain their existing ACK-based arming semantics.
New ground stations must clearly identify an old endpoint; they must not invent
an ACK or silently claim version 2 readiness. Extensions preserve the original
MAVLink minimum lengths and CRCs: identity 224/250, UOM status 72/206.

## Local preparation and platform state

Version 2 `AMOV_UOM_STATUS` adds `local_ready`, `provider_state`, `local_error`
and `last_sample_age_ms`. Before arming, production PX4 requires official
configuration, matching activated identity, fresh status (3 s), identity /
activation / telemetry flags, `local_ready=1`, `local_error=0`, and telemetry
sample age plus transport age no greater than 3 s. Local readiness includes
credentials, writable storage, the reporting scheduler and valid prerequisites.
Mock status is accepted only in a dedicated test firmware configuration.

Platform state and its existing `error_code` / `last_ack_age_ms` are independent.
A missing or failed platform ACK cannot block version 2 local preparation.
After arming, `UNAVAILABLE` and `PENDING` mean waiting to run or waiting within
the first actual transmission's confirmation grace period; they do not require
an ACK and are not treated as accepted reports. `FAILED` warns immediately;
`ACCEPTED` requires a successful ACK no older than 5 s, including transport age.
The ground station starts its missing-confirmation timer at actual transmission,
not at arming or queue creation. Loss of local prerequisites still warns
immediately. These warnings do not select a flight failsafe action.
An ACK never compensates for missing local prerequisites.

## Flight status

`AMOV_UOM_FLIGHT_STATUS` (61005) is emitted at 5 Hz for the negotiated owner.
Validate sender, target, version 2, boot/session and advancing `time_boot_us`,
and expire received context independently of telemetry and identity.

- `flight_sequence` is boot-local and zero before the first actual takeoff.
  Each armed transition from confirmed landed to airborne increments it.
  Confirmed landing ends the flight; another takeoff increments it even if the
  vehicle has not disarmed. Reconnecting a ground station does not change it.
- `takeoff_utc_us` is latched at the takeoff event from a fresh GNSS UTC sample
  and that sample's monotonic timestamp. If unavailable then, it remains zero
  for that flight. A subsequent UTC fix must never reconstruct it.
- `ACTIVE=1`, `AIRBORNE=2`, `TAKEOFF_UTC_VALID=4` describe the flight. Sequence
  and known UTC remain available after landing for final records.
- `BROADCAST_READY=8` means the bound activated RID's independent DroneCAN
  self-check is successful and no older than 3 s. It contains no UOM decision.
- `SIMULATION=16` identifies a PX4 SITL build. Official reporting is forbidden
  for SITL, HIL, replay and mock sources. Absence of this flag is not proof of
  a manufacturer-approved aircraft: production profiles still require the
  manufacturer-bound product identity and matching authorized serial.

## Validation

Run `test_protocol.py` with the PX4 and QGC pinned upstream definition and
pymavlink paths, and pass each other repository's `protocol/amov_rid_uom`
directory with `--mirror`. It generates both dialects, exchanges real frames,
checks unmodified legacy CRC/minimum lengths and zero-extension frames, rejects
corruption, and verifies mirrors without modifying upstream definitions.
PX4 regression targets are `unit-RidState`, `unit-RidFlightContext`,
`functional-CommanderRemoteID` and `functional-RidBenchPolicy`.
