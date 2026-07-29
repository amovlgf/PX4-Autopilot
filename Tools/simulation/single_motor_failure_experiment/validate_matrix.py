#!/usr/bin/env python3
"""Validate matrix coverage, provenance, and degraded-control invariants."""

import argparse
import json
import sys
from itertools import product
from pathlib import Path


EXPECTED_MOTORS = (1, 2, 3, 4)
EXPECTED_HEIGHTS = (2.5, 10.0, 20.0)
EXPECTED_DELAYS = (0.0, 5.0)
MAX_RESPONSE_S = 0.5


def combo(result):
    return (
        int(result["motor_number"]),
        round(float(result["mis_takeoff_alt"]), 3),
        round(float(result["com_fail_act_t"]), 3),
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("results", type=Path)
    parser.add_argument("--expected-sha", required=True)
    args = parser.parse_args()

    document = json.loads(args.results.read_text(encoding="utf-8"))
    results = document["results"]
    expected_combos = set(product(EXPECTED_MOTORS, EXPECTED_HEIGHTS, EXPECTED_DELAYS))
    observed_combos = {
        combo(result) for result in results if result.get("status") == "ok"
    }
    errors = []

    if len(results) != 24:
        errors.append("expected 24 logs, found {}".format(len(results)))

    missing = expected_combos - observed_combos
    duplicate_count = len(results) - len(observed_combos)

    if missing:
        errors.append("missing combinations: {}".format(sorted(missing)))

    if duplicate_count:
        errors.append("{} duplicate or invalid combinations".format(duplicate_count))

    hashes = [result.get("sha256") for result in results]

    if None in hashes or len(set(hashes)) != len(hashes):
        errors.append("ULog hashes are missing or not unique")

    wrong_sha = [
        result.get("file")
        for result in results
        if result.get("software_sha") != args.expected_sha
    ]

    if wrong_sha:
        errors.append("software SHA mismatch: {}".format(wrong_sha))

    invariant_checks = {
        "handled_mask_exact": lambda result: bool(result.get("handled_mask_exact")),
        "allocator_finite": lambda result: bool(result.get("allocator_finite")),
        "stopped_output_valid": lambda result: bool(
            result.get("stopped_output_valid")
        ),
        "handled_response": lambda result: result.get("handled_delay_s") is not None
        and result["handled_delay_s"] <= MAX_RESPONSE_S,
        "land_response": lambda result: result.get("land_delay_s") is not None
        and result["land_delay_s"] <= MAX_RESPONSE_S,
    }

    for name, check in invariant_checks.items():
        failures = [result.get("file") for result in results if not check(result)]

        if failures:
            errors.append("{} failed: {}".format(name, failures))

    summary = document["summary"]
    print(
        json.dumps(
            {
                "logs": len(results),
                "safe_threshold_pass_count": summary[
                    "safe_threshold_pass_count"
                ],
                "natural_disarm_count": summary["natural_disarm_count"],
                "landed_count": sum(bool(result.get("landed")) for result in results),
                "errors": errors,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
