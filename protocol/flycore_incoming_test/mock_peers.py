#!/usr/bin/env python3
"""Simulate the 8550 station and/or the 6490 link responder."""

import argparse
import sqlite3
import time
from contextlib import ExitStack


class Peer:
    def __init__(self, mavlink, port, sysid, compid, flight_id, respond_to_ping=False):
        self.port = port
        self.codec = mavlink.MAVLink(port, srcSystem=sysid, srcComponent=compid)
        self.codec.robust_parsing = True
        self.flight_id = flight_id
        self.address = sysid, compid
        self.respond_to_ping = respond_to_ping

    def receive(self):
        pending = self.port.read(self.port.in_waiting)
        for message in self.codec.parse_buffer(pending) or []:
            if message.get_type() == "BAD_DATA" or message.get_msgbuf()[0] != 0xFD:
                continue
            if (message.get_srcSystem(), message.get_srcComponent()) != self.flight_id:
                continue

            if message.get_type() == "PING":
                if self.respond_to_ping and (message.target_system, message.target_component) == (0, 0):
                    self.codec.ping_send(message.time_usec, message.seq, *self.flight_id)
            elif (getattr(message, "target_system", 0), getattr(message, "target_component", 0)) == self.address:
                yield message


def save_report(db, board, result):
    full_mask = 0x7FFF
    if result.executed_mask & ~full_mask or result.passed_mask & ~result.executed_mask or result.overall_pass != (
        result.executed_mask == result.passed_mask == full_mask
    ):
        raise ValueError("inconsistent execution, pass masks or overall result")

    with db:
        db.execute("INSERT INTO inspections VALUES (?, ?, ?, ?)",
                   (board, result.executed_mask, result.passed_mask, result.overall_pass))
    print(
        f"{'PASS' if result.overall_pass else 'FAIL'}, "
        f"executed=0x{result.executed_mask:04x} passed=0x{result.passed_mask:04x}",
        flush=True,
    )


def station_loop(args, mavlink, station, responder, db):
    deadline = time.monotonic() + args.timeout

    station.codec.flycore_test_start_send(*station.flight_id)
    while time.monotonic() < deadline:
        if responder:
            for _ in responder.receive():
                pass

        for message in station.receive():
            if message.get_type() == "FLYCORE_TEST_STATUS":
                phase = mavlink.enums["FLYCORE_TEST_PHASE"][message.phase].name
                print(f"{phase}, item={message.item}", flush=True)
                if message.phase == mavlink.FLYCORE_TEST_PHASE_BUSY:
                    return 2

            elif message.get_type() == "FLYCORE_TEST_RESULT":
                save_report(db, args.board_key, message)
                print("UART3 bidirectional communication passed", flush=True)
                return 0 if message.overall_pass else 1

        time.sleep(0.01)

    print("Timed out waiting for the inspection result", flush=True)
    return 2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--station", help="Serial or PTY path connected to the flight controller UART3")
    parser.add_argument("--responder", help="Serial or PTY path connected to the flight controller UART5")
    parser.add_argument("--station-baud", type=int, default=115200)
    parser.add_argument("--responder-baud", type=int, default=115200)
    parser.add_argument("--station-system", type=int, default=2)
    parser.add_argument("--responder-system", type=int, default=3)
    parser.add_argument("--station-component", type=int, default=191)
    parser.add_argument("--responder-component", type=int, default=191)
    parser.add_argument("--board-key", help="Unique flight-controller identifier used to store station results")
    parser.add_argument("--database", default="flycore-inspection.sqlite")
    parser.add_argument("--timeout", type=float, default=90)
    args = parser.parse_args()
    if not (args.station or args.responder):
        parser.error("pass --station, --responder, or both")
    if args.station and not args.board_key:
        parser.error("--station requires --board-key")
    if args.station and args.responder and args.station == args.responder:
        parser.error("use separate serial ports for UART3 and UART5")
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    for identifier in (args.station_system, args.station_component, args.responder_system, args.responder_component):
        if not 1 <= identifier <= 255:
            parser.error("MAVLink peer IDs must be in the range 1–255")

    import flycore_test as mavlink
    import serial

    with ExitStack() as resources:
        responder = None
        if args.responder:
            port = resources.enter_context(serial.Serial(args.responder, args.responder_baud, timeout=0, write_timeout=1))
            responder = Peer(mavlink, port, args.responder_system, args.responder_component, (1, 1), respond_to_ping=True)

        if args.station:
            port = resources.enter_context(serial.Serial(args.station, args.station_baud, timeout=0, write_timeout=1))
            station = Peer(mavlink, port, args.station_system, args.station_component, (1, 1))
            db = sqlite3.connect(args.database)
            resources.callback(db.close)
            db.execute(
                "CREATE TABLE IF NOT EXISTS inspections (board TEXT, "
                "executed_mask INTEGER, passed_mask INTEGER, overall_pass INTEGER)"
            )
            db.commit()
            return station_loop(args, mavlink, station, responder, db)

        try:
            while True:
                for _ in responder.receive():
                    pass
                time.sleep(0.01)
        except KeyboardInterrupt:
            return 0


if __name__ == "__main__":
    raise SystemExit(main())
