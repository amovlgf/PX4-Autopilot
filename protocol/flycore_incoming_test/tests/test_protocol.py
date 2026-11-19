#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Build the Flycore-Q service and runner on Linux and test their serial protocol."""

import errno
import importlib
import os
import pty
import select
import sqlite3
import subprocess
import sys
import tempfile
import time
import tty
import unittest
from collections import deque
from contextlib import closing
from pathlib import Path


TESTS = Path(__file__).resolve().parent
PROTOCOL = TESTS.parent
REPO = PROTOCOL.parents[1]
FLIGHT = (1, 1)
ALL_ITEMS = 0x7FFF
UART5 = 1 << 5
SD = 1 << 14


class SerialPeer:
    def __init__(self, mavlink, system):
        self.master, self.slave = pty.openpty()
        tty.setraw(self.slave)
        self.path = os.ttyname(self.slave)
        self.system = system
        self.codec = mavlink.MAVLink(self, srcSystem=system, srcComponent=191)
        self.incoming = deque()

    def close(self):
        os.close(self.master)
        os.close(self.slave)

    def write(self, data):
        return os.write(self.master, data)

    def read(self, deadline):
        while not self.incoming:
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not select.select([self.master], [], [], remaining)[0]:
                return None
            self.incoming.extend(self.codec.parse_buffer(os.read(self.master, 4096)) or ())
        return self.incoming.popleft()


class ProtocolTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        temporary = tempfile.TemporaryDirectory(prefix="flycore-protocol-")
        cls.addClassCleanup(temporary.cleanup)
        cls.work = Path(temporary.name)
        cls.mavlink, cls.service = build_service(cls.work)

    def connect(self, fail_sd=False):
        self.station = self.add_peer(2)
        self.responder = self.add_peer(3)
        command = [self.service]
        if fail_sd:
            command.append("--fail-sd")
        env = dict(os.environ, FLYCORE_TEST_STATION=self.station.path, FLYCORE_TEST_RESPONDER=self.responder.path)
        self.firmware = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, env=env)
        self.addCleanup(stop_service, self.firmware)
        self.local_reply("READY")

    def add_peer(self, system):
        peer = SerialPeer(self.mavlink, system)
        self.addCleanup(peer.close)
        return peer

    def expect(self, peer, kind, phase=None, timeout=2):
        deadline = time.monotonic() + timeout
        while True:
            frame = peer.read(deadline)
            self.assertIsNotNone(frame, f"Expected {kind} phase={phase} on peer {peer.system}")
            self.assertEqual(frame.get_msgbuf()[0], 0xFD)
            self.assertEqual((frame.get_srcSystem(), frame.get_srcComponent()), FLIGHT)
            if frame.get_type() == "FLYCORE_TEST_STATUS":
                self.assertEqual((frame.target_system, frame.target_component), (2, 191))
                self.assertIn(frame.phase, (0, 1, 2))
                if frame.phase == 1:
                    self.assertIn(frame.item, range(15))
                else:
                    self.assertEqual(frame.item, 255)
                if kind != frame.get_type() or phase != frame.phase:
                    continue

            self.assertEqual(frame.get_type(), kind)
            return frame

    def quiet(self, peer, duration=0.15):
        self.assertIsNone(peer.read(time.monotonic() + duration), f"Unexpected frame on peer {peer.system}")

    def begin(self):
        self.station.codec.flycore_test_start_send(*FLIGHT)
        self.expect(self.station, "FLYCORE_TEST_STATUS", phase=0)

    def ping(self):
        message = self.expect(self.responder, "PING")
        self.assertEqual((message.target_system, message.target_component), (0, 0))
        return message

    def answer(self, message):
        self.responder.codec.ping_send(message.time_usec, message.seq, *FLIGHT)

    def report(self, failed):
        message = self.expect(self.station, "FLYCORE_TEST_RESULT")
        self.assertEqual((message.target_system, message.target_component), (2, 191))
        self.assertEqual((message.overall_pass, message.executed_mask, message.passed_mask),
                         (int(not failed), ALL_ITEMS, ALL_ITEMS & ~failed))

    def local_command(self, command):
        self.firmware.stdin.write(command)
        self.firmware.stdin.flush()

    def local_reply(self, expected):
        self.assertTrue(select.select([self.firmware.stdout], [], [], 2)[0], f"Expected local reply {expected}")
        self.assertEqual(self.firmware.stdout.readline().decode().strip(), expected)

    def test_reject_invalid_start(self):
        self.connect()
        self.station.codec.flycore_test_start_send(9, 1)
        self.station.codec.flycore_test_start_send(1, 9)
        self.mavlink.MAVLink(self.station, srcSystem=9, srcComponent=191).flycore_test_start_send(*FLIGHT)
        self.mavlink.MAVLink(self.station, srcSystem=2, srcComponent=9).flycore_test_start_send(*FLIGHT)
        self.mavlink.MAVLink(self.responder, srcSystem=2, srcComponent=191).flycore_test_start_send(*FLIGHT)
        invalid = bytearray(self.station.codec.flycore_test_start_encode(*FLIGHT).pack(self.station.codec))
        invalid[-1] ^= 1
        self.station.write(invalid)
        self.quiet(self.station)
        self.quiet(self.responder)
        self.begin()
        self.answer(self.ping())
        self.report(0)

    def test_one_report_and_fresh_retest(self):
        self.connect()
        for _ in range(2):
            self.begin()
            self.answer(self.ping())
            self.report(0)
            self.quiet(self.station, 1.2)

    def test_sd_failure(self):
        self.connect(fail_sd=True)
        self.begin()
        self.answer(self.ping())
        self.report(SD)

    def test_uart5_timeout(self):
        self.check_timeout(fail_sd=False)
        self.begin()
        self.answer(self.ping())
        self.report(0)

    def test_uart5_and_sd_failure(self):
        self.check_timeout(fail_sd=True)

    def check_timeout(self, fail_sd):
        self.connect(fail_sd=fail_sd)
        self.begin()
        attempts = [self.ping() for _ in range(3)]
        for previous, current in zip(attempts, attempts[1:]):
            self.assertEqual(current.seq, previous.seq + 1)
            self.assertGreaterEqual(current.time_usec - previous.time_usec, 1_000_000)
        self.report(UART5 | (SD if fail_sd else 0))
        self.quiet(self.responder)

    def test_invalid_and_late_ping_replies(self):
        self.connect()
        self.begin()
        first = self.ping()
        self.responder.codec.ping_send(first.time_usec, first.seq + 1, *FLIGHT)
        self.responder.codec.ping_send(first.time_usec + 1, first.seq, *FLIGHT)
        self.responder.codec.ping_send(first.time_usec, first.seq, 9, 1)
        wrong_source = self.mavlink.MAVLink(self.responder, srcSystem=9, srcComponent=191)
        wrong_source.ping_send(first.time_usec, first.seq, *FLIGHT)
        second = self.ping()
        self.assertNotEqual(first.seq, second.seq)
        self.answer(first)
        self.answer(self.ping())
        self.report(0)
        self.quiet(self.responder)

    def test_busy_and_local_exclusion(self):
        self.connect()
        self.local_command(b"L")
        self.local_reply("LOCAL STARTED")
        self.station.codec.flycore_test_start_send(*FLIGHT)
        self.expect(self.station, "FLYCORE_TEST_STATUS", phase=2)
        self.quiet(self.station)
        self.local_command(b"R")
        self.local_reply("LOCAL RESULT 0")

        self.begin()
        waiting = self.ping()
        self.station.codec.flycore_test_start_send(*FLIGHT)
        self.expect(self.station, "FLYCORE_TEST_STATUS", phase=2)
        self.local_command(b"L")
        self.local_reply(f"LOCAL RESULT {-errno.EBUSY}")
        self.answer(waiting)
        self.report(0)
        self.quiet(self.station)
        self.quiet(self.responder)

    def test_station_times_out_without_valid_result(self):
        isolated_station = self.add_peer(2)
        database = self.work / "disconnected.sqlite"
        outcome = subprocess.run(
            [sys.executable, PROTOCOL / "mock_peers.py", "--station", isolated_station.path,
             "--board-key", "disconnected", "--database", database, "--timeout", "0.2"],
            env=dict(os.environ, PYTHONPATH=str(self.work)), capture_output=True, text=True, timeout=5,
        )
        self.assertEqual(outcome.returncode, 2, outcome.stderr)
        self.assertIn("Timed out waiting for the inspection result", outcome.stdout)
        self.assertNotIn("UART3 bidirectional communication passed", outcome.stdout)
        with closing(sqlite3.connect(database)) as connection:
            self.assertEqual(connection.execute("SELECT COUNT(*) FROM inspections").fetchone(), (0,))


def stop_service(process):
    process.terminate()
    try:
        process.communicate(timeout=2)
    except subprocess.TimeoutExpired:
        process.kill()
        process.communicate()


def build_service(work):
    mavlink = REPO / "src/modules/mavlink/mavlink"
    generator = [sys.executable, PROTOCOL / "generate.py", "--pymavlink-root", mavlink,
                 "--definitions", mavlink / "message_definitions/v1.0"]
    include = work / "include"
    headers = (
        "drivers/drv_hrt.h", "parameters/param.h", "px4_platform_common/defines.h",
        "px4_platform_common/log.h", "px4_platform_common/posix.h", "px4_platform_common/px4_config.h",
        "px4_platform_common/sem.h", "px4_platform_common/tasks.h",
    )
    # Adapt the PX4 include paths to one Linux test shim without changing the production sources.
    for name in headers:
        header = include / name
        header.parent.mkdir(parents=True, exist_ok=True)
        header.symlink_to(TESTS / "host_px4.h")

    subprocess.run([*generator, "--output", work / "c"], check=True)
    subprocess.run([*generator, "--language", "Python3", "--output", work / "flycore_test.py"], check=True)
    hw_test = REPO / "src/systemcmds/hw_test"
    service = work / "firmware-service"
    subprocess.run(
        [os.environ.get("CXX", "g++"), "-std=c++14", "-Wall", "-Wextra", "-Werror", "-pthread",
         "-Wl,--wrap=open", "-I", include, "-I", hw_test, "-isystem", work / "c",
         TESTS / "host_px4.cpp", hw_test / "hw_test_flycore_mavlink.cpp", hw_test / "hw_test_runner.cpp",
         "-o", service],
        check=True,
    )
    sys.path.insert(0, str(work))
    try:
        return importlib.import_module("flycore_test"), service
    finally:
        sys.path.pop(0)


if __name__ == "__main__":
    unittest.main()
