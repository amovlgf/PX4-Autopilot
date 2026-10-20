#!/usr/bin/env python3
"""Run one interactive PX4/jMAVSim session and clean up its own descendants."""

import os
import signal
import subprocess
import sys
import termios
import time


def main():
    if len(sys.argv) < 2:
        raise SystemExit("usage: run_jmavsim_sitl.py <px4-executable> [px4-arguments...]")

    terminal_settings = termios.tcgetattr(sys.stdin) if sys.stdin.isatty() else None
    # PX4's startup script launches Java. A separate session keeps those
    # descendants together even if the startup shell exits before Java does.
    # Inherited descriptors retain the interactive PX4 console.
    child = subprocess.Popen(sys.argv[1:], start_new_session=True)

    def signal_session(signum):
        try:
            os.killpg(child.pid, signum)
            return True
        except ProcessLookupError:
            return False

    def forward_signal(signum, _frame):
        signal_session(signum)

    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, forward_signal)

    try:
        result = child.wait()
        return result if result >= 0 else 128 - result
    finally:
        # Never search for Java/PX4 by name: this process group belongs only
        # to the instance started above. Other simulator sessions are untouched.
        if signal_session(signal.SIGTERM):
            deadline = time.monotonic() + 2
            while time.monotonic() < deadline and signal_session(0):
                time.sleep(0.05)
            signal_session(signal.SIGKILL)
        if terminal_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, terminal_settings)


if __name__ == "__main__":
    sys.exit(main())
