#!/usr/bin/env python3
"""Build the portable codec tests and verify byte-identical repository protocol mirrors."""
import argparse
import subprocess
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mirror', action='append', type=Path, default=[])
    parser.add_argument('--cxx', default='c++')
    parser.add_argument('--sanitize', action='store_true')
    args = parser.parse_args()
    for mirror in args.mirror:
        for name in ('management_protocol.h', 'test_codec.cpp', 'test_protocol.py',
                     'dsdl/amov/transport/200.DeviceManagement.uavcan', 'status_digest_vector.json'):
            if (HERE / name).read_bytes() != (mirror / name).read_bytes():
                raise SystemExit('Protocol mirror mismatch: ' + str(mirror / name))
    with tempfile.TemporaryDirectory(prefix='amov-device-codec-') as temporary:
        binary = str(Path(temporary) / 'test_codec')
        command = [args.cxx, '-std=c++11', '-Wall', '-Wextra', '-Werror', '-pedantic',
                   str(HERE / 'test_codec.cpp'), '-o', binary]
        if args.sanitize:
            command += ['-fsanitize=address,undefined', '-fno-omit-frame-pointer']
        subprocess.run(command, check=True)
        subprocess.run([binary], check=True)


if __name__ == '__main__':
    main()
