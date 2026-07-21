#!/usr/bin/env python3
"""Create a full-flash ICF6 image containing bootloader and application."""

import argparse
from pathlib import Path
import sys


FLASH_BASE = 0x08000000
APPLICATION_OFFSET = 0x20000
FLASH_SIZE = 0x200000


def parse_int(value: str) -> int:
    return int(value, 0)


def read_image(path: Path, name: str) -> bytes:
    if not path.is_file():
        raise ValueError(f"{name} image not found: {path}")

    image = path.read_bytes()

    if not image:
        raise ValueError(f"{name} image is empty: {path}")

    return image


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Combine the ICF6 bootloader and application into a full-flash binary image."
    )
    parser.add_argument("--bootloader", type=Path, required=True)
    parser.add_argument("--application", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--application-offset", type=parse_int, default=APPLICATION_OFFSET)
    parser.add_argument("--flash-size", type=parse_int, default=FLASH_SIZE)
    args = parser.parse_args()

    if args.application_offset <= 0 or args.application_offset >= args.flash_size:
        raise ValueError("application offset must be inside flash")

    bootloader = read_image(args.bootloader, "bootloader")
    application = read_image(args.application, "application")

    if len(bootloader) > args.application_offset:
        raise ValueError(
            f"bootloader size {len(bootloader)} exceeds application offset {args.application_offset}"
        )

    application_max_size = args.flash_size - args.application_offset

    if len(application) > application_max_size:
        raise ValueError(
            f"application size {len(application)} exceeds available flash {application_max_size}"
        )

    padding_size = args.application_offset - len(bootloader)
    combined = bootloader + (b"\xff" * padding_size) + application

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_bytes(combined)

    print(f"Combined image: {args.output}")
    print(f"  bootloader: 0x{FLASH_BASE:08x}, {len(bootloader)} bytes")
    print(f"  padding:    {padding_size} bytes (0xff)")
    print(f"  app:        0x{FLASH_BASE + args.application_offset:08x}, {len(application)} bytes")
    print(f"  image size: {len(combined)} bytes")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except ValueError as error:
        print(f"error: {error}", file=sys.stderr)
        sys.exit(1)
