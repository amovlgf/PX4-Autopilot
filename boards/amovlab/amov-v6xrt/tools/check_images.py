#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Validate RT1176 bring-up images without connecting to or writing hardware."""

import argparse
import base64
import hashlib
import json
from pathlib import Path
import struct
import subprocess
import zlib


def require(condition, message):
    if not condition:
        raise ValueError(message)


def u32(data, offset):
    return struct.unpack_from('<I', data, offset)[0]


def symbols(elf, prefix):
    output = subprocess.check_output([prefix + 'nm', '-n', str(elf)], text=True)
    return {parts[2]: int(parts[0], 16) for line in output.splitlines()
            if len(parts := line.split()) == 3 and parts[1] != 'U'}


def check_image(build, label, start, limit, prefix):
    name = 'amovlab_amov-v6xrt_' + label
    directory = build / name
    binary = (directory / (name + '.bin')).read_bytes()
    elf = directory / (name + '.elf')
    sym = symbols(elf, prefix)
    require(0x2008 < len(binary) <= limit, name + ': image size outside partition')
    require(sym['_boot_loadaddr'] == start, name + ': wrong link address')
    require(sym['_boot_size'] == len(binary), name + ': ROM size differs from binary')
    require(sym['_vectors'] == start + 0x2000, name + ': wrong vector address')
    require(sym['_ram_size'] == 0x100000, name + ': OCRAM exceeds dedicated 1 MB')

    # FCB offsets are defined by the RT1176 ROM, independent of compiler layout.
    fcb = binary[0x400:0x600]
    require(len(fcb) == 512 and u32(fcb, 0) == 0x42464346, name + ': invalid FCB tag')
    require(u32(fcb, 4) == 0x56010400, name + ': wrong FCB version')
    require(fcb[0x0c] == 0, name + ': DQS must not be used')
    require(fcb[0x10] == 0 and fcb[0x1c] == 0, name + ': unexpected mode switching')
    require(fcb[0x44:0x47] == bytes([1, 1, 1]), name + ': expected NOR, 1 pad, 30 MHz')
    require(u32(fcb, 0x50) == 0x1000000, name + ': expected 16 MB flash')
    require(u32(fcb, 0x1c0) == 256 and u32(fcb, 0x1c4) == 4096,
            name + ': incorrect program/erase geometry')
    require(fcb[0x1c8] == 1 and u32(fcb, 0x1d0) == 65536,
            name + ': incorrect IP clock/block size')
    # Decode LUT instructions: opcode, pad count exponent, operand.
    instructions = struct.unpack_from('<128H', fcb, 0x80)
    decoded = [(i >> 10, (i >> 8) & 3, i & 0xff) for i in instructions]
    expected = {
        0: [(1, 0, 0x03), (2, 0, 24), (9, 0, 4)],
        1: [(1, 0, 0x05), (9, 0, 4)],
        3: [(1, 0, 0x06)],
        5: [(1, 0, 0x20), (2, 0, 24)],
        8: [(1, 0, 0xd8), (2, 0, 24)],
        9: [(1, 0, 0x02), (2, 0, 24), (8, 0, 4)],
    }
    for seq in range(16):
        want = expected.get(seq, [])
        want = want + [(0, 0, 0)] * (8 - len(want))
        require(decoded[seq * 8:seq * 8 + 8] == want, name + ': wrong LUT sequence ' + str(seq))

    ivt = struct.unpack_from('<8I', binary, 0x1000)
    require(ivt == (0x412000d1, start + 0x2000, 0, 0, start + 0x1020,
                    start + 0x1000, 0, 0), name + ': invalid ROM IVT')
    require(struct.unpack_from('<4I', binary, 0x1020) == (start, len(binary), 0, 0xffffffff),
            name + ': invalid boot data')
    sp, pc = struct.unpack_from('<2I', binary, 0x2000)
    require(0x20240000 < sp <= 0x20340000 and sp % 8 == 0, name + ': invalid initial SP')
    # NuttX's reset trampoline clears LR, then branches to the ELF entry __start.
    require(pc & 1 and (pc & ~1) == (sym['start'] & ~1), name + ': invalid reset vector')
    require(start + 0x2000 <= (pc & ~1) < start + len(binary), name + ': reset outside image')

    # Every loadable file byte must be backed by this NOR partition.
    data = elf.read_bytes()
    header = struct.unpack_from('<16sHHIIIIIHHHHHH', data)
    require(header[0][:6] == b'\x7fELF\x01\x01' and header[2] == 40, name + ': expected ARM ELF32')
    require((header[4] & ~1) == sym['__start'], name + ': incorrect ELF entry')
    for i in range(header[10]):
        segment = struct.unpack_from('<8I', data, header[5] + i * header[9])
        kind, offset, virtual, physical, size, memory_size, flags, alignment = segment
        if kind != 1 or size == 0:
            continue
        require(start <= physical and physical + size <= start + limit, name + ': load outside partition')
        require(binary[physical - start:physical - start + size] == data[offset:offset + size],
                name + ': ELF and BIN differ')
    if label == 'bootloader':
        for function in ['board_flash_initialize', 'ROM_FLEXSPI_NorFlash_Init',
                         'ROM_FLEXSPI_NorFlash_ProgramPage', 'flash_func_erase_sector']:
            require(0x20240000 <= sym[function] < 0x20340000, function + ': must execute from RAM')
    else:
        package = json.loads((directory / (name + '.px4')).read_text())
        payload = zlib.decompress(base64.b64decode(package['image']))
        require(package['board_id'] == 65001 and package['image_maxsize'] == limit,
                name + ': wrong PX4 board ID/partition')
        require(package['image_size'] == len(binary) and payload == binary,
                name + ': PX4 package differs from BIN')
    print(f'{name}: {len(binary)} / {limit} flash bytes, vectors={start + 0x2000:#010x}, '
          f'SP={sp:#010x}, PC={pc:#010x}, SHA256={hashlib.sha256(binary).hexdigest()}')
    return fcb


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build-dir', type=Path, default=Path('build'))
    parser.add_argument('--tool-prefix', default='arm-none-eabi-')
    args = parser.parse_args()
    loader = check_image(args.build_dir, 'bootloader', 0x30000000, 0x20000, args.tool_prefix)
    application = check_image(args.build_dir, 'default', 0x30020000, 0x3e0000, args.tool_prefix)
    require(loader == application, 'Bootloader/application FCBs differ')
    print('PASS: image structure and partition checks; hardware operation is not tested.')


if __name__ == '__main__':
    main()
