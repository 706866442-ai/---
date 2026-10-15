from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from vision_packet import encode
from telemetry_report import parse_lines, report, FIELDS

packet = encode(1, 0, 3, 90)
assert packet[:6] == bytes([0xa5, 1, 1, 0, 3, 90]) and len(packet) == 7
try:
    encode(1, 0, 5)
    raise AssertionError('invalid roman result accepted')
except ValueError:
    pass
values = [0] * len(FIELDS)
values[FIELDS.index('gray_valid')] = 1
values[FIELDS.index('gray')] = 0xe7
values[FIELDS.index('ms')] = 0xfffffff0
first = 'T,' + ','.join(map(str, values))
values[0] = 84
second = 'T,' + ','.join(map(str, values))
rows, rejected = parse_lines(['BOOT', first, 'T,broken', second])
assert len(rows) == 2 and rejected == 1 and '100 ms' in report(rows, rejected)
print('PASS: packet encoder validation, telemetry parsing, corrupted rows, clock wrap')
