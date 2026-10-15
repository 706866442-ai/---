"""Transport-independent v1 packet encoder; no camera/model/pin assumptions.

Example: python vision_packet.py --value 3   # print three V -> C frames
Serial transmission is opt-in with --port; never run against a moving car.
"""
def encode(kind, sequence, value, confidence=90):
    maximum = {1: 4, 2: 2, 3: 3}.get(kind)
    if maximum is None or not 0 <= sequence <= 255 or not 0 <= value <= maximum or not 0 <= confidence <= 100:
        raise ValueError('invalid kind, sequence, value or confidence')
    packet = [0xa5, 1, kind, sequence, value, confidence]
    crc = 0
    for byte in packet[1:]:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ (7 if crc & 128 else 0)) & 255
    return bytes(packet + [crc])

if __name__ == '__main__':
    import argparse
    import time
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--kind', type=int, default=1)
    parser.add_argument('--value', type=int, required=True)
    parser.add_argument('--confidence', type=int, default=90)
    parser.add_argument('--sequence', type=int, default=0)
    parser.add_argument('--count', type=int, default=3)
    parser.add_argument('--port', help='optional COM port; omitted means hex output only')
    args = parser.parse_args()
    if not 1 <= args.count <= 100:
        parser.error('count must be 1..100')
    port = None
    try:
        if args.port:
            import serial
            port = serial.Serial(args.port, 115200, timeout=0.1, write_timeout=0.2)
        for i in range(args.count):
            packet = encode(args.kind, (args.sequence + i) & 255, args.value, args.confidence)
            print(packet.hex(' '))
            if port:
                if port.write(packet) != len(packet):
                    raise IOError('incomplete serial write')
                time.sleep(0.1)
    finally:
        if port:
            port.close()
