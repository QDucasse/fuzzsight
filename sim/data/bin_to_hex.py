import struct
import sys

with open(sys.argv[1], 'rb') as f:
    data = f.read()

while len(data) % 4:
    data += b'\x00'

for i in range(0, len(data), 4):
    word = struct.unpack_from('<I', data, i)[0]  # little-endian 32-bit
    print(f'{word:08x}')