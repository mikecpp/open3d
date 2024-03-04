import sys
import time
from ply_to_boxsize import ply_to_boxsize

def measure(filename):
    (width, length, depth) = ply_to_boxsize(filename)
    print("Filename: ", filename)
    print(f"Width:  {width:.0f}mm  ({185 - width:.0f}mm)")
    print(f"Length: {length:.0f}mm  ({300 - length:.0f}mm)")
    print(f"Depth:  {depth:.0f}mm  ({210 - depth:.0f}mm)")

for i in range(10):
    time_start = time.time() 
    measure(sys.argv[1])
    time_end = time.time()
    print(f"time elapse: {(time_end - time_start)*1000:.0f} ms")
