#!/usr/bin/env python3
# Parse controller debug output and extract joint angles to CSV.
# Usage:
#   ./build/Controller 2>&1 | python3 parse_angles.py > angles.csv
#   python3 parse_angles.py < recorded_output.txt > angles.csv

import re
import sys

PATTERN = re.compile(
    r'\[DEBUG\] A: ([^;]+); N: [^;]+; T: (.+)'
)

print("i,A1,A2,A3,T1,T2,T3")
for i, line in enumerate(sys.stdin):
    m = PATTERN.search(line)
    if m:
        a = m.group(1).replace(' ', '')
        t = m.group(2).replace(' ', '')
        print(f"{i},{a},{t}")
