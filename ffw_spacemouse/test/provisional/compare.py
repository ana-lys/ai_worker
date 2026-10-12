#!/usr/bin/env python3
"""compare.py <expected.json> < actual.json : recursive compare, numbers within TOL."""
import json, sys

TOL = 2e-3


def same(a, b):
    if isinstance(a, bool) or isinstance(b, bool):
        return a is b
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return abs(a - b) <= TOL
    if isinstance(a, list) and isinstance(b, list):
        return len(a) == len(b) and all(same(x, y) for x, y in zip(a, b))
    if isinstance(a, dict) and isinstance(b, dict):
        return a.keys() == b.keys() and all(same(a[k], b[k]) for k in a)
    return a == b


raw = sys.stdin.read()
try:
    actual = json.loads(raw)
except json.JSONDecodeError:
    print('  no JSON output (harness failed):'); print(raw); sys.exit(1)
expected = json.load(open(sys.argv[1]))
bad = [k for k in sorted(set(expected) | set(actual)) if not same(expected.get(k), actual.get(k))]
for k in bad:
    print(f'  {k}: expected {expected.get(k)} got {actual.get(k)}')
sys.exit(1 if bad else 0)
