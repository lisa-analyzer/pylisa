#!/usr/bin/env bash
#
# Compares the tests that failed in the last Gradle run with a reference list.
#
# Usage: scripts/compare-test-failures.sh <reference-list> [<junit-results-dir>]
#
# The reference list has one failing test per line, as "<TestClass> <method()>",
# optionally followed by " | <message>"; lines starting with "total" are ignored.
# The JUnit XML results default to build/test-results/test.
#
# Prints the tests that fail now but not in the reference ("new") and those that
# failed in the reference but pass now ("fixed"). Exits with 1 if there is any new
# failure, 0 otherwise.

set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
	echo "usage: $0 <reference-list> [<junit-results-dir>]" >&2
	exit 2
fi

reference="$1"
results="${2:-build/test-results/test}"

if [[ ! -d "$results" ]]; then
	echo "no JUnit results in $results: run the tests first" >&2
	exit 2
fi

current="$(mktemp)"
expected="$(mktemp)"
trap 'rm -f "$current" "$expected"' EXIT

python3 - "$results" > "$current" <<'EOF'
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

for report in sorted(Path(sys.argv[1]).glob("*.xml")):
    for case in ET.parse(report).getroot().iter("testcase"):
        if case.find("failure") is not None or case.find("error") is not None:
            print(case.get("classname").rsplit(".", 1)[-1], case.get("name"))
EOF
sort -u -o "$current" "$current"

grep -v '^total' "$reference" | sed 's/ | .*$//' | sort -u > "$expected"

new_failures="$(comm -23 "$current" "$expected")"
fixed="$(comm -13 "$current" "$expected")"

echo "failing now: $(wc -l < "$current"), in reference: $(wc -l < "$expected")"
if [[ -n "$fixed" ]]; then
	echo "fixed (failed in the reference, pass now):"
	echo "$fixed" | sed 's/^/  /'
fi
if [[ -n "$new_failures" ]]; then
	echo "new failures:"
	echo "$new_failures" | sed 's/^/  /'
	exit 1
fi
echo "no new failures"
