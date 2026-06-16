#!/bin/sh
# measure-speedup.sh — median-of-5 timing for ONE harness on a split-bin input.
#
# WSL2 is noisy; a single run is not evidence. The harness is run 5 times
# back-to-back on the same input; the "total collision time" line is parsed and
# the 5 samples plus their median are printed.
#
# Usage:
#   measure-speedup.sh <harness> <shapes.bin> <pairs.bin>
#
# Harness contract: <harness> <shapes.bin> <pairs.bin> <results.txt>
#
# Reference and optimized harnesses read DIFFERENT inputs (reference: the plain
# split bins; optimized: the *_optimized.bin), so speedup is computed by the
# caller from two separate invocations of this script.
set -e
cd "$(dirname "$0")/.."   # project root (script lives in performance-test-optimized/)

B=build

if [ "$#" -ne 3 ]; then
    echo "usage: measure-speedup.sh <harness> <shapes.bin> <pairs.bin>" >&2
    exit 2
fi

harness="$1"; shapes="$2"; pairs="$3"
tag=$(basename "$harness")
res="$B/measure_${tag}.res"

samples=""
for _ in 1 2 3 4 5; do
    s=$("$harness" "$shapes" "$pairs" "$res" \
        | sed -n 's/^total collision time: \([0-9.]*\) s.*/\1/p')
    if [ -z "$s" ]; then
        echo "measure-speedup: failed to parse collision time from $harness" >&2
        exit 2
    fi
    samples="$samples $s"
done
median=$(printf '%s\n' $samples | sort -n | sed -n '3p')
echo "harness: $harness"
echo "samples:$samples s"
echo "median:  $median s"
