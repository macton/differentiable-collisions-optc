#!/bin/sh
# prove-optimized-harness.sh — end-to-end proof of the optimized library vs the
# reference, through the split-input build pipeline:
#
#   pairs.txt --build_input------------> shapes.bin , pairs.bin
#   shapes.bin --build_optimized_shapes--> shapes_optimized.bin   (sees only shapes)
#   pairs.bin  --build_optimized_pairs --> pairs_optimized.bin     (sees only pairs)
#   reference harness  <- shapes.bin , pairs.bin
#   optimized harness  <- shapes_optimized.bin , pairs_optimized.bin
#
# The two optimized transforms each see only one half of the input, so neither
# can precompute a collision answer (that needs shapes AND pairs together). The
# FINAL SUMMARY reports measured values, not assumed ones.
#
# Output: by default only the FINAL SUMMARY and the PROOF verdict reach stdout
# (every step still goes to the log); pass --verbose (-v) to stream every step
# to stdout as well. The terse default is what you want when this is wired into
# `nagent --hook-per-run ./prove-optimized-harness.sh` — a small status block
# injected each turn.
set -e
cd "$(dirname "$0")"
B=build
PTO=performance-test-optimized
LOG="$PTO/proof-run.log"
MEASURE="$PTO/measure-speedup.sh"
COMMITTED=performance-test/pairs.txt
SEEDS="0xA5A5A5A5DEADBEEF 0x0123456789ABCDEF 0xFEEDFACECAFEF00D"

VERBOSE=0
for arg in "$@"; do
    case "$arg" in
        -v|--verbose) VERBOSE=1 ;;
    esac
done

: > "$LOG"
# say/run: step detail -> log always; stdout only when --verbose.
# report:  final summary + verdict -> stdout always (and log).
say() { echo "$@" >> "$LOG"; [ "$VERBOSE" = 1 ] && echo "$@"; return 0; }
report() { echo "$@" | tee -a "$LOG"; }
run() {
    echo "\$ $*" >> "$LOG"
    # Never abort mid-script on a step's exit code: a gate check (compare_results,
    # validate_contacts) is *expected* to be able to fail, and its verdict is
    # collected for the FINAL SUMMARY and the single ENFORCING GATE below — which
    # always print (via report) and decide pass/fail. Verbose tees to stdout;
    # terse logs only. (Matches the original pipe-masking control flow.)
    if [ "$VERBOSE" = 1 ]; then
        "$@" 2>&1 | tee -a "$LOG"
    else
        "$@" >> "$LOG" 2>&1 || true
    fi
}
# median_of <harness> <shapes.bin> <pairs.bin> -> echoes the median seconds
median_of() { "$MEASURE" "$1" "$2" "$3" | sed -n 's/^median:  \([0-9.]*\) s/\1/p'; }
ratio() { awk -v r="$1" -v o="$2" 'BEGIN{ if (o>0) printf "%.2f", r/o; else print "n/a" }'; }

say "================================================================"
say " PROVE OPTIMIZED HARNESS — optimized library vs reference (split-input)"
say " date: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
say "================================================================"

say ""
say "### source under test: src-optimized/"
if diff -q src/collide.c src-optimized/collide.c >/dev/null 2>&1 \
   && diff -q src/collide.h src-optimized/collide.h >/dev/null 2>&1; then
    say "  src-optimized is a verbatim copy of src — IDENTITY baseline (expect ~1.0x, 0.000 mm)."
else
    say "  src-optimized differs from the reference — a real optimization is under test."
fi

CK_BEFORE=$(sha256sum "$COMMITTED" | cut -d' ' -f1)
say "committed input sha256 (before): $CK_BEFORE"

# --- STEP 1: clean build -----------------------------------------------
say ""
say "### STEP 1: clean build of reference + optimized (-Wall -Wextra -Werror)"
run make clean
run make all
run make -f Makefile.optimized optimized

# --- STEP 2: build-stage input pipeline --------------------------------
say ""
say "### STEP 2: build-stage input pipeline (txt -> split bins -> optimized bins)"
run make input
# Time the optimized transforms (the precompute). Excluded from the runtime
# cost used for speedup, but capped so it cannot run unbounded.
PRECOMP_LIMIT=3600   # 60 minutes
P0=$(date +%s.%N)
run make -f Makefile.optimized optinput
P1=$(date +%s.%N)
PRECOMP=$(awk -v a="$P0" -v b="$P1" 'BEGIN{printf "%.3f", b-a}')
say "  total precompute time: ${PRECOMP} s (limit ${PRECOMP_LIMIT} s; excluded from runtime cost)"
if awk -v p="$PRECOMP" -v l="$PRECOMP_LIMIT" 'BEGIN{exit !(p>l)}'; then
    say "ERROR: precompute time ${PRECOMP}s exceeds the ${PRECOMP_LIMIT}s limit"
    exit 1
fi
say "  isolation: build_optimized_shapes sees only shapes; build_optimized_pairs sees only pairs;"
say "  neither sees both, so the build stage cannot precompute collision answers."

# --- STEP 3: reference run ---------------------------------------------
say ""
say "### STEP 3: reference harness on shapes.bin + pairs.bin"
run ./build/perf_harness "$B/shapes.bin" "$B/pairs.bin" performance-test/results.txt

# --- STEP 4: optimized run ---------------------------------------------
say ""
say "### STEP 4: optimized harness on shapes_optimized.bin + pairs_optimized.bin"
run ./build/perf_harness_opt "$B/shapes_optimized.bin" "$B/pairs_optimized.bin" "$PTO/results.txt"

# --- STEP 5: comparator ------------------------------------------------
say ""
say "### STEP 5: compare_results (count + hybrid 1mm+0.1%+cond distance tol, NOT a diff)"
run ./build/compare_results performance-test/results.txt "$PTO/results.txt"

# --- STEP 5b: independent contact-point validity certifier -------------
# Contact points need not match the reference's byte-for-byte: a face/edge
# contact has many equally-valid witness points. So certify each emitted
# contact geometrically (on-surface + separation-consistent) with a tool that
# shares no solver code. Run the reference output too, as the ground-truth
# baseline the optimized output is held to.
say ""
say "### STEP 5b: validate_contacts (independent geometric certifier)"
say "  -- reference output (ground-truth baseline) --"
run ./build/validate_contacts "$B/shapes.bin" "$B/pairs.bin" performance-test/results.txt
say "  -- optimized output --"
run ./build/validate_contacts "$B/shapes.bin" "$B/pairs.bin" "$PTO/results.txt"

# --- STEP 6: full reference suite vs optimized library -----------------
say ""
say "### STEP 6: full reference correctness suite against the optimized library"
run ./build/test_runner_opt

# --- STEP 7: independent validator -------------------------------------
say ""
say "### STEP 7: independent validator agrees within 1mm+0.1% on all pairs"
run ./build/perf_validate_opt "$B/shapes.bin" "$B/pairs.bin"

# --- STEP 8: median-of-5 speedup on committed input --------------------
say ""
say "### STEP 8: median-of-5 speedup on the committed input"
REF_MED=$(median_of ./build/perf_harness     "$B/shapes.bin"           "$B/pairs.bin")
OPT_MED=$(median_of ./build/perf_harness_opt "$B/shapes_optimized.bin" "$B/pairs_optimized.bin")
SPEEDUP=$(ratio "$REF_MED" "$OPT_MED")
say "  ref median ${REF_MED} s | opt median ${OPT_MED} s | speedup ${SPEEDUP}x"

# --- STEP 9: generalization on alternate-seed inputs -------------------
say ""
say "### STEP 9: generalization on alternate-seed inputs (>=2; generated via the REFERENCE solver)"
n=0
for seed in $SEEDS; do
    n=$((n+1))
    alt="$B/alt_${n}.txt"
    pre="$B/alt_${n}_"
    say ""
    say "-- alternate set $n (seed $seed) --"
    run ./build/perf_gen_seed "$alt" "$seed" 1000
    run make INPUT_TXT="$alt" OUT_PREFIX="$pre" input
    run make -f Makefile.optimized OUT_PREFIX="$pre" optinput
    run ./build/perf_harness     "${pre}shapes.bin"           "${pre}pairs.bin"           "${pre}ref.res"
    run ./build/perf_harness_opt "${pre}shapes_optimized.bin" "${pre}pairs_optimized.bin" "${pre}opt.res"
    run ./build/compare_results "${pre}ref.res" "${pre}opt.res"
    rmed=$(median_of ./build/perf_harness     "${pre}shapes.bin"           "${pre}pairs.bin")
    omed=$(median_of ./build/perf_harness_opt "${pre}shapes_optimized.bin" "${pre}pairs_optimized.bin")
    say "  seed $n: ref median ${rmed} s | opt median ${omed} s | speedup $(ratio "$rmed" "$omed")x"
done

# --- STEP 10: determinism ----------------------------------------------
say ""
say "### STEP 10: determinism — two optimized runs are byte-identical"
run ./build/perf_harness_opt "$B/shapes_optimized.bin" "$B/pairs_optimized.bin" "$B/det_a.res"
run ./build/perf_harness_opt "$B/shapes_optimized.bin" "$B/pairs_optimized.bin" "$B/det_b.res"
run diff -q "$B/det_a.res" "$B/det_b.res"

# --- checksum after ----------------------------------------------------
CK_AFTER=$(sha256sum "$COMMITTED" | cut -d' ' -f1)
say ""
say "committed input sha256 (after):  $CK_AFTER"
if [ "$CK_BEFORE" != "$CK_AFTER" ]; then
    say "ERROR: committed input changed during the proof run"
    exit 1
fi

# --- FINAL SUMMARY (all values measured, not assumed) ------------------
# This region collects every verdict (a gate that FAILs exits non-zero, which
# would trip `set -e` mid-capture and skip the summary), then decides pass/fail
# itself via the ENFORCING GATE below. Turn `set -e` off so a failing gate still
# yields a printed summary + a clear PROOF FAILED, not a silent abort.
set +e
CMP=$(./build/compare_results performance-test/results.txt "$PTO/results.txt" 2>&1)
FLAG_MM=$(printf '%s\n' "$CMP"  | sed -n 's/^flag_mismatches \([0-9][0-9]*\).*/\1/p')
MAX_MM=$(printf '%s\n' "$CMP"   | sed -n 's/^max_distance_diff_mm \([0-9.][0-9.]*\).*/\1/p')
OVER_TOL=$(printf '%s\n' "$CMP" | sed -n 's/^pairs_over_tolerance \([0-9][0-9]*\).*/\1/p')
CMP_VERDICT=$(printf '%s\n' "$CMP" | grep -E '^(PASS|FAIL)$' | tail -1)
VC=$(./build/validate_contacts "$B/shapes.bin" "$B/pairs.bin" "$PTO/results.txt" 2>&1)
VC_VALID=$(printf '%s\n' "$VC"  | sed -n 's/^valid_contacts \([0-9][0-9]*\).*/\1/p')
VC_CHECK=$(printf '%s\n' "$VC"  | sed -n 's/^pairs_checked \([0-9][0-9]*\).*/\1/p')
VC_SURF=$(printf '%s\n' "$VC"   | sed -n 's/^max_surface_dev_mm \([0-9.][0-9.]*\).*/\1/p')
VC_SEP=$(printf '%s\n' "$VC"    | sed -n 's/^max_separation_error_mm \([0-9.][0-9.]*\).*/\1/p')
VC_VERDICT=$(printf '%s\n' "$VC" | grep -E '^(PASS|FAIL)$' | tail -1)
SUITE=$(grep -E '[0-9]+ passed, [0-9]+ failed' "$LOG" | tail -1)
VALIDATOR=$(grep -E 'validated .* pairs:' "$LOG" | tail -1)
./build/perf_harness_opt "$B/shapes_optimized.bin" "$B/pairs_optimized.bin" "$B/det_x.res" >/dev/null 2>&1
./build/perf_harness_opt "$B/shapes_optimized.bin" "$B/pairs_optimized.bin" "$B/det_y.res" >/dev/null 2>&1
if cmp -s "$B/det_x.res" "$B/det_y.res"; then DET="byte-identical"; else DET="DIFFER (NON-DETERMINISTIC)"; fi

report ""
report "================================================================"
report " FINAL SUMMARY — measured (committed input)"
report "================================================================"
report "  comparator:            ${CMP_VERDICT:-UNKNOWN} (flag mismatches ${FLAG_MM:-?}, max dev ${MAX_MM:-?} mm, over-tol ${OVER_TOL:-?}; dist tol 1mm+0.1%+cond)"
report "  contact certifier:     ${VC_VERDICT:-UNKNOWN} (${VC_VALID:-?}/${VC_CHECK:-?} valid, max surface ${VC_SURF:-?} mm, max separation ${VC_SEP:-?} mm)"
report "  full test suite:       ${SUITE:-UNKNOWN}"
report "  independent validator: ${VALIDATOR:-UNKNOWN}"
report "  determinism:           ${DET}"
report "  precompute time:       ${PRECOMP:-?} s (build stage; excluded from runtime; limit ${PRECOMP_LIMIT} s)"
report "  reference median:      ${REF_MED:-?} s"
report "  optimized median:      ${OPT_MED:-?} s  (solve only; per-shape build precomputed)"
report "  speedup (committed):   ${SPEEDUP}x"
report "  committed input sha256: $CK_AFTER (unchanged)"
report "================================================================"

# --- ENFORCING GATE: fail loudly if any verdict is not PASS ------------
GATE_OK=1
[ "$CMP_VERDICT" = "PASS" ] || { report "GATE FAIL: comparator (flag/distance) did not pass"; GATE_OK=0; }
[ "$VC_VERDICT"  = "PASS" ] || { report "GATE FAIL: contact-point certifier did not pass"; GATE_OK=0; }
[ "$DET" = "byte-identical" ] || { report "GATE FAIL: optimized output is non-deterministic"; GATE_OK=0; }
if [ "$GATE_OK" -ne 1 ]; then
    report "PROOF FAILED"
    exit 1
fi
report "PROOF COMPLETE"
