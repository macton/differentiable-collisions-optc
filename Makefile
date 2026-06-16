CC      := gcc
CFLAGS  := -std=c11 -Wall -Wextra -Werror -O3 -march=native -DNDEBUG
LDLIBS  := -lm
B       := build

all: $(B)/test_runner $(B)/perf_gen $(B)/perf_harness $(B)/perf_validate \
     $(B)/build_input

$(B):
	mkdir -p $(B)

$(B)/collide.o: src/collide.c src/collide.h | $(B)
	$(CC) $(CFLAGS) -c src/collide.c -o $@

$(B)/validator.o: test/validator.c test/validator.h src/collide.h | $(B)
	$(CC) $(CFLAGS) -c test/validator.c -o $@

$(B)/pairs_io.o: performance-test/pairs_io.c performance-test/pairs_io.h src/collide.h | $(B)
	$(CC) $(CFLAGS) -c performance-test/pairs_io.c -o $@

$(B)/test_runner: test/test_main.c test/validator.h src/collide.h $(B)/collide.o $(B)/validator.o
	$(CC) $(CFLAGS) test/test_main.c $(B)/collide.o $(B)/validator.o -o $@ $(LDLIBS)

$(B)/perf_gen: performance-test/gen_pairs.c performance-test/pairs_io.h $(B)/collide.o $(B)/pairs_io.o
	$(CC) $(CFLAGS) performance-test/gen_pairs.c $(B)/collide.o $(B)/pairs_io.o -o $@ $(LDLIBS)

$(B)/perf_harness: performance-test/perf_main.c performance-test/pairs_io.h bin_format.h $(B)/collide.o $(B)/pairs_io.o
	$(CC) $(CFLAGS) performance-test/perf_main.c $(B)/collide.o $(B)/pairs_io.o -o $@ $(LDLIBS)

$(B)/perf_validate: performance-test/validate_main.c performance-test/pairs_io.h bin_format.h test/validator.h $(B)/collide.o $(B)/validator.o $(B)/pairs_io.o
	$(CC) $(CFLAGS) performance-test/validate_main.c $(B)/collide.o $(B)/validator.o $(B)/pairs_io.o -o $@ $(LDLIBS)

# Build step: split the text pair set into separate shapes.bin + pairs.bin.
$(B)/build_input: performance-test/build_input.c performance-test/pairs_io.h bin_format.h $(B)/collide.o $(B)/pairs_io.o
	$(CC) $(CFLAGS) performance-test/build_input.c $(B)/collide.o $(B)/pairs_io.o -o $@ $(LDLIBS)

# Produce the split binary input from the committed text set. INPUT_TXT may be
# overridden to split an alternate-seed set; OUT_PREFIX places the outputs.
INPUT_TXT   ?= performance-test/pairs.txt
OUT_PREFIX  ?= $(B)/
.PHONY: input
input: $(B)/build_input
	$(B)/build_input $(INPUT_TXT) $(OUT_PREFIX)shapes.bin $(OUT_PREFIX)pairs.bin

test: $(B)/test_runner
	$(B)/test_runner

clean:
	rm -rf $(B)

.PHONY: all test clean
