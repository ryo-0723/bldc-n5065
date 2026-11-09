CC ?= gcc
CFLAGS = -O2 -g -std=gnu99 -Wall -Wextra -Werror -ffunction-sections -fdata-sections
CPPFLAGS = -I../utils_math -I. -I../../motor -I../../util -I../..
BUILD = ../../build/tests/hall_pll
HEADERS = ../../motor/foc_speed_source.h ../../motor/foc_math.h ../../datatypes.h
SOURCES = ../../motor/foc_speed_source.c ../../motor/foc_math.c ../../util/utils_math.c
TESTS = source_switch integration pll_angles gain_schedule

.PHONY: run
run: $(addprefix $(BUILD)/,$(TESTS))
	@set -e; for test in $(TESTS); do $(BUILD)/$$test; done

$(BUILD)/%: %.c $(SOURCES) $(HEADERS)
	mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $(CPPFLAGS) $< $(SOURCES) -Wl,--gc-sections -lm -o $@
