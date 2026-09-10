CC ?= cc
CFLAGS ?= -O3 -std=c11 -Wall -Wextra -Wpedantic -pthread
CPPFLAGS ?=
LDLIBS ?= -lasound -lm -pthread

.PHONY: all clean test

all: realaudio_bridge_s32_c

realaudio_bridge_s32_c: realaudio_bridge_s32.c
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ $< $(LDLIBS)

test: realaudio_bridge_s32_c
	python3 test_bridge_c_faithfulness.py

clean:
	rm -f realaudio_bridge_s32_c
