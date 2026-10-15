# Host build of the SRE-VCU firmware for software-in-the-loop.
#
#   make -C sil            -> sil/build/vcu_sil
#   make -C sil clean
#
# Compiles the firmware sources in SRE-VCU/dev against the real TTTech
# headers in SRE-VCU/inc, with the XC2000 types replaced by fixed-width
# host types (host/host_types.h, pre-included) and the IO library replaced
# by host/io_stubs.c. Firmware warnings are silenced (-w): they are the
# team's to fix in their own build, not here.
#
# The sources are copied to build/src and the patches in patches/ applied
# there first — the submodule itself is never modified. Each patch is a
# firmware bug the host build cannot live with; the goal is zero patches.
#
# Two binaries come out:
#   build/vcu_sil       the firmware as flashed — main.c calls SDiff_new(TRUE)
#   build/vcu_sil_open  identical except SDiff_new(FALSE): the firmware's own
#                       s-diff switch turned off, so both rear motors get the
#                       same command. This is the "open diff" config.

VCU   ?= SRE-VCU
CC    ?= cc
BUILD  = build
BIN    = $(BUILD)/vcu_sil
BIN_OPEN = $(BUILD)/vcu_sil_open

# -Ihost comes first so host/APDB.h shadows the firmware's (see that file)
#
# The two -Wno-error flags: GCC 14 promoted incompatible-pointer-types and
# int-conversion from warnings to errors, and -w does not downgrade an error.
# The firmware trips both (e.g. initializations.c passes a ubyte2* where
# IO_DI_Get wants a bool*). They are the team's to fix in their own build, not
# ours — same rationale as -w. Drop these once upstream is clean.
CFLAGS = -std=gnu11 -O0 -g -w -include host/host_types.h \
         -Wno-error=incompatible-pointer-types -Wno-error=int-conversion \
         -Ihost -I$(VCU)/inc -I$(SRC)
LDLIBS = -lm

VCU_SRCS  = $(wildcard $(VCU)/dev/*.c)
HOST_SRCS = $(wildcard host/*.c)
PATCHES   = $(wildcard patches/*.patch)
SRC       = $(BUILD)/src
SRC_OPEN  = $(BUILD)/src_open
HOST_OBJS = $(patsubst host/%.c,$(BUILD)/host/%.o,$(HOST_SRCS))
OBJS = $(patsubst $(VCU)/dev/%.c,$(BUILD)/vcu/%.o,$(VCU_SRCS)) $(HOST_OBJS)
OBJS_OPEN = $(patsubst $(VCU)/dev/%.c,$(BUILD)/vcu_open/%.o,$(VCU_SRCS)) $(HOST_OBJS)

all: $(BIN) $(BIN_OPEN)

$(BIN): $(OBJS)
	$(CC) -o $@ $^ $(LDLIBS)

$(BIN_OPEN): $(OBJS_OPEN)
	$(CC) -o $@ $^ $(LDLIBS)

# patched copy of the firmware sources
$(SRC)/.patched: $(VCU_SRCS) $(wildcard $(VCU)/dev/*.h) $(PATCHES)
	rm -rf $(SRC) && mkdir -p $(SRC)
	cp $(VCU)/dev/*.c $(VCU)/dev/*.h $(SRC)/
	for p in $(PATCHES); do patch -s -d $(SRC) -p1 < $$p || exit 1; done
	touch $@

$(BUILD)/vcu/%.o: $(SRC)/.patched | $(BUILD)/vcu
	$(CC) $(CFLAGS) -c -o $@ $(SRC)/$*.c

# the open-diff copy: same patched sources, s-diff switch off in main.c.
# Fails loudly if main.c stops calling SDiff_new(TRUE), rather than silently
# building a second copy of the s-diff.
$(SRC_OPEN)/.patched: $(SRC)/.patched
	rm -rf $(SRC_OPEN) && mkdir -p $(SRC_OPEN)
	cp $(SRC)/*.c $(SRC)/*.h $(SRC_OPEN)/
	grep -q 'SDiff_new(TRUE)' $(SRC_OPEN)/main.c || \
	    { echo "main.c no longer calls SDiff_new(TRUE): update the open-diff build in sil/Makefile"; exit 1; }
	sed -i.bak 's/SDiff_new(TRUE)/SDiff_new(FALSE)/' $(SRC_OPEN)/main.c && rm $(SRC_OPEN)/main.c.bak
	touch $@

$(BUILD)/vcu_open/%.o: $(SRC_OPEN)/.patched | $(BUILD)/vcu_open
	$(CC) $(CFLAGS) -c -o $@ $(SRC_OPEN)/$*.c

# io_stubs.c includes the firmware's own sensorCalculations.h for the SAS
# calibration, so the patched copy has to exist before the host objects build.
$(BUILD)/host/%.o: host/%.c $(SRC)/.patched | $(BUILD)/host
	$(CC) $(CFLAGS) -c -o $@ $<

$(BUILD)/vcu $(BUILD)/vcu_open $(BUILD)/host:
	mkdir -p $@

clean:
	rm -rf $(BUILD)

.PHONY: all clean
