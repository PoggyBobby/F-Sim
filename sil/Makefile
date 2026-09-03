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

VCU   ?= SRE-VCU
CC    ?= cc
BUILD  = build
BIN    = $(BUILD)/vcu_sil

# -Ihost comes first so host/APDB.h shadows the firmware's (see that file)
CFLAGS = -std=gnu11 -O0 -g -w -include host/host_types.h \
         -Ihost -I$(VCU)/inc -I$(SRC)
LDLIBS = -lm

VCU_SRCS  = $(wildcard $(VCU)/dev/*.c)
HOST_SRCS = $(wildcard host/*.c)
PATCHES   = $(wildcard patches/*.patch)
SRC       = $(BUILD)/src
OBJS = $(patsubst $(VCU)/dev/%.c,$(BUILD)/vcu/%.o,$(VCU_SRCS)) \
       $(patsubst host/%.c,$(BUILD)/host/%.o,$(HOST_SRCS))

all: $(BIN)

$(BIN): $(OBJS)
	$(CC) -o $@ $^ $(LDLIBS)

# patched copy of the firmware sources
$(SRC)/.patched: $(VCU_SRCS) $(wildcard $(VCU)/dev/*.h) $(PATCHES)
	rm -rf $(SRC) && mkdir -p $(SRC)
	cp $(VCU)/dev/*.c $(VCU)/dev/*.h $(SRC)/
	for p in $(PATCHES); do patch -s -d $(SRC) -p1 < $$p || exit 1; done
	touch $@

$(BUILD)/vcu/%.o: $(SRC)/.patched | $(BUILD)/vcu
	$(CC) $(CFLAGS) -c -o $@ $(SRC)/$*.c

$(BUILD)/host/%.o: host/%.c | $(BUILD)/host
	$(CC) $(CFLAGS) -c -o $@ $<

$(BUILD)/vcu $(BUILD)/host:
	mkdir -p $@

clean:
	rm -rf $(BUILD)

.PHONY: all clean
