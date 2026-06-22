# Octavi IFR-1 native X-Plane 12 plugin (Linux)

PLUGIN  := Octavi
SDK     := sdk/CHeaders/XPLM
XP      ?= /home/rego/.local/share/Steam/steamapps/common/X-Plane 12

CC      := gcc
CFLAGS  := -std=c11 -O2 -Wall -Wextra -fPIC -fvisibility=hidden -m64 \
           -DLIN=1 -DAPL=0 -DIBM=0 \
           -DXPLM200 -DXPLM210 -DXPLM300 -DXPLM301 -DXPLM303 -DXPLM400 \
           -I$(SDK)
# On Linux the XPLM symbols are resolved by X-Plane at load time, so we must
# NOT pass --no-undefined / -z defs and we do NOT link any XPLM library.
LDFLAGS := -shared -fPIC -lm

SRC     := $(wildcard src/*.c)
HDR     := $(wildcard src/*.h)
OUTDIR  := build/$(PLUGIN)/lin_x64
OUT     := $(OUTDIR)/$(PLUGIN).xpl

.PHONY: all clean install sdk probe

all: $(OUT)

# Standalone host-side tester (no X-Plane / SDK needed).
probe: build/octavi_probe
build/octavi_probe: tools/octavi_probe.c src/hidraw.c src/octavi.c src/hidraw.h src/octavi.h
	@mkdir -p build
	$(CC) -std=c11 -O2 -Wall -Wextra tools/octavi_probe.c src/hidraw.c src/octavi.c -o build/octavi_probe -lm
	@echo "Built build/octavi_probe"

$(OUT): $(SRC) $(HDR) | $(SDK)
	@mkdir -p $(OUTDIR)
	$(CC) $(CFLAGS) $(SRC) -o $(OUT) $(LDFLAGS)
	@cp -r profiles build/$(PLUGIN)/
	@echo "Built $(OUT) (+ profiles)"

# Fetch the XPLM SDK headers if missing.
$(SDK):
	@./scripts/fetch-sdk.sh

sdk:
	@./scripts/fetch-sdk.sh

install: all
	@./scripts/install.sh "$(XP)"

clean:
	rm -rf build
