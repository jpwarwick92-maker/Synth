# Synth firmware: build, test and assemble the browser prototype.
#
#   make test       build and run the native engine tests
#   make wasm       build the firmware as WebAssembly
#   make site       assemble the playable page in build/site
#   make serve      serve build/site on http://localhost:8080
#   make page-test  run the page tests in a headless browser
#   make docs       regenerate docs/controls.md from the firmware's table
#   make all        test + site + page-test

FW_SRC      := $(wildcard firmware/*.cpp)
FW_HDR      := $(wildcard firmware/*.h)
BUILD_LABEL ?= dev

CXX        ?= g++
WASM_CXX   ?= clang++
CXXFLAGS   := -std=c++17 -O2 -Wall -Wextra -Wshadow -fno-exceptions -fno-rtti
WASM_FLAGS := --target=wasm32 -std=c++17 -O2 -ffreestanding -nostdlib \
              -fno-exceptions -fno-rtti -fno-threadsafe-statics -fvisibility=hidden -Wall -Wextra
WASM_LD    := -Wl,--no-entry -Wl,--strip-all -Wl,-z,stack-size=65536

.PHONY: all test wasm site serve page-test docs

all: test site page-test

# ---- native tests ----
build/tests: $(FW_SRC) $(FW_HDR) $(wildcard tests/native/*.cpp) $(wildcard tests/native/*.h)
	@mkdir -p build
	$(CXX) $(CXXFLAGS) -Ifirmware $(FW_SRC) tests/native/*.cpp -o $@ \
	    -Wl,--wrap=malloc -Wl,--wrap=calloc -Wl,--wrap=realloc

test: build/tests
	@mkdir -p build/out
	build/tests build/out
	@build/tests --controls | diff -u docs/controls.md - \
	    || (echo "docs/controls.md is out of date: run 'make docs'"; exit 1)

docs: build/tests
	build/tests --controls > docs/controls.md

# ---- WebAssembly ----
build/firmware.wasm: $(FW_SRC) $(FW_HDR) $(wildcard targets/web/*.cpp)
	@mkdir -p build
	$(WASM_CXX) $(WASM_FLAGS) -fno-builtin -c targets/web/libc_min.cpp -o build/libc_min.o
	$(WASM_CXX) $(WASM_FLAGS) $(FW_SRC) targets/web/wasm_api.cpp build/libc_min.o -o $@ $(WASM_LD)

wasm: build/firmware.wasm

# ---- the page ----
# Files are copied over the top of any previous build.
site: build/firmware.wasm
	@mkdir -p build/site/layouts
	cp -r web/. build/site/
	cp layouts/*.json build/site/layouts/
	cp build/firmware.wasm build/site/
	@printf '{"build":"%s","date":"%s"}\n' "$(BUILD_LABEL)" "$$(date -u +%Y-%m-%d)" > build/site/build.json
	@echo "site ready in build/site"

serve: site
	python3 -m http.server 8080 -d build/site

page-test: site
	node tests/page/run.mjs build/site
