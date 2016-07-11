# Toolchain
OBJCOPY=arm-none-eabi-objcopy
OBJDUMP=arm-none-eabi-objdump
LOADER=teensy_loader_cli

# Target
TARGET=thumbv7em-none-eabi

# Files
OUT_DIR=target/$(TARGET)/release
OUT_FILE=$(OUT_DIR)/dotstar

# Rust version
RUSTV=nightly-2016-06-09

.PHONY: build clean listing load realclean debug

all: build listing
build: $(OUT_FILE).hex
listing: $(OUT_FILE).lst $(OUT_FILE).S

$(OUT_FILE): $(wildcard *.rs Cargo.*)
	RTC_TIME=$(shell date "+%s") rustup run $(RUSTV) cargo build --release --target=$(TARGET) --verbose
	arm-none-eabi-size -A $@

debug:
	rustup run $(RUSTV) cargo build --target=$(TARGET) --verbose
	$(OBJDUMP) -S target/$(TARGET)/debug/dotstar > cli.S

$(OUT_DIR)/%.hex: $(OUT_DIR)/%
	$(OBJCOPY) -O ihex $< $@

$(OUT_DIR)/%.lst: $(OUT_DIR)/%
	$(OBJDUMP) -D $< > $@

$(OUT_DIR)/%.S: $(OUT_DIR)/%
	$(OBJDUMP) -S $< > $@

clean:
	rm $(OUT_FILE)

realclean:
	cargo clean

load: $(OUT_FILE).hex
	$(LOADER) -s --mcu=mk20dx256 -v $<
