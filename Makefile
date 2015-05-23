# Toolchain
OBJCOPY=arm-none-eabi-objcopy
OBJDUMP=arm-none-eabi-objdump

# Target
TARGET=lpc17xx

# Files
OUT_DIR=target/$(TARGET)/release
OUT_FILE=$(OUT_DIR)/blink

.PHONY: build clean listing $(OUT_FILE)

all: build listing

build: $(OUT_FILE).bin

clean:
	cargo clean

listing: $(OUT_FILE).lst

# Target is PHONY so cargo can deal with dependencies
$(OUT_FILE):
	cd $(CARGO_ROOT)
	cargo build --release --target=$(TARGET) --verbose

$(OUT_DIR)/%.bin: $(OUT_DIR)/%
	$(OBJCOPY) -O binary $< $@

$(OUT_DIR)/%.lst: $(OUT_DIR)/%
	$(OBJDUMP) -D $< > $@