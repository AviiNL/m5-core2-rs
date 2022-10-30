# Run Cargo Build
all: build flash

build:
	cargo build --release

flash:
	espflash COM4 target/xtensa-esp32-none-elf/release/m5_core2 --monitor

