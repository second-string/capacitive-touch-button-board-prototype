OPENOCD_BOARD_FILE="board/esp32s3-builtin.cfg"
TARGET=cap_touch_button_board

build/$(TARGET).elf:
	idf.py build

# Currently bootloader and partition table must be flashed at least once with idf.py before this!
.PHONY: flash
flash:
	openocd  -f $(OPENOCD_BOARD_FILE) -c "program_esp build/$(TARGET).bin 0x10000 verify exit"

# flash erase_sector <bank id> <first sector> [<last sector>|'last']
.PHONY: erase_flash
erase_flash:
	openocd  -f $(OPENOCD_BOARD_FILE) -c "flash init; init; flash erase_sector 0 1 last; exit"

# Currently bootloader and partition table must be flashed at least once with idf.py before this!
.PHONY: debug
debug: build/$(TARGET).elf
	openocd  -f $(OPENOCD_BOARD_FILE) -c "program_esp build/$(TARGET).bin 0x10000 verify exit"
	openocd  -f $(OPENOCD_BOARD_FILE) > /dev/null 2>&1 &
	TERM=xterm-256color; xtensa-esp32-elf-gdb build/$(TARGET).elf -x .gdbinit -tui && pkill openocd

# Won't work with default tmux value of TERM=screen-256color
.PHONY: gdb
gdb:
	TERM=xterm-256color; xtensa-esp32-elf-gdb build/$(TARGET).elf -x .gdbinit -tui

.PHONY: jtag_log
jtag_log:
	./jtag_log.expect

# Make sure $IDF_PATH defined if 'No such file or directory' error seen
.PHONY: parse_jtag_log
parse_jtag_log:
	$(IDF_PATH)/tools/esp_app_trace/logtrace_proc.py jtag_log_output.hex build/$(TARGET).elf
