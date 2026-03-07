#\!/bin/bash
# Usage: ./flash_test.sh [a|b|c|d|normal|test]
# Swaps main.c with the requested test variant and flashes

set -e
cd ~/Desktop/kart_medulla

case "$1" in
    a) SRC="main/main_test_a.c" ;;
    b) SRC="main/main_test_b.c" ;;
    c) SRC="main/main_test_c.c" ;;
    d) SRC="main/main_test_d.c" ;;
    normal) SRC="main/main_normal.c" ;;
    test) SRC="main/test_main.c"
          echo "NOTE: test_main.c replaces app_main — swap it manually"
          exit 1 ;;
    *)
        echo "Usage: $0 [a|b|c|d|normal|test]"
        echo "  a      = PID only, no comms (debug printf)"
        echo "  b      = PID + comms (UART noise test)"
        echo "  c      = Full tasks (reproduce bug)"
        echo "  d      = PID direction check"
        echo "  normal = Production firmware"
        exit 1 ;;
esac

if [ \! -f "$SRC" ]; then
    echo "ERROR: $SRC not found"
    exit 1
fi

# Backup current main.c if not already backed up
if [ \! -f "main/main_backup.c" ]; then
    cp main/main.c main/main_backup.c
    echo "Backed up main.c -> main_backup.c"
fi

cp "$SRC" main/main.c
echo "Copied $SRC -> main/main.c"
echo "Building and flashing..."
~/.local/bin/pio run --target upload --environment esp32dev --upload-port /dev/ttyUSB0
echo "Done\! Open serial monitor: pio device monitor -b 115200 -p /dev/ttyUSB0"
