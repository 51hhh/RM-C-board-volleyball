#!/bin/bash
# CAN Motor Communication Test Script

cd /home/rick/desktop/RM-C-board-volleyball

echo "========================================="
echo "CAN Motor Communication Test"
echo "========================================="
echo ""

# 检查 OpenOCD 是否运行
if ! pgrep -x openocd > /dev/null; then
    echo "Starting OpenOCD server..."
    openocd -f config/daplink.cfg > /tmp/openocd.log 2>&1 &
    sleep 2
    echo "OpenOCD started (PID: $!)"
else
    echo "OpenOCD is already running"
fi

echo ""
echo "Starting GDB session..."
echo ""
echo "Available GDB commands:"
echo "  print_all_motors    - Show all motor status"
echo "  print_m3508_motors  - Show M3508 chassis motors (CAN1)"
echo "  print_dm4340_motors - Show DM4340 motors (CAN2)"
echo "  monitor_can         - Start continuous monitoring"
echo "  c                   - Continue execution"
echo "  Ctrl+C              - Pause execution"
echo ""
echo "========================================="
echo ""

gdb-multiarch -q cmake-build-debug/LED.elf -x /tmp/gdb_can_test.gdb
