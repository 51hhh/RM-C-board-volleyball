#!/usr/bin/env python3
"""
Automated CAN Motor Communication Test
"""

import subprocess
import time
import re
import sys

def run_gdb_command(gdb_process, command, timeout=2):
    """Send command to GDB and read response"""
    gdb_process.stdin.write(f"{command}\n")
    gdb_process.stdin.flush()
    time.sleep(timeout)

def main():
    print("=" * 50)
    print("Automated CAN Motor Communication Test")
    print("=" * 50)
    print()

    # GDB commands script
    gdb_commands = """
target remote localhost:3333
monitor reset halt
load
set pagination off

define print_m3508
    printf "\\n=== M3508 Chassis Motors (CAN1) ===\\n"
    printf "Motor1 [0x201]: ecd=%u, speed=%d rpm, current=%d, temp=%u\\n", motor_chassis[0].ecd, motor_chassis[0].speed_rpm, motor_chassis[0].given_current, motor_chassis[0].temperate
    printf "Motor2 [0x202]: ecd=%u, speed=%d rpm, current=%d, temp=%u\\n", motor_chassis[1].ecd, motor_chassis[1].speed_rpm, motor_chassis[1].given_current, motor_chassis[1].temperate
    printf "Motor3 [0x203]: ecd=%u, speed=%d rpm, current=%d, temp=%u\\n", motor_chassis[2].ecd, motor_chassis[2].speed_rpm, motor_chassis[2].given_current, motor_chassis[2].temperate
    printf "Motor4 [0x204]: ecd=%u, speed=%d rpm, current=%d, temp=%u\\n", motor_chassis[3].ecd, motor_chassis[3].speed_rpm, motor_chassis[3].given_current, motor_chassis[3].temperate
end

define print_dm4340
    printf "\\n=== DM4340 Motors (CAN2) ===\\n"
    printf "DM1 [ID 1]: pos=%.3f, speed=%.3f, current=%.3f\\n", DM4340_Date[0].esc_back_position, DM4340_Date[0].esc_back_speed, DM4340_Date[0].esc_back_current
    printf "DM2 [ID 2]: pos=%.3f, speed=%.3f, current=%.3f\\n", DM4340_Date[1].esc_back_position, DM4340_Date[1].esc_back_speed, DM4340_Date[1].esc_back_current
    printf "DM3 [ID 3]: pos=%.3f, speed=%.3f, current=%.3f\\n", DM4340_Date[2].esc_back_position, DM4340_Date[2].esc_back_speed, DM4340_Date[2].esc_back_current
end

continue
"""

    # Write GDB script
    with open('/tmp/auto_can_test.gdb', 'w') as f:
        f.write(gdb_commands)

    print(">>> Starting GDB and loading firmware...")

    # Start GDB with batch commands
    cmd = [
        'gdb-multiarch',
        '-batch',
        '-nx',
        '-x', '/tmp/auto_can_test.gdb',
        'cmake-build-debug/LED.elf'
    ]

    try:
        # Run GDB in background mode
        result = subprocess.run(
            cmd,
            cwd='/home/rick/desktop/RM-C-board-volleyball',
            capture_output=True,
            text=True,
            timeout=10
        )

        print(result.stdout)
        if result.stderr:
            print("STDERR:", result.stderr)

    except subprocess.TimeoutExpired:
        print(">>> Program running, stopping after 5 seconds...")

    # Now check motor status
    print("\n>>> Checking motor status...")

    check_commands = """
target remote localhost:3333
set pagination off

define print_m3508
    printf "\\n=== M3508 Chassis Motors (CAN1) ===\\n"
    printf "Motor1 [0x201]: ecd=%u, speed=%d rpm, current=%d, temp=%u\\n", motor_chassis[0].ecd, motor_chassis[0].speed_rpm, motor_chassis[0].given_current, motor_chassis[0].temperate
    printf "Motor2 [0x202]: ecd=%u, speed=%d rpm, current=%d, temp=%u\\n", motor_chassis[1].ecd, motor_chassis[1].speed_rpm, motor_chassis[1].given_current, motor_chassis[1].temperate
    printf "Motor3 [0x203]: ecd=%u, speed=%d rpm, current=%d, temp=%u\\n", motor_chassis[2].ecd, motor_chassis[2].speed_rpm, motor_chassis[2].given_current, motor_chassis[2].temperate
    printf "Motor4 [0x204]: ecd=%u, speed=%d rpm, current=%d, temp=%u\\n", motor_chassis[3].ecd, motor_chassis[3].speed_rpm, motor_chassis[3].given_current, motor_chassis[3].temperate
end

define print_dm4340
    printf "\\n=== DM4340 Motors (CAN2) ===\\n"
    printf "DM1 [ID 1]: pos=%.3f, speed=%.3f, current=%.3f\\n", DM4340_Date[0].esc_back_position, DM4340_Date[0].esc_back_speed, DM4340_Date[0].esc_back_current
    printf "DM2 [ID 2]: pos=%.3f, speed=%.3f, current=%.3f\\n", DM4340_Date[1].esc_back_position, DM4340_Date[1].esc_back_speed, DM4340_Date[1].esc_back_current
    printf "DM3 [ID 3]: pos=%.3f, speed=%.3f, current=%.3f\\n", DM4340_Date[2].esc_back_position, DM4340_Date[2].esc_back_speed, DM4340_Date[2].esc_back_current
end

interrupt
print_m3508
print_dm4340
detach
quit
"""

    time.sleep(5)  # Wait for CAN messages

    with open('/tmp/check_motors.gdb', 'w') as f:
        f.write(check_commands)

    cmd_check = [
        'gdb-multiarch',
        '-batch',
        '-nx',
        '-x', '/tmp/check_motors.gdb',
        'cmake-build-debug/LED.elf'
    ]

    result = subprocess.run(
        cmd_check,
        cwd='/home/rick/desktop/RM-C-board-volleyball',
        capture_output=True,
        text=True,
        timeout=10
    )

    print(result.stdout)

    print("\n" + "=" * 50)
    print("Test completed!")
    print("=" * 50)

    # Analyze results
    print("\n>>> Analysis:")

    # Parse M3508 motors
    m3508_pattern = r"Motor\d+ \[0x20\d\]: ecd=(\d+)"
    m3508_matches = re.findall(m3508_pattern, result.stdout)

    if m3508_matches:
        active_m3508 = sum(1 for ecd in m3508_matches if int(ecd) > 0)
        print(f"  M3508 motors responding: {active_m3508}/4")
        if active_m3508 == 0:
            print("  ⚠ No M3508 motors detected on CAN1")

    # Parse DM4340 motors
    dm_pattern = r"DM\d+ \[ID \d\]: pos=([-\d.]+)"
    dm_matches = re.findall(dm_pattern, result.stdout)

    if dm_matches:
        active_dm = sum(1 for pos in dm_matches if abs(float(pos)) > 0.001)
        print(f"  DM4340 motors responding: {active_dm}/3")
        if active_dm == 0:
            print("  ⚠ No DM4340 motors detected on CAN2")

    print()

if __name__ == '__main__':
    main()
