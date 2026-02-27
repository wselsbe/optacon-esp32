"""Test script to compare machine.bootloader() vs pz_actuator.enter_bootloader()
reliability. Requires the board to be running on a COM port and the
siglent-spd power supply to be accessible for power cycling."""

import subprocess
import sys
import time

import serial
import serial.tools.list_ports

MPY_VID = 0x303A
MPY_PID = 0x4001
BOOT_PID = 0x1001

def find_port(target_pid):
    """Find COM port with given PID."""
    for p in serial.tools.list_ports.comports():
        if p.vid == MPY_VID and p.pid == target_pid:
            return p.device
    return None

def wait_for_port(target_pid, timeout=10):
    """Wait for a port with given PID to appear."""
    start = time.time()
    while time.time() - start < timeout:
        port = find_port(target_pid)
        if port:
            return port
        time.sleep(0.5)
    return None

def send_bootloader_command(port, command):
    """Send a bootloader command via raw REPL and return quickly."""
    try:
        s = serial.Serial(port, 115200, timeout=2)
        # mpremote-style: set DTR/RTS false before any comms
        s.dtr = False
        s.rts = False
        time.sleep(0.1)

        # Enter raw REPL
        s.write(b'\r\x03\x03')  # interrupt
        time.sleep(0.1)
        s.read(s.in_waiting)  # flush
        s.write(b'\x01')  # ctrl-A (raw REPL)
        time.sleep(0.3)
        s.read(s.in_waiting)  # flush prompt

        # Send command
        cmd = command.encode() + b'\x04'  # ctrl-D to execute
        s.write(cmd)
        time.sleep(0.3)

        s.close()
        return True
    except Exception as e:
        print(f"  Error sending command: {e}")
        return False

def power_cycle():
    """Power cycle via siglent-spd MCP (using curl to the MCP)."""
    # We'll use subprocess to call a simple Python snippet
    subprocess.run([sys.executable, "-c", """
import socket, json, time

# Simple approach: just toggle GPIO or use system command
# For now, we'll skip power cycling and just wait for the board to reboot
"""], timeout=5, capture_output=True)

def test_bootloader(method, num_trials=5):
    """Test bootloader entry reliability.
    method: 'machine' or 'pz_actuator'
    """
    if method == 'machine':
        cmd = 'import machine; machine.bootloader()'
    else:
        cmd = 'import pz_actuator; pz_actuator.enter_bootloader()'

    results = []

    for trial in range(num_trials):
        # Find the MicroPython port
        port = wait_for_port(MPY_PID, timeout=8)
        if not port:
            print(f"  Trial {trial+1}: SKIP - no MicroPython port found")
            results.append('skip')
            continue

        print(f"  Trial {trial+1}: Found {port}, sending '{method}' bootloader command...")
        time.sleep(0.5)  # Let USB settle

        send_bootloader_command(port, cmd)

        # Wait and check result
        time.sleep(3)

        boot_port = find_port(BOOT_PID)
        mpy_port = find_port(MPY_PID)

        if boot_port:
            print(f"  Trial {trial+1}: PASS - bootloader on {boot_port}")
            results.append('pass')

            # Use esptool to reset back to app (hard reset via RTS)
            import contextlib
            with contextlib.suppress(Exception):
                subprocess.run(
                    [sys.executable, "-m", "esptool",
                     "--chip", "esp32s3", "--port", boot_port,
                     "chip-id"],
                    timeout=10, capture_output=True
                )
            time.sleep(3)

        elif mpy_port:
            print(f"  Trial {trial+1}: FAIL - still MicroPython on {mpy_port}")
            results.append('fail')
        else:
            print(f"  Trial {trial+1}: UNKNOWN - no port found")
            results.append('unknown')

        time.sleep(1)

    return results


if __name__ == '__main__':
    method = sys.argv[1] if len(sys.argv) > 1 else 'both'
    trials = int(sys.argv[2]) if len(sys.argv) > 2 else 5

    if method in ('machine', 'both'):
        print(f"\n=== Testing machine.bootloader() ({trials} trials) ===")
        machine_results = test_bootloader('machine', trials)
        passes = machine_results.count('pass')
        fails = machine_results.count('fail')
        print(f"  Results: {passes} pass, {fails} fail, {len(machine_results)-passes-fails} other")

    if method in ('pz_actuator', 'both'):
        print(f"\n=== Testing pz_actuator.enter_bootloader() ({trials} trials) ===")
        pz_results = test_bootloader('pz_actuator', trials)
        passes = pz_results.count('pass')
        fails = pz_results.count('fail')
        print(f"  Results: {passes} pass, {fails} fail, {len(pz_results)-passes-fails} other")
