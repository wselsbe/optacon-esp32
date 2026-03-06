#!/bin/bash
# Check what's holding a listening port on the Pi and optionally kill it.
# Runs on the Pi via: ssh optacon-pi 'bash -s' < pi-check-tunnel-port.sh PORT [--kill]
# Output: PID IP (one line), or empty if port is free.

PORT="$1"
KILL="$2"

if [ -z "$PORT" ]; then
    echo "Usage: $0 PORT [--kill]" >&2
    exit 1
fi

# Find the PID holding the listening port (needs sudo for process info)
LINE=$(sudo ss -tlnp "sport = :$PORT" 2>/dev/null | grep -v '^State')
if [ -z "$LINE" ]; then
    exit 0  # port is free
fi

PID=$(echo "$LINE" | grep -oP 'pid=\K[0-9]+' | head -1)
if [ -z "$PID" ]; then
    exit 0
fi

# Find the remote IP of the SSH connection for this PID
REMOTE_IP=$(sudo ss -tnp 2>/dev/null | grep "pid=$PID" | grep ':22 ' | grep -oP '\d+\.\d+\.\d+\.\d+(?=:\d+\s*$)' | head -1)

echo "$PID $REMOTE_IP"

if [ "$KILL" = "--kill" ] && [ -n "$PID" ]; then
    sudo kill "$PID" 2>/dev/null
    echo "KILLED"
fi
