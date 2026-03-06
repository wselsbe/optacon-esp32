#!/usr/bin/env python3
"""One-time hardware provisioning — write device identity to hw_info NVS partition.

Usage:
    python scripts/provision.py --port /dev/ttyACM0 \
        --serial-number "OPT-2026-0042" \
        --hw-revision "1.2"

    # Arbitrary extra keys:
    python scripts/provision.py --port /dev/ttyACM0 \
        --serial-number "OPT-2026-0042" \
        --hw-revision "1.2" \
        --set batch_date=2026-03-06

Requires: esptool (pip install esptool)
"""
import argparse
import contextlib
import os
import subprocess
import sys
import tempfile


def _find_nvs_gen():
    """Find nvs_partition_gen.py."""
    # Try ESP-IDF installation
    for base in [
        os.path.expanduser("~/esp/v5.5.1"),
        os.environ.get("IDF_PATH", ""),
    ]:
        path = os.path.join(base, "components", "nvs_flash",
                           "nvs_partition_generator", "nvs_partition_gen.py")
        if os.path.exists(path):
            return [sys.executable, path]
    # Fallback: pip-installed
    return [sys.executable, "-m", "esp_idf_nvs_partition_gen"]


def _find_partition_offset(partitions_csv, name):
    """Find the offset of a named partition in the CSV."""
    with open(partitions_csv) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = [p.strip() for p in line.split(",")]
            if len(parts) >= 4 and parts[0] == name:
                return int(parts[3], 0)
    return None


def _find_partition_size(partitions_csv, name):
    """Find the size of a named partition in the CSV."""
    with open(partitions_csv) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = [p.strip() for p in line.split(",")]
            if len(parts) >= 5 and parts[0] == name:
                return int(parts[4], 0)
    return None


def main():
    parser = argparse.ArgumentParser(description="Provision device hardware identity")
    parser.add_argument("--port", required=True, help="Serial port (e.g. /dev/ttyACM0, COM7)")
    parser.add_argument("--serial-number", required=True, help="Device serial number")
    parser.add_argument("--hw-revision", required=True, help="Hardware revision (e.g. 1.2)")
    parser.add_argument("--set", action="append", default=[], metavar="KEY=VALUE",
                        help="Additional key=value pairs")
    parser.add_argument("--partitions", default=None,
                        help="Path to partition CSV (default: config/partitions-4MiB-ota.csv)")
    parser.add_argument("--dry-run", action="store_true",
                        help="Generate binary but don't flash")

    args = parser.parse_args()

    # Build key-value pairs
    kv = {
        "serial_number": args.serial_number,
        "hw_revision": args.hw_revision,
    }
    for extra in args.set:
        if "=" not in extra:
            print(f"Error: --set value must be KEY=VALUE, got: {extra}", file=sys.stderr)
            sys.exit(1)
        k, v = extra.split("=", 1)
        kv[k] = v

    # Find partition table
    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    partitions_csv = args.partitions or os.path.join(repo_root, "config", "partitions-4MiB-ota.csv")

    if not os.path.exists(partitions_csv):
        print(f"Error: partition table not found: {partitions_csv}", file=sys.stderr)
        sys.exit(1)

    offset = _find_partition_offset(partitions_csv, "hw_info")
    size = _find_partition_size(partitions_csv, "hw_info")
    if offset is None or size is None:
        print("Error: 'hw_info' partition not found in partition table", file=sys.stderr)
        sys.exit(1)

    print("Provisioning device:")
    for k, v in kv.items():
        print(f"  {k}: {v}")
    print(f"Partition offset: {hex(offset)}, size: {hex(size)}")

    # Build NVS CSV
    csv_lines = ["key,type,encoding,value", "hw_info,namespace,,"]
    for k, v in kv.items():
        csv_lines.append(f"{k},data,string,{v}")
    csv_content = "\n".join(csv_lines) + "\n"

    # Generate NVS binary
    nvs_gen = _find_nvs_gen()
    with tempfile.NamedTemporaryFile(mode="w", suffix=".csv", delete=False) as f:
        f.write(csv_content)
        csv_path = f.name

    bin_path = csv_path.replace(".csv", ".bin")

    try:
        cmd = nvs_gen + ["generate", csv_path, bin_path, hex(size)]
        print(f"Generating NVS binary: {' '.join(cmd)}")
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode != 0:
            print(f"Error generating NVS binary:\n{result.stderr}", file=sys.stderr)
            sys.exit(1)

        with open(bin_path, "rb") as f:
            nvs_bin = f.read()
        print(f"NVS binary size: {len(nvs_bin)} bytes")

        if args.dry_run:
            out_path = os.path.join(repo_root, "hw_info.bin")
            with open(out_path, "wb") as f:
                f.write(nvs_bin)
            print(f"Dry run — binary saved to {out_path}")
            return

        # Flash to device
        flash_cmd = [
            sys.executable, "-m", "esptool",
            "--port", args.port,
            "--baud", "460800",
            "write_flash", hex(offset), bin_path,
        ]
        print(f"Flashing: {' '.join(flash_cmd)}")
        subprocess.run(flash_cmd, check=True)
        print("Provisioning complete!")

    finally:
        os.unlink(csv_path)
        with contextlib.suppress(FileNotFoundError):
            os.unlink(bin_path)


if __name__ == "__main__":
    main()
