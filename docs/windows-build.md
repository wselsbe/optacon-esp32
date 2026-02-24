# Building on Windows (without Docker)

This documents the patches and workflow needed to build the MicroPython firmware
natively on Windows using the Espressif ESP-IDF toolchain.

## Prerequisites

1. **ESP-IDF v5.5.1** — install via the [Espressif Windows Installer](https://dl.espressif.com/dl/esp-idf/)
   (installs to `C:\Espressif` by default)
2. **GNU Make** — `choco install make` or via MSYS2/Git Bash
3. **Git Bash** — comes with [Git for Windows](https://gitforwindows.org/)
4. **MicroPython v1.27.0** — cloned to `C:\Projects\Optacon\micropython`
5. **mpy-cross** — pre-built at `micropython/mpy-cross/build/mpy-cross.exe`

## Quick Start

From Git Bash:

```bash
MSYS_NO_PATHCONV=1 cmd.exe /C "C:\Projects\Optacon\optacon-firmware\run-build.bat"
```

Or open an **ESP-IDF 5.5 CMD** prompt and run:

```cmd
cd C:\Projects\Optacon\optacon-firmware
build-local.cmd
```

Output binaries land in `build/`:
- `bootloader.bin` @ 0x0
- `partition-table.bin` @ 0x8000
- `micropython.bin` @ 0x10000

## Required Patches

Building MicroPython on Windows from Git Bash requires patches to three codebases:
ESP-IDF, MicroPython, and optacon-firmware itself. The root cause of most issues is
that Git Bash sets the `MSYSTEM` environment variable, which ESP-IDF treats as a
hard blocker.

### 1. ESP-IDF Patches (in `C:\Espressif\frameworks\esp-idf-v5.5.1`)

These are local edits to the installed ESP-IDF. They'll need to be re-applied if
ESP-IDF is reinstalled or upgraded.

#### `tools/idf.py` — MSYSTEM warning silently skips build

**Problem:** When `MSYSTEM` is in the environment, `idf.py` prints a warning but
falls through the `if/elif/else` without ever calling `main()`. The build appears
to run but produces no output — `make` then fails on the post-build `makeimg.py`
step because no binaries exist.

**Fix:** Add `main()` call after the warning:

```python
# Before (line ~853):
if 'MSYSTEM' in os.environ:
    print_warning('MSys/Mingw is no longer supported...')
elif os.name == 'posix' and not _valid_unicode_config():
    ...
else:
    main()

# After:
if 'MSYSTEM' in os.environ:
    print_warning('MSys/Mingw is no longer supported...')
    main()  # patched: continue building despite MSYSTEM warning
elif os.name == 'posix' and not _valid_unicode_config():
    ...
else:
    main()
```

#### `tools/idf_tools.py` — hard exit on MSYSTEM

**Problem:** `idf_tools.py` checks `'MSYSTEM' in os.environ` at the `__main__`
entry point and calls `sys.exit(1)`. This blocks `export.bat` from setting up the
environment, and blocks `idf_tools.py install` from installing missing tools.

**Fix:** Remove or comment out the MSYSTEM check (lines ~3599-3605):

```python
# Before:
if __name__ == '__main__':
    if 'MSYSTEM' in os.environ:
        fatal('MSys/Mingw is not supported...')
        raise SystemExit(1)
    main(sys.argv[1:])

# After:
if __name__ == '__main__':
    main(sys.argv[1:])
```

#### `export.bat` — refuses to run when MSYSTEM defined

**Problem:** Line 2 of `export.bat` checks `if defined MSYSTEM` and exits
immediately. Even CMD launched from Git Bash inherits `MSYSTEM` in its environment
block, and CMD's `set MSYSTEM=` doesn't truly unset it (just sets it to empty,
which `if defined` still matches).

**Fix:** Remove or comment out the check (lines 2-5):

```batch
@rem Before:
if defined MSYSTEM (
    echo This .bat file is for Windows CMD.EXE shell only.
    goto :eof
)

@rem After:
REM MSYSTEM check removed — allow building from Git Bash via cmd.exe wrapper
```

### 2. MicroPython Patches (in `C:\Projects\Optacon\micropython`)

These are tracked as local modifications to the MicroPython source tree.

#### `py/mkrules.cmake` — response file for compiler flags (Windows CMD line limit)

**Problem:** The qstr generation step constructs a GCC command line containing all
`MICROPY_CPP_FLAGS` (~94,000 characters). Windows CMD has an 8,191-character limit,
so the command is silently truncated and GCC fails.

**Fix** (based on [MicroPython PR #17226](https://github.com/micropython/micropython/pull/17226)):
Write compiler flags to a response file and pass `@path/to/file` to GCC.

Key changes:
- Write flags to `genhdr/qstr.i.last.in` via `file(GENERATE ...)`
- Escape double quotes (`\"`) in the response file so GCC preserves them literally
  (flags like `-DFFCONF_H="oofatfs/ffconf.h"` lose their quotes otherwise)
- Pass `@${MICROPY_QSTRDEFS_LAST_IN}` instead of raw flags to `makeqstrdefs.py`

#### `py/preprocess_qstr.py` — cross-platform qstr preprocessor (new file)

**Problem:** The `qstrdefs.preprocessed.h` generation uses a Unix shell pipeline:
```
cat $files | sed 's/^Q(.*)/\"&\"/' | $CC -E $CFLAGS - | sed 's/^\"(Q(.*))"/\1/' > $output
```
This doesn't work on Windows CMD (no `cat`, `sed` quote handling is broken).

**Fix:** Python script that does the same thing:
1. Read all input files, wrap `Q(...)` lines in quotes
2. Write to a temp file, run GCC preprocessor with `@flagsfile`
3. Unwrap `Q(...)` lines from the output

Used conditionally via `CMAKE_HOST_WIN32` check in `mkrules.cmake`.

#### `ports/esp32/Makefile` + `py/mkenv.mk` — PYTHON variable

**Problem:** Both files hardcode `PYTHON ?= python3`, but on Windows the Python
executable is `python` (not `python3`).

**Fix:** Conditional based on `$(OS)`:
```makefile
ifeq ($(OS),Windows_NT)
PYTHON ?= python
else
PYTHON ?= python3
endif
```

### 3. optacon-firmware Changes

#### `python/manifest.py` — portable freeze path

**Problem:** Previously used `freeze("/workspace/python")` which is Docker-specific.

**Fix:** Changed to `freeze(".")` which works everywhere because MicroPython's
`makemanifest.py` calls `os.chdir(os.path.dirname(manifest_path))` before
executing the manifest.

#### `build-local.cmd` — Windows build script

Batch script that:
- Sets board, workspace, and `MICROPY_MPYCROSS` paths
- Calls `export.bat` if `IDF_PATH` not already set
- Converts backslashes to forward slashes before passing paths to GNU Make
  (Make interprets `\` as escape characters, stripping them from paths)
- Runs `make submodules` then `make` with `USER_C_MODULES` and `FROZEN_MANIFEST`
- Copies output binaries to `build/`

#### `run-build.bat` — Git Bash wrapper

Thin wrapper that sources the ESP-IDF environment via `export.bat` then calls
`build-local.cmd` with an absolute path (needed because `call build-local.cmd`
fails after `export.bat` modifies the PATH).

## Troubleshooting

### `'build-local.cmd' is not recognized`
Use absolute path: `call "C:\...\build-local.cmd"`. After `export.bat` modifies
PATH, relative command resolution can break.

### `USER_C_MODULES doesn't exist: C:ProjectsOptacon...`
Backslashes are being stripped by GNU Make. Ensure `build-local.cmd` converts
`%WORKSPACE%` to forward slashes before passing to `make`.

### Build produces no output, `makeimg.py` fails with `FileNotFoundError`
`idf.py` is silently skipping the build. Check that `idf.py` is patched to call
`main()` after the MSYSTEM warning.

### `tool riscv32-esp-elf-gdb has no installed versions`
Run from a CMD prompt (not Git Bash):
```cmd
python %IDF_PATH%\tools\idf_tools.py install riscv32-esp-elf-gdb
```
Or after patching `idf_tools.py`, from Git Bash:
```bash
python "$IDF_PATH/tools/idf_tools.py" install riscv32-esp-elf-gdb
```

### CMake cache mismatch (`source does not match`)
If you switch between ESP-IDF installations, delete the build directory:
```cmd
rmdir /S /Q C:\Projects\Optacon\micropython\ports\esp32\build-ESP32_GENERIC_S3
```

### `#include expects "FILENAME" or <FILENAME>` errors
The response file has unescaped quotes. Ensure `mkrules.cmake` escapes `"` as `\"`
in the flags written to `qstr.i.last.in`.
