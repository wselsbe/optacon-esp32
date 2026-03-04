# WebREPL & Integrated Terminal Design

Date: 2026-03-04

## Goal

Add two complementary features to the Optacon web interface:
1. **Built-in WebREPL** (port 8266) — MicroPython's standard WebSocket REPL for file transfer and fallback terminal, activated by config file
2. **Integrated terminal panel** — Collapsible exec console in index.html, reusing the existing Microdot WebSocket

## 1. Built-in WebREPL

### Activation

WebREPL is opt-in via MicroPython's standard `webrepl_cfg.py` convention:

```python
# In main.py, after imports
try:
    import webrepl
    webrepl.start()  # reads PASS from webrepl_cfg.py
except (ImportError, OSError):
    pass  # no config file → WebREPL stays disabled
```

To enable: upload `webrepl_cfg.py` containing `PASS = 'yourpassword'` to the board.
To disable: delete `webrepl_cfg.py`.

No custom config system needed. WebREPL runs on port 8266 alongside the Microdot server on port 80.

### Use Cases

- File upload/download over WiFi (alternative to mpremote over serial)
- Fallback interactive REPL when serial is unavailable
- Compatible with MicroPython's official WebREPL HTML client

## 2. Integrated Terminal Panel

### Server Side

New `exec` command in `web_server.py`'s `_handle_command()`:

```python
elif cmd == "exec":
    code = data.get("code", "")
    import io, sys
    buf = io.StringIO()
    old_stdout = sys.stdout
    sys.stdout = buf
    try:
        try:
            result = eval(code)
            if result is not None:
                print(repr(result))
        except SyntaxError:
            exec(code)
    except Exception as e:
        sys.stdout = old_stdout
        return {"error": str(e)}
    finally:
        sys.stdout = old_stdout
    return {"output": buf.getvalue()}
```

Eval-first, exec-fallback pattern: expressions return their value, statements execute and capture stdout. Errors return traceback text.

### UI: Collapsible Panel

Location: bottom of `index.html`, below the HV509 card.

**Components:**
- **Toggle bar** — "Terminal" label + chevron, click to expand/collapse. Starts collapsed.
- **Output area** — `<pre>` element, monospace font, dark background for contrast. Vertically resizable (CSS `resize: vertical`), starts at 200px, min 100px, max 80vh. Scrolls to bottom on new output.
- **Input line** — Single-line text input with `>>>` prompt. Enter to submit.
- **Clear button** — Small link to reset output area.

**Styling:**
```css
.terminal-output {
    min-height: 100px;
    height: 200px;
    max-height: 80vh;
    resize: vertical;
    overflow-y: auto;
    background: #1a1a2e;
    color: #e0e0e0;
    font-family: monospace;
    padding: 8px;
}
```

**JS behavior:**
- Command history: up/down arrow keys cycle previous commands (sessionStorage)
- Enter sends `{cmd: "exec", code: "..."}` over existing `/ws` WebSocket
- Output appended as `>>> command\nresult\n`
- Errors shown in red
- Auto-scroll to bottom

### Data Flow

```
User types command + Enter
  → JSON {cmd: "exec", code: "..."} over /ws
  → _handle_command() eval/exec with stdout capture
  → returns {output: "..."} or {error: "..."}
  → client appends to <pre>, scrolls down
```

## Security Note

Both features expose Python execution over the network. WebREPL requires a password. The integrated terminal has no auth (same as all existing WebSocket commands). Acceptable for a development/lab tool on a local network.
