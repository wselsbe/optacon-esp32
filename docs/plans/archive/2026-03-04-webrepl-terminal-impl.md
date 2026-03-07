# WebREPL & Integrated Terminal Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Add opt-in WebREPL (port 8266) and a collapsible Python terminal panel to the existing Optacon web UI.

**Architecture:** WebREPL is enabled by `webrepl_cfg.py` presence on the board's filesystem. The integrated terminal reuses the existing Microdot WebSocket at `/ws` with a new `exec` command that does eval-then-exec with stdout capture. The terminal panel is a collapsible section at the bottom of `index.html`.

**Tech Stack:** MicroPython WebREPL, Microdot WebSocket, vanilla HTML/CSS/JS

---

### Task 1: Add WebREPL opt-in startup to main.py

**Files:**
- Modify: `python/main.py:1-9`

**Step 1: Add WebREPL startup block**

After the existing `pz_drive.i2c_write(0x02, 0x40)` line (line 8), and before the demo functions (line 11), add:

```python
# Start WebREPL if configured (needs webrepl_cfg.py with PASS='...')
try:
    import webrepl
    webrepl.start()
    print("WebREPL started on port 8266")
except Exception:
    pass
```

**Step 2: Commit**

```bash
git add python/main.py
git commit -m "feat: add opt-in WebREPL startup in main.py"
```

---

### Task 2: Add exec command handler to web_server.py

**Files:**
- Modify: `python/web_server.py:20-81`

**Step 1: Add the exec command branch**

In `_handle_command()`, after the `wifi_config` handler (line 77, after `return {"msg": "WiFi reconnected", "ip": wifi.ip}`) and before the `else` clause (line 78), add a new `elif`:

```python
    elif cmd == "exec":
        code = data.get("code", "")
        import io
        import sys

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
            import sys as _sys

            _sys.print_exception(e, buf)
            sys.stdout = old_stdout
            return {"output": buf.getvalue()}
        finally:
            sys.stdout = old_stdout
        return {"output": buf.getvalue()}
```

Note: Uses `sys.print_exception(e, buf)` (MicroPython-specific) to get full tracebacks including line numbers, rather than just `str(e)`.

**Step 2: Verify the exec command returns output, not status**

The existing WebSocket handler at line 115-119 does:
```python
err = _handle_command(msg)
if err:
    await ws.send(json.dumps(err))
else:
    await ws.send(json.dumps(_get_status()))
```

The `exec` handler returns a dict `{"output": "..."}` which is truthy, so it takes the `if err:` path and sends the output directly. This is the correct behavior — exec responses should NOT include the full hardware status. No changes needed to the WebSocket handler.

**Step 3: Commit**

```bash
git add python/web_server.py
git commit -m "feat(web): add exec command for terminal REPL over WebSocket"
```

---

### Task 3: Add terminal panel CSS to index.html

**Files:**
- Modify: `web/index.html:119-125` (CSS section, before `</style>`)

**Step 1: Add terminal styles**

Before the closing `</style>` tag (line 125), add:

```css
/* Terminal */
.term{background:var(--card);border:1px solid var(--border);border-radius:var(--radius);
  overflow:hidden;box-shadow:0 1px 3px #0001}
.term-toggle{display:flex;align-items:center;justify-content:space-between;padding:9px 16px;
  cursor:pointer;-webkit-user-select:none;user-select:none;background:#faf9f7;
  border-bottom:1px solid var(--border)}
.term-toggle b{font-family:"Consolas","SF Mono","Menlo",monospace;font-size:13px;
  letter-spacing:.5px;color:var(--text2)}
.term-toggle .chev{font-size:11px;color:var(--text3);transition:transform .2s}
.term.open .chev{transform:rotate(180deg)}
.term-body{display:none;padding:0}
.term.open .term-body{display:block}
.term-out{min-height:100px;height:200px;max-height:80vh;resize:vertical;overflow-y:auto;
  background:#1a1a2e;color:#e0e0e0;font-family:"Consolas","SF Mono","Menlo",monospace;
  font-size:13px;padding:8px 10px;margin:0;white-space:pre-wrap;word-break:break-all;
  line-height:1.45;border:none;border-bottom:1px solid #2a2a3e}
.term-out .err{color:#f06060}
.term-in-row{display:flex;align-items:center;background:#1a1a2e;padding:0 10px 8px}
.term-prompt{color:#7a7;font-family:"Consolas","SF Mono","Menlo",monospace;font-size:13px;
  margin-right:6px;flex-shrink:0}
.term-in{flex:1;background:transparent;border:none;outline:none;color:#e0e0e0;
  font-family:"Consolas","SF Mono","Menlo",monospace;font-size:13px;padding:4px 0}
.term-clear{font-size:11px;color:var(--text3);background:none;border:none;cursor:pointer;
  padding:2px 6px;margin-left:4px;font-family:inherit}
.term-clear:hover{color:#e0e0e0}
```

**Step 2: Commit**

```bash
git add web/index.html
git commit -m "feat(web): add terminal panel CSS"
```

---

### Task 4: Add terminal panel HTML to index.html

**Files:**
- Modify: `web/index.html:239` (after closing HV509 card `</div>`, before toast `<div>`)

**Step 1: Add terminal HTML**

After the HV509 card closing `</div>` (line 237) and before `<div class="toast"` (line 239), add:

```html
<!-- Terminal -->
<div class="term" id="term">
  <div class="term-toggle" onclick="toggleTerm()">
    <b>Terminal</b>
    <span class="chev">&#9660;</span>
  </div>
  <div class="term-body">
    <pre class="term-out" id="term-out"></pre>
    <div class="term-in-row">
      <span class="term-prompt">&gt;&gt;&gt;</span>
      <input class="term-in" id="term-in" type="text" spellcheck="false" autocomplete="off"
             placeholder="Python expression or statement">
      <button class="term-clear" onclick="clearTerm()" title="Clear output">Clear</button>
    </div>
  </div>
</div>
```

**Step 2: Commit**

```bash
git add web/index.html
git commit -m "feat(web): add terminal panel HTML"
```

---

### Task 5: Add terminal panel JavaScript to index.html

**Files:**
- Modify: `web/index.html` (in `<script>` section, before `connect();` near end)

**Step 1: Add terminal JS**

Before the final `connect();` call (line 411), add:

```javascript
// Terminal
var termOut=$('term-out'),termIn=$('term-in'),termEl=$('term');
var termHist=[],termIdx=-1;
function toggleTerm(){termEl.classList.toggle('open');if(termEl.classList.contains('open'))termIn.focus()}
function clearTerm(){termOut.textContent=''}
function termAppend(text,cls){
  if(cls){var s=document.createElement('span');s.className=cls;s.textContent=text;termOut.appendChild(s)}
  else termOut.appendChild(document.createTextNode(text));
  termOut.scrollTop=termOut.scrollHeight;
}
termIn.onkeydown=function(e){
  if(e.key==='Enter'){
    var code=termIn.value;if(!code)return;
    termHist.push(code);termIdx=termHist.length;
    termAppend('>>> '+code+'\n');
    send({cmd:'exec',code:code});
    termIn.value='';
  }else if(e.key==='ArrowUp'){
    e.preventDefault();
    if(termIdx>0){termIdx--;termIn.value=termHist[termIdx]}
  }else if(e.key==='ArrowDown'){
    e.preventDefault();
    if(termIdx<termHist.length-1){termIdx++;termIn.value=termHist[termIdx]}
    else{termIdx=termHist.length;termIn.value=''}
  }
};
```

**Step 2: Update the ws.onmessage handler to route exec responses**

The existing `ws.onmessage` handler (line 277-281) is:

```javascript
ws.onmessage=function(e){
    var d;try{d=JSON.parse(e.data)}catch(x){return}
    if(d.error){showErr(d.error);return}
    state=d;upd=true;updateUI();upd=false;
};
```

Replace it with:

```javascript
ws.onmessage=function(e){
    var d;try{d=JSON.parse(e.data)}catch(x){return}
    if(d.output!=null){termAppend(d.output||'');return}
    if(d.error){showErr(d.error);return}
    state=d;upd=true;updateUI();upd=false;
};
```

The only change: added the `if(d.output!=null)` check before the existing error/status handling. Exec responses have an `output` key and get routed to the terminal; all other responses (status, errors) are handled as before.

**Step 3: Commit**

```bash
git add web/index.html
git commit -m "feat(web): add terminal panel JavaScript with command history"
```

---

### Task 6: Upload and test

**Step 1: Upload changed files to board**

`web_server.py` and `main.py` are filesystem modules (not frozen), and `web/index.html` is served from filesystem. Use the fast path:

```bash
mpremote cp python/web_server.py python/main.py :
mpremote cp web/index.html :web/index.html
```

**Step 2: Soft reset the board**

```
mcp__micropython__soft_reset()
```

**Step 3: Verify boot**

```
mcp__micropython__exec("import sys; print('MicroPython', sys.version)")
```

**Step 4: Test the terminal panel**

Open the web UI in a browser. Click the "Terminal" bar at the bottom to expand. Type test commands:

- `2+2` → should show `4`
- `import os; os.listdir('/')` → should show filesystem listing
- `print("hello")` → should show `hello`
- `1/0` → should show traceback in output

**Step 5: Test WebREPL activation**

Upload a config file to enable WebREPL:

```
mcp__micropython__write_file("/webrepl_cfg.py", "PASS = 'optacon'\n")
mcp__micropython__soft_reset()
```

Verify WebREPL starts (check for "WebREPL started on port 8266" in boot output).

To disable again: `mcp__micropython__rm("/webrepl_cfg.py")`

**Step 6: Final commit if any fixes needed**

```bash
git add -A
git commit -m "fix: adjustments from testing"
```
