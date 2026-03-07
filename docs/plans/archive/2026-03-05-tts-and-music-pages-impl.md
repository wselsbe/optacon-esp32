# TTS & Music Pages Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Add on-device English text-to-speech via SAM (Software Automatic Mouth) C module, plus `/speech` and `/music` web pages with WebSocket controls.

**Architecture:** SAM's C code compiles as a MicroPython C user module (`modules/sam/`). It outputs 8-bit PCM at 22,050 Hz, played via the existing `pz_drive.pwm_play_samples()` ISR. Two new web pages provide browser controls. Music playback reuses `music.py`.

**Tech Stack:** C (SAM formant synth), MicroPython C modules, Microdot WebSocket, vanilla HTML/CSS/JS

---

## Task 1: Clone SAM source and strip to core files

**Files:**
- Create: `modules/sam/sam.c` (from upstream, modified)
- Create: `modules/sam/render.c` (from upstream, modified)
- Create: `modules/sam/reciter.c` (from upstream, unmodified)
- Create: `modules/sam/sam.h` (from upstream, unmodified)
- Create: `modules/sam/render.h` (from upstream, unmodified)
- Create: `modules/sam/reciter.h` (from upstream, unmodified)
- Create: `modules/sam/SamTabs.h` (from upstream, unmodified)
- Create: `modules/sam/RenderTabs.h` (from upstream, unmodified)
- Create: `modules/sam/ReciterTabs.h` (from upstream, unmodified)

**Step 1: Clone the SAM repo**

```bash
cd /tmp && git clone https://github.com/s-macke/SAM.git
```

**Step 2: Copy core source files**

```bash
cd /path/to/optacon-firmware
mkdir -p modules/sam
cp /tmp/SAM/src/sam.c modules/sam/
cp /tmp/SAM/src/sam.h modules/sam/
cp /tmp/SAM/src/render.c modules/sam/
cp /tmp/SAM/src/render.h modules/sam/
cp /tmp/SAM/src/reciter.c modules/sam/
cp /tmp/SAM/src/reciter.h modules/sam/
cp /tmp/SAM/src/SamTabs.h modules/sam/
cp /tmp/SAM/src/RenderTabs.h modules/sam/
cp /tmp/SAM/src/ReciterTabs.h modules/sam/
```

Do NOT copy `main.c` (SDL/CLI frontend) or `debug.c`/`debug.h` (printf-based debug).

**Step 3: Strip debug and stdio dependencies**

In `modules/sam/sam.c`:
- Remove `#include "debug.h"` and `#include <stdio.h>` (only used by debug paths)
- Set `int debug = 0;` (keep the global, remove debug function calls)
- Remove all `if(debug) { ... }` blocks and calls to `PrintPhonemes()`, `PrintOutput()`, etc.
- Keep: `#include <string.h>`, `#include <stdlib.h>` (for `strlen`, `memset`, `malloc`)

In `modules/sam/render.c`:
- Remove `#include "debug.h"` and `#include <stdio.h>`
- Remove all `if(debug) { ... }` blocks
- Keep everything else — the formant synthesis is all here

In `modules/sam/reciter.c`:
- Should have no debug or stdio dependencies. Verify and leave as-is.

**Step 4: Verify the key API functions exist in sam.h**

The public API we need:
```c
void SetInput(char *_input);
void SetSpeed(unsigned char _speed);
void SetPitch(unsigned char _pitch);
void SetMouth(unsigned char _mouth);
void SetThroat(unsigned char _throat);
int SAMMain();             // returns 1 on success
char* GetBuffer();         // pointer to PCM output
int GetBufferLength();     // number of PCM bytes
```

**Step 5: Commit**

```bash
git add modules/sam/
git commit -m "feat: add SAM speech synthesis source files (stripped of SDL/debug)"
```

---

## Task 2: Create MicroPython C module bindings

**Files:**
- Create: `modules/sam/mod_sam.c`
- Create: `modules/sam/micropython.cmake`
- Modify: `modules/micropython.cmake`

**Step 1: Create the MicroPython binding file**

Create `modules/sam/mod_sam.c`:

```c
#include "py/runtime.h"
#include "py/obj.h"
#include "sam.h"

// Forward declarations from pz_drive (for say())
extern void pzd_pwm_play_samples(const uint8_t *buf, size_t len,
                                  uint32_t sample_rate, bool loop);
extern bool pzd_pwm_is_sample_done(void);

// sam.render(text, speed=72, pitch=64, mouth=128, throat=128)
// Returns bytearray of 8-bit unsigned PCM at 22050 Hz
static mp_obj_t mod_sam_render(size_t n_args, const mp_obj_t *pos_args,
                                mp_map_t *kw_args) {
    enum { ARG_text, ARG_speed, ARG_pitch, ARG_mouth, ARG_throat };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_text,   MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_speed,  MP_ARG_KW_ONLY  | MP_ARG_INT, {.u_int = 72} },
        { MP_QSTR_pitch,  MP_ARG_KW_ONLY  | MP_ARG_INT, {.u_int = 64} },
        { MP_QSTR_mouth,  MP_ARG_KW_ONLY  | MP_ARG_INT, {.u_int = 128} },
        { MP_QSTR_throat, MP_ARG_KW_ONLY  | MP_ARG_INT, {.u_int = 128} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args,
                     MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    const char *text = mp_obj_str_get_str(args[ARG_text].u_obj);
    unsigned char speed  = (unsigned char)args[ARG_speed].u_int;
    unsigned char pitch  = (unsigned char)args[ARG_pitch].u_int;
    unsigned char mouth  = (unsigned char)args[ARG_mouth].u_int;
    unsigned char throat = (unsigned char)args[ARG_throat].u_int;

    // SAM modifies the input buffer, so copy it
    size_t len = strlen(text);
    if (len > 254) {
        mp_raise_ValueError(MP_ERROR_TEXT("text too long (max 254 chars)"));
    }
    char input[256];
    memcpy(input, text, len + 1);

    SetInput(input);
    SetSpeed(speed);
    SetPitch(pitch);
    SetMouth(mouth);
    SetThroat(throat);

    if (!SAMMain()) {
        mp_raise_ValueError(MP_ERROR_TEXT("SAM synthesis failed"));
    }

    char *buf = GetBuffer();
    int buf_len = GetBufferLength();

    if (buf == NULL || buf_len <= 0) {
        mp_raise_ValueError(MP_ERROR_TEXT("SAM produced no output"));
    }

    // Return as bytearray (copies data so SAM buffer can be freed/reused)
    mp_obj_t result = mp_obj_new_bytearray(buf_len, (const byte *)buf);

    // Free SAM's internal malloc'd buffer
    free(buf);

    return result;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(mod_sam_render_obj, 1, mod_sam_render);

// sam.say(text, speed=72, pitch=64, mouth=128, throat=128, gain=100)
// Renders and plays through pz_drive, blocks until done
static mp_obj_t mod_sam_say(size_t n_args, const mp_obj_t *pos_args,
                             mp_map_t *kw_args) {
    enum { ARG_text, ARG_speed, ARG_pitch, ARG_mouth, ARG_throat, ARG_gain };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_text,   MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_speed,  MP_ARG_KW_ONLY  | MP_ARG_INT, {.u_int = 72} },
        { MP_QSTR_pitch,  MP_ARG_KW_ONLY  | MP_ARG_INT, {.u_int = 64} },
        { MP_QSTR_mouth,  MP_ARG_KW_ONLY  | MP_ARG_INT, {.u_int = 128} },
        { MP_QSTR_throat, MP_ARG_KW_ONLY  | MP_ARG_INT, {.u_int = 128} },
        { MP_QSTR_gain,   MP_ARG_KW_ONLY  | MP_ARG_INT, {.u_int = 100} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args,
                     MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // Render via the render function (reuse the same logic)
    // Build kwargs for render call
    mp_obj_t render_args[5] = {
        args[ARG_text].u_obj,
        MP_OBJ_NEW_SMALL_INT(args[ARG_speed].u_int),
        MP_OBJ_NEW_SMALL_INT(args[ARG_pitch].u_int),
        MP_OBJ_NEW_SMALL_INT(args[ARG_mouth].u_int),
        MP_OBJ_NEW_SMALL_INT(args[ARG_throat].u_int),
    };

    // Call SAM directly (same logic as render)
    const char *text = mp_obj_str_get_str(args[ARG_text].u_obj);
    unsigned char speed  = (unsigned char)args[ARG_speed].u_int;
    unsigned char pitch  = (unsigned char)args[ARG_pitch].u_int;
    unsigned char mouth  = (unsigned char)args[ARG_mouth].u_int;
    unsigned char throat = (unsigned char)args[ARG_throat].u_int;

    size_t len = strlen(text);
    if (len > 254) {
        mp_raise_ValueError(MP_ERROR_TEXT("text too long (max 254 chars)"));
    }
    char input[256];
    memcpy(input, text, len + 1);

    SetInput(input);
    SetSpeed(speed);
    SetPitch(pitch);
    SetMouth(mouth);
    SetThroat(throat);

    if (!SAMMain()) {
        mp_raise_ValueError(MP_ERROR_TEXT("SAM synthesis failed"));
    }

    char *buf = GetBuffer();
    int buf_len = GetBufferLength();

    if (buf == NULL || buf_len <= 0) {
        free(buf);
        mp_raise_ValueError(MP_ERROR_TEXT("SAM produced no output"));
    }

    // Play via pz_drive
    pzd_pwm_play_samples((const uint8_t *)buf, buf_len, 22050, false);

    // Poll until done, allowing Ctrl-C
    while (!pzd_pwm_is_sample_done()) {
        mp_handle_pending(true);
        mp_hal_delay_ms(10);
    }

    free(buf);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(mod_sam_say_obj, 1, mod_sam_say);

// Module globals table
static const mp_rom_map_elem_t sam_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_sam) },
    { MP_ROM_QSTR(MP_QSTR_render),   MP_ROM_PTR(&mod_sam_render_obj) },
    { MP_ROM_QSTR(MP_QSTR_say),      MP_ROM_PTR(&mod_sam_say_obj) },
};
static MP_DEFINE_CONST_DICT(sam_module_globals, sam_module_globals_table);

const mp_obj_module_t sam_module = {
    .base    = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sam_module_globals,
};
MP_REGISTER_MODULE(MP_QSTR_sam, sam_module);
```

**Step 2: Create cmake file**

Create `modules/sam/micropython.cmake`:

```cmake
add_library(usermod_sam INTERFACE)

target_sources(usermod_sam INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/mod_sam.c
    ${CMAKE_CURRENT_LIST_DIR}/sam.c
    ${CMAKE_CURRENT_LIST_DIR}/render.c
    ${CMAKE_CURRENT_LIST_DIR}/reciter.c
)

target_include_directories(usermod_sam INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/../pz_drive
)

target_link_libraries(usermod_sam INTERFACE usermod_pz_drive)
target_link_libraries(usermod INTERFACE usermod_sam)
```

**Step 3: Add include to root cmake**

In `modules/micropython.cmake`, add after the existing includes:

```cmake
include(${CMAKE_CURRENT_LIST_DIR}/sam/micropython.cmake)
```

**Step 4: Commit**

```bash
git add modules/sam/mod_sam.c modules/sam/micropython.cmake modules/micropython.cmake
git commit -m "feat: add SAM MicroPython C module bindings"
```

---

## Task 3: Build and test SAM on hardware

**Step 1: Build firmware**

```bash
./scripts/build.sh
```

Expected: build succeeds. If there are compile errors in SAM source (missing includes, debug references), fix them. Check firmware size — should still fit in 1.5 MB OTA slot.

**Step 2: Flash and verify import**

Use the `/flash` skill to flash the board.

**Step 3: Test render**

```python
import sam
buf = sam.render("hello world")
print(len(buf), "bytes")  # expect ~20000-60000
```

**Step 4: Test say**

```python
import sam
sam.say("hello world")
```

Expected: you hear robot speech from the piezo. If no sound, check:
- `pz_drive.pwm_play_samples()` is being called correctly
- The ISR is started (may need `pz_drive.pwm_start()` first — check if `play_samples` auto-starts)
- Gain/amplitude is sufficient

**Step 5: Test parameters**

```python
sam.say("testing speed", speed=50)   # slower
sam.say("testing speed", speed=120)  # faster
sam.say("testing pitch", pitch=30)   # low pitch
sam.say("testing pitch", pitch=200)  # high pitch
```

**Step 6: Commit any fixes**

```bash
git add -u
git commit -m "fix: resolve SAM build/runtime issues"
```

---

## Task 4: Add WebSocket commands for speech and music

**Files:**
- Modify: `python/web_server.py`

**Step 1: Add `say` command to `_handle_command`**

In `python/web_server.py`, in `_handle_command()`, add before the `else` branch:

```python
    elif cmd == "say":
        import sam
        text = data.get("text", "")
        if not text:
            return {"error": "no text provided"}
        sam.say(
            text,
            speed=data.get("speed", 72),
            pitch=data.get("pitch", 64),
            mouth=data.get("mouth", 128),
            throat=data.get("throat", 128),
        )
        return {"msg": "speech complete"}
```

**Step 2: Add `play_music` and `stop_music` commands**

```python
    elif cmd == "play_music":
        import music
        notes_str = data.get("notes", "")
        if not notes_str:
            return {"error": "no notes provided"}
        # Parse "C4:4 D4:8 E4:2 R:4" format
        song = []
        for token in notes_str.split():
            parts = token.split(":")
            note = parts[0]
            beats = float(parts[1]) if len(parts) > 1 else 1
            # Convert beat notation: 4=quarter(1), 8=eighth(0.5), 2=half(2), 1=whole(4)
            song.append((note if note != "R" else None, beats))
        music.play(
            song,
            bpm=data.get("bpm", 120),
            waveform=data.get("waveform", "sine"),
            gain=data.get("gain", 75),
        )
        return {"msg": "music complete"}
    elif cmd == "play_song":
        import music
        name = data.get("name", "")
        if name not in music.SONGS:
            return {"error": "unknown song: " + name}
        music.play_song(
            name,
            waveform=data.get("waveform", "sine"),
        )
        return {"msg": "music complete"}
    elif cmd == "stop":
        pa.stop()
```

**Step 3: Add routes for new pages**

Add before the existing `@app.route("/update")`:

```python
@app.route("/speech")
async def speech_page(request):
    return send_file("/web/speech.html")


@app.route("/music")
async def music_page(request):
    return send_file("/web/music.html")
```

**Step 4: Add a REST endpoint to list available songs**

```python
@app.route("/api/music/songs")
async def music_songs(request):
    import music
    songs = list(music.SONGS.keys())
    return json.dumps({"songs": songs}), 200, {"Content-Type": "application/json"}
```

**Step 5: Commit**

```bash
git add python/web_server.py
git commit -m "feat: add say, play_music, play_song WebSocket commands and routes"
```

---

## Task 5: Create the speech web page

**Files:**
- Create: `web/speech.html`

**Step 1: Create `web/speech.html`**

Follow the design system from existing pages (update.html pattern). The page should have:

- Header: OPTACON brand + "Control Panel" back link
- Card: "Speech" section with:
  - Text input (full width, type="text", placeholder="Type something to say...")
  - Speed slider: range 1-255, default 72, with live value display
  - Pitch slider: range 1-255, default 64, with live value display
  - SPEAK button (`.big-btn.apply` style, disabled while speaking)
- Status message area (`.msg` pattern)
- Footer nav: Control Panel, Music, WiFi, Updates + version info

WebSocket connection pattern: connect to `ws://` + `location.host` + `/ws`. On SPEAK click:
1. Disable button, show "Speaking..."
2. Send `{"cmd":"say","text":"...","speed":N,"pitch":N}`
3. On response, re-enable button, show "Done" or error

Use the same CSS variables, card structure, button classes, and footer pattern as `update.html`.

**Step 2: Test in browser**

Navigate to `http://<board-ip>/speech`, type text, click SPEAK, verify sound plays.

**Step 3: Commit**

```bash
git add web/speech.html
git commit -m "feat: add speech web page for text-to-speech"
```

---

## Task 6: Create the music web page

**Files:**
- Create: `web/music.html`

**Step 1: Create `web/music.html`**

Same design system. The page should have:

- Header: OPTACON brand + "Control Panel" back link
- Card: "Sheet Music" section with:
  - Textarea for note notation (rows=4, placeholder="C4:4 D4:4 E4:2 R:4 G4:1")
  - Help text below textarea explaining format: `NOTE:BEATS` (e.g. `C4:4` = C4 quarter note, `R:2` = half rest)
  - Field row with:
    - BPM input (type="number", default 120, min=30, max=600)
    - Waveform select (sine/triangle/square)
    - Gain select (25/50/75/100)
  - PLAY button (`.big-btn.go`) and STOP button (`.big-btn.halt`) side by side
- Card: "Built-in Songs" section with:
  - Buttons for each song, fetched from `/api/music/songs`
  - Each button sends `{"cmd":"play_song","name":"song_name","waveform":"..."}`
- Status message area
- Footer nav: Control Panel, Speech, WiFi, Updates + version info

WebSocket interaction:
- PLAY → disable play, show "Playing...", send `play_music` command
- STOP → send `{"cmd":"stop"}`
- Song button → same as PLAY but send `play_song` command
- On response → re-enable buttons, show status

**Step 2: Test in browser**

Navigate to `http://<board-ip>/music`, enter notes, click PLAY, verify playback.

**Step 3: Commit**

```bash
git add web/music.html
git commit -m "feat: add music web page for sheet music playback"
```

---

## Task 7: Add nav links to all existing pages

**Files:**
- Modify: `web/index.html`
- Modify: `web/wifi.html`
- Modify: `web/update.html`
- Modify: `web/docs.html`

**Step 1: Update footer nav on all pages**

Add `Speech` and `Music` links to the footer of each page. The footer pattern is:

```html
<a href="/">Control Panel</a> &middot;
<a href="/speech">Speech</a> &middot;
<a href="/music">Music</a> &middot;
<a href="/wifi">WiFi</a> &middot;
<a href="/update">Updates</a>
```

Each page omits its own link (e.g. `index.html` omits "Control Panel", `speech.html` omits "Speech").

**Step 2: Commit**

```bash
git add web/index.html web/wifi.html web/update.html web/docs.html
git commit -m "feat: add Speech and Music nav links to all pages"
```

---

## Task 8: Upload filesystem files and end-to-end test

**Step 1: Upload files to board**

```bash
mpremote cp python/web_server.py :
mpremote cp web/speech.html web/music.html :web/
mpremote cp web/index.html web/wifi.html web/update.html web/docs.html :web/
```

**Step 2: Soft reset and test**

1. Soft reset the board
2. Connect to WiFi
3. Navigate to `/speech` — type "hello world", click SPEAK, verify audio
4. Navigate to `/music` — enter `C4:4 D4:4 E4:4 F4:4 G4:2`, click PLAY, verify playback
5. Try a built-in song button
6. Verify all nav links work across pages
7. Test error cases: empty text, invalid notes

**Step 3: Commit any fixes**

```bash
git add -u
git commit -m "fix: resolve end-to-end test issues"
```

---

## Task 9: Update flash skill and documentation

**Files:**
- Modify: `.claude/skills/flash/SKILL.md`
- Modify: `CLAUDE.md`

**Step 1: Update flash skill**

Add `web/speech.html` and `web/music.html` to the filesystem web files list in SKILL.md.

**Step 2: Update CLAUDE.md**

Add SAM module to the project structure section:
```
- `modules/sam/` — SAM speech synthesizer C module (text-to-speech)
```

Add to Python API section:
```python
import sam
sam.say("hello world", speed=72, pitch=64)  # blocking TTS
buf = sam.render("hello")                   # returns PCM bytearray
```

**Step 3: Commit**

```bash
git add .claude/skills/flash/SKILL.md CLAUDE.md
git commit -m "docs: update flash skill and CLAUDE.md for SAM and new web pages"
```
