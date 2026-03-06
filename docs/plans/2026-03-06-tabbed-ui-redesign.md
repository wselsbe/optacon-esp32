# ESP32 Card Tabbed UI Redesign

## Goal

Eliminate duplicated controls between the ESP32 Signal Generator card tabs and the DRV2665 card. The tabs should only contain tab-specific inputs. Shared controls (waveform, gain, start/stop) live in their natural locations.

## Layout

### ESP32 Card

```
┌─ ESP32 ──────────── [Signal] [Speech] [Music] ─┐
│                                                  │
│  WAVEFORM  (shared, hidden on Speech tab)        │
│  [DC] [Sine] [Triangle] [Square]                 │
│  (DC disabled on Music tab)                      │
│                                                  │
│  Signal tab:                                     │
│    Frequency slider + input + Hz                 │
│    Toggle Polarity checkbox                      │
│    Amplitude slider                              │
│                                                  │
│  Speech tab:                                     │
│    Text input (Enter triggers START)             │
│    Speed slider | Pitch slider                   │
│                                                  │
│  Music tab:                                      │
│    Notes textarea                                │
│    BPM input                                     │
│    Built-in Songs grid (click loads notes only)  │
└──────────────────────────────────────────────────┘
```

### DRV2665 Card (unchanged structure, dynamic button)

```
┌─ DRV2665 ─── Piezo Driver ──────────────────────┐
│  GAIN: [25Vpp] [50Vpp] [75Vpp] [100Vpp]         │
│  [ START / SPEAK / PLAY ]  (label per tab)       │
└──────────────────────────────────────────────────┘
```

## Behavior

### Tab switching
- Changes the DRV2665 button label: Signal→"START", Speech→"SPEAK", Music→"PLAY"
- Waveform selector: hidden on Speech tab, visible on Signal/Music tabs
- DC waveform: disabled on Music tab (only sine/triangle/square)

### DRV2665 START button per tab
- **Signal**: sends `start` with gain (existing behavior). Shows "STOP" while running.
- **Speech**: sends `say` with text/speed/pitch. Shows "SPEAKING..." while busy, reverts on completion. No STOP (blocking call).
- **Music**: sends `play_music` with notes/bpm/waveform/gain. Shows "PLAYING..." while busy, "STOP" to interrupt.

### Song buttons
- Click loads notes into textarea. Does NOT auto-play.
- May also set recommended gain/waveform for the song if metadata exists.

### Enter key
- In Speech text input: triggers the DRV2665 start button click.

## Files to change

### Modify
- `web/index.html` — restructure ESP32 card (waveform above tabs, remove per-tab duplicates), rewire DRV2665 button JS
- `python/web_server.py` — remove `/speech` and `/music` routes

### Delete
- `web/speech.html`
- `web/music.html`

## Removed controls
- Music tab: PLAY button, STOP button, Waveform select, Gain select
- Speech tab: SPEAK button
