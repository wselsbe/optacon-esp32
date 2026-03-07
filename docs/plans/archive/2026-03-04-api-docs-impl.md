# API Documentation Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Add a hand-written HTML API reference page served from the ESP32, with CI validation and a link from the main UI.

**Architecture:** Single self-contained `web/docs.html` styled like the existing UI. Served by the existing Microdot static route. Link added to the terminal card toggle bar. CI checks the file exists. Flash skill updated to upload it.

**Tech Stack:** Vanilla HTML/CSS/JS, Microdot static serving, GitHub Actions

---

### Task 1: Create the API docs HTML page

**Files:**
- Create: `web/docs.html`

**Step 1: Create `web/docs.html`**

Create a self-contained HTML page with the same CSS variables and styling patterns as `web/wifi.html` and `web/index.html`. Use card-based layout with one card per module.

The page must include:
- Header with "OPTACON" brand and "Control Panel" back link (same as `web/wifi.html`)
- Four module cards (PzActuator, music, DRV2665, ShiftRegister)
- Each card has method signatures, parameter descriptions, and code examples
- Syntax-highlighted code blocks (minimal inline CSS, no external libs)

Full content for the HTML file:

```html
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>Optacon — API Reference</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
:root{
  --bg:#edebe7;--card:#fff;--border:#ccc9c2;--text:#1a1a20;--text2:#68686e;--text3:#98989e;
  --input:#f5f4f0;--drv:#b85a14;--esp:#2856a0;--hv:#167868;--green:#1a8848;--radius:6px;
}
html{font-family:system-ui,-apple-system,sans-serif;font-size:14px;background:var(--bg);color:var(--text)}
body{max-width:640px;margin:0 auto;padding:10px 12px 28px}
header{display:flex;align-items:center;justify-content:space-between;padding:10px 16px;
  background:var(--card);border:1px solid var(--border);border-radius:var(--radius);margin-bottom:12px;
  box-shadow:0 1px 3px #0001}
.brand{font-family:"Consolas","SF Mono","Menlo",monospace;font-weight:700;font-size:15px;letter-spacing:3px;color:var(--text2)}
a.back{font-size:13px;color:var(--esp);text-decoration:none;font-weight:600}
a.back:hover{text-decoration:underline}
.card{background:var(--card);border:1px solid var(--border);border-radius:var(--radius);
  margin-bottom:12px;overflow:hidden;box-shadow:0 1px 3px #0001}
.card.esp{border-left:4px solid var(--esp)}
.card.drv{border-left:4px solid var(--drv)}
.card.hv{border-left:4px solid var(--hv)}
.card.music{border-left:4px solid var(--green)}
.card-h{padding:10px 16px;border-bottom:1px solid var(--border);background:#faf9f7;
  display:flex;align-items:baseline;gap:8px}
.card-h b{font-family:"Consolas","SF Mono","Menlo",monospace;font-size:13px;letter-spacing:.5px}
.card.esp .card-h b{color:var(--esp)}
.card.drv .card-h b{color:var(--drv)}
.card.hv .card-h b{color:var(--hv)}
.card.music .card-h b{color:var(--green)}
.card-h span{font-size:12px;color:var(--text3);font-weight:400}
.card-b{padding:14px 16px}
p{margin-bottom:10px;line-height:1.5}
h3{font-size:13px;color:var(--text);margin:16px 0 6px;font-family:"Consolas","SF Mono","Menlo",monospace;
  border-bottom:1px solid var(--border);padding-bottom:4px}
h3:first-child{margin-top:0}
.sig{font-family:"Consolas","SF Mono","Menlo",monospace;font-size:12.5px;color:var(--esp);
  background:var(--input);padding:6px 10px;border-radius:4px;margin-bottom:8px;
  overflow-x:auto;white-space:pre;display:block;border:1px solid var(--border)}
.desc{font-size:13px;color:var(--text2);margin-bottom:8px;line-height:1.45}
.params{width:100%;border-collapse:collapse;margin-bottom:10px;font-size:12.5px}
.params th{text-align:left;padding:4px 8px;background:#faf9f7;border:1px solid var(--border);
  font-size:11px;text-transform:uppercase;color:var(--text3);letter-spacing:.5px}
.params td{padding:4px 8px;border:1px solid var(--border);vertical-align:top}
.params td:first-child{font-family:"Consolas","SF Mono","Menlo",monospace;white-space:nowrap;color:var(--esp)}
pre.example{background:#1a1a2e;color:#e0e0e0;font-family:"Consolas","SF Mono","Menlo",monospace;
  font-size:12.5px;padding:10px 12px;border-radius:5px;overflow-x:auto;margin-bottom:10px;
  line-height:1.45;white-space:pre}
pre.example .c{color:#6a9955}
pre.example .s{color:#ce9178}
pre.example .n{color:#9cdcfe}
pre.example .k{color:#569cd6}
.const-list{font-size:12.5px;margin-bottom:10px}
.const-list code{font-family:"Consolas","SF Mono","Menlo",monospace;background:var(--input);
  padding:1px 5px;border-radius:3px;font-size:12px}
.song-table{width:100%;border-collapse:collapse;font-size:12.5px;margin-bottom:10px}
.song-table th{text-align:left;padding:4px 8px;background:#faf9f7;border:1px solid var(--border);
  font-size:11px;text-transform:uppercase;color:var(--text3)}
.song-table td{padding:4px 8px;border:1px solid var(--border)}
.song-table td:first-child{font-family:"Consolas","SF Mono","Menlo",monospace}
.note{font-size:12px;color:var(--text3);background:var(--input);padding:8px 12px;
  border-radius:4px;margin-bottom:10px;line-height:1.45}
</style>
</head>
<body>

<header>
  <div class="brand">OPTACON</div>
  <a href="/" class="back">Control Panel</a>
</header>

<!-- PzActuator -->
<div class="card esp" id="doc-pzactuator">
  <div class="card-h"><b>PzActuator</b> <span>pz_drive_py — High-Level Piezo Controller</span></div>
  <div class="card-b">
    <p>Main API for driving piezo actuators. Wraps the DRV2665 amplifier, HV509 shift registers, and
    the real-time C module. Supports two signal paths: <b>analog</b> (PWM + RC filter + DDS at 32 kHz)
    and <b>digital</b> (I2C FIFO to DRV2665 internal DAC at 8 kHz).</p>

<pre class="example"><span class="k">from</span> pz_drive_py <span class="k">import</span> PzActuator
pa = PzActuator()</pre>

    <h3>set_frequency_analog(hz, ...)</h3>
    <code class="sig">pa.set_frequency_analog(hz, resolution=8, amplitude=100, fullwave=False, dead_time=0, phase_advance=0, waveform="sine")</code>
    <p class="desc">Configure analog PWM+DDS mode. Use for most applications — higher sample rate (32 kHz) and smoother output than digital mode.</p>
    <table class="params">
      <tr><th>Param</th><th>Type</th><th>Description</th></tr>
      <tr><td>hz</td><td>int</td><td>Frequency 0–1000 Hz (0 = DC output)</td></tr>
      <tr><td>resolution</td><td>int</td><td>PWM bit depth: 8 or 10</td></tr>
      <tr><td>amplitude</td><td>int</td><td>Output amplitude 0–100%</td></tr>
      <tr><td>fullwave</td><td>bool</td><td>If True, generate |waveform| and toggle polarity at zero-crossings for ~200V pk-pk</td></tr>
      <tr><td>dead_time</td><td>int</td><td>ISR ticks (at 32 kHz) to force zero near zero-crossings</td></tr>
      <tr><td>phase_advance</td><td>int</td><td>ISR ticks to advance polarity toggle (compensates DRV2665 output lag)</td></tr>
      <tr><td>waveform</td><td>str</td><td>"sine", "triangle", or "square"</td></tr>
    </table>
<pre class="example"><span class="c"># 250 Hz sine wave, half amplitude</span>
pa.set_frequency_analog(<span class="n">250</span>, amplitude=<span class="n">50</span>)
pa.start(gain=<span class="n">100</span>)

<span class="c"># Fullwave mode for higher voltage output</span>
pa.set_frequency_analog(<span class="n">250</span>, fullwave=<span class="k">True</span>)</pre>

    <h3>set_frequency_digital(hz, ...)</h3>
    <code class="sig">pa.set_frequency_digital(hz, fullwave=False, waveform="sine")</code>
    <p class="desc">Configure digital FIFO mode. Uses DRV2665's internal DAC at 8 kHz sample rate. Lower quality than analog but bypasses the external RC filter.</p>
    <table class="params">
      <tr><th>Param</th><th>Type</th><th>Description</th></tr>
      <tr><td>hz</td><td>int</td><td>Frequency 1–1000 Hz</td></tr>
      <tr><td>fullwave</td><td>bool</td><td>If True, play waveform at 2x rate with polarity toggle</td></tr>
      <tr><td>waveform</td><td>str</td><td>"sine", "triangle", or "square"</td></tr>
    </table>
<pre class="example"><span class="c"># 100 Hz triangle wave via digital path</span>
pa.set_frequency_digital(<span class="n">100</span>, waveform=<span class="s">"triangle"</span>)
pa.start(gain=<span class="n">75</span>)</pre>

    <h3>set_frequency_live(hz, ...)</h3>
    <code class="sig">pa.set_frequency_live(hz, amplitude=100, waveform="sine")</code>
    <p class="desc">Update frequency, amplitude, and waveform while the ISR is running — no stop/start needed. Used by the music player for smooth note transitions. Set amplitude=0 for rests (ISR outputs silence).</p>
<pre class="example"><span class="c"># Smooth transition between notes</span>
pa.set_frequency_analog(<span class="n">440</span>)
pa.start(gain=<span class="n">100</span>)
pa.set_frequency_live(<span class="n">880</span>)  <span class="c"># jump to 880 Hz without restart</span>
pa.set_frequency_live(<span class="n">440</span>, amplitude=<span class="n">0</span>)  <span class="c"># silence (rest)</span></pre>

    <h3>start(gain) / stop()</h3>
    <code class="sig">pa.start(gain=100)</code>
    <code class="sig">pa.stop()</code>
    <p class="desc"><b>start()</b> begins output in the configured mode. Must call <code>set_frequency_analog()</code> or <code>set_frequency_digital()</code> first. <b>stop()</b> stops output and puts the DRV2665 into standby.</p>
    <table class="params">
      <tr><th>Param</th><th>Type</th><th>Description</th></tr>
      <tr><td>gain</td><td>int</td><td>DRV2665 output gain: 25 (28.8 dB), 50 (34.8 dB), 75 (38.4 dB), or 100 Vpp (40.7 dB)</td></tr>
    </table>
<pre class="example"><span class="c"># Basic start/stop cycle</span>
pa.set_frequency_analog(<span class="n">250</span>)
pa.shift_register.set_all(<span class="k">True</span>)  <span class="c"># enable all output channels</span>
pa.start(gain=<span class="n">100</span>)
<span class="c"># ... output is active ...</span>
pa.stop()</pre>

    <h3>is_running()</h3>
    <code class="sig">pa.is_running()  # returns bool</code>
    <p class="desc">Returns True if either the analog ISR or digital FIFO task is currently running.</p>

    <h3>play_wav(path, loop=False)</h3>
    <code class="sig">pa.play_wav(path, loop=False)</code>
    <p class="desc">Play a WAV file through the analog PWM path. Supports 8/16-bit PCM, mono or stereo, any sample rate. Stereo files are downmixed to mono (left channel). If loop=True, repeats until stop() is called.</p>
<pre class="example">pa.play_wav(<span class="s">"/audio/alert.wav"</span>)
pa.play_wav(<span class="s">"/audio/tone.wav"</span>, loop=<span class="k">True</span>)
<span class="c"># ... later ...</span>
pa.stop()</pre>

    <h3>sweep_analog(start_hz, end_hz, duration_ms, ...)</h3>
    <code class="sig">pa.sweep_analog(start_hz, end_hz, duration_ms, logarithmic=False, waveform="sine", resolution=8, amplitude=100, gain=100)</code>
    <p class="desc">Sweep frequency from start_hz to end_hz over duration_ms. After the sweep completes, output holds at end_hz until stop(). Supports linear and logarithmic sweeps.</p>
    <table class="params">
      <tr><th>Param</th><th>Type</th><th>Description</th></tr>
      <tr><td>start_hz</td><td>int</td><td>Starting frequency 1–1000</td></tr>
      <tr><td>end_hz</td><td>int</td><td>Ending frequency 1–1000</td></tr>
      <tr><td>duration_ms</td><td>int</td><td>Sweep duration 1–60000 ms</td></tr>
      <tr><td>logarithmic</td><td>bool</td><td>If True, logarithmic sweep; if False, linear</td></tr>
    </table>
<pre class="example"><span class="c"># Linear sweep from 100 to 500 Hz over 2 seconds</span>
pa.sweep_analog(<span class="n">100</span>, <span class="n">500</span>, <span class="n">2000</span>)

<span class="c"># Logarithmic sweep (perceptually even)</span>
pa.sweep_analog(<span class="n">50</span>, <span class="n">1000</span>, <span class="n">3000</span>, logarithmic=<span class="k">True</span>)</pre>

    <h3>get_status()</h3>
    <code class="sig">pa.get_status()  # returns dict</code>
    <p class="desc">Returns current state as a dict. Used by the web UI for real-time status updates.</p>
<pre class="example">status = pa.get_status()
<span class="c"># {'running': True, 'mode': 'analog', 'frequency': 250,</span>
<span class="c">#  'gain': 100, 'fullwave': False, 'waveform': 'sine',</span>
<span class="c">#  'polarity': False, 'pins': [1,1,0,...],</span>
<span class="c">#  'amplitude': 100, 'dead_time': 0, 'phase_advance': 0}</span></pre>

    <h3>shift_register</h3>
    <p class="desc">The <code>pa.shift_register</code> attribute is a <a href="#doc-shiftreg">ShiftRegister</a> instance for controlling the 20 HV509 output channels.</p>
  </div>
</div>

<!-- music -->
<div class="card music" id="doc-music">
  <div class="card-h"><b>music</b> <span>Sheet Music Player</span></div>
  <div class="card-b">
    <p>Sheet music player for piezo actuators. Plays sequences of notes using the analog PWM path
    with live frequency updates for smooth playback. Includes 7 pre-built songs.</p>

<pre class="example"><span class="k">from</span> music <span class="k">import</span> play_song, play, note_freq</pre>

    <h3>play_song(name, **kwargs)</h3>
    <code class="sig">play_song(name, **kwargs)</code>
    <p class="desc">Play a named song from the built-in registry. Extra kwargs override the song's default bpm, gain, waveform, etc.</p>
<pre class="example"><span class="c"># Play Super Mario theme</span>
play_song(<span class="s">"super_mario"</span>)

<span class="c"># Play Tetris at lower volume</span>
play_song(<span class="s">"tetris"</span>, gain=<span class="n">50</span>)

<span class="c"># Play with triangle waveform</span>
play_song(<span class="s">"imperial_march"</span>, waveform=<span class="s">"triangle"</span>)</pre>

    <table class="song-table">
      <tr><th>Name</th><th>Song</th><th>BPM</th><th>Default Gain</th></tr>
      <tr><td>"super_mario"</td><td>Super Mario Bros (Koji Kondo)</td><td>400</td><td>75</td></tr>
      <tr><td>"tetris"</td><td>Korobeiniki (Russian folk)</td><td>300</td><td>75</td></tr>
      <tr><td>"happy_birthday"</td><td>Happy Birthday</td><td>160</td><td>75</td></tr>
      <tr><td>"fur_elise"</td><td>Fur Elise (Beethoven)</td><td>240</td><td>50</td></tr>
      <tr><td>"ode_to_joy"</td><td>Ode to Joy (Beethoven)</td><td>120</td><td>75</td></tr>
      <tr><td>"imperial_march"</td><td>Imperial March (John Williams)</td><td>128</td><td>100</td></tr>
      <tr><td>"nokia"</td><td>Nokia Tune (Gran Vals)</td><td>360</td><td>75</td></tr>
    </table>

    <h3>play(song, bpm=72, ...)</h3>
    <code class="sig">play(song, bpm=72, gain=100, waveform="sine", amplitude=100, staccato_ratio=0.9, loop=False)</code>
    <p class="desc">Play a custom song — a list of note tuples. Notes use variable-length tuples for backward compatibility.</p>
    <table class="params">
      <tr><th>Param</th><th>Type</th><th>Description</th></tr>
      <tr><td>song</td><td>list</td><td>List of note tuples (see format below)</td></tr>
      <tr><td>bpm</td><td>int</td><td>Tempo in beats per minute</td></tr>
      <tr><td>gain</td><td>int</td><td>DRV2665 gain: 25, 50, 75, or 100 Vpp</td></tr>
      <tr><td>waveform</td><td>str</td><td>"sine", "triangle", or "square"</td></tr>
      <tr><td>amplitude</td><td>int</td><td>Default amplitude 0–100 (when tuple omits it)</td></tr>
      <tr><td>staccato_ratio</td><td>float</td><td>Fraction of note duration to sound (0.0–1.0)</td></tr>
      <tr><td>loop</td><td>bool</td><td>If True, repeat until KeyboardInterrupt</td></tr>
    </table>

    <div class="note">
      <b>Note format</b> (variable-length tuples):<br>
      <code>("C4", 1)</code> — play at default amplitude<br>
      <code>("C4", 1, 80)</code> — play at amplitude 80<br>
      <code>("C4", 1, 80, "E4")</code> — glissando from C4 to E4<br>
      <code>("R", 1)</code> — rest (silence)<br><br>
      Note names: C, Db, D, Eb, E, F, Gb, G, Ab, A, Bb, B (octaves 2–6). Sharps also accepted: C#, D#, F#, G#, A#.
    </div>

<pre class="example"><span class="c"># Custom melody with dynamics</span>
my_song = [
    (<span class="s">"C4"</span>, <span class="n">1</span>, <span class="n">60</span>), (<span class="s">"E4"</span>, <span class="n">1</span>, <span class="n">80</span>),
    (<span class="s">"G4"</span>, <span class="n">1</span>, <span class="n">100</span>), (<span class="s">"C5"</span>, <span class="n">2</span>),
    (<span class="s">"R"</span>, <span class="n">1</span>),
    (<span class="s">"G4"</span>, <span class="n">1</span>), (<span class="s">"E4"</span>, <span class="n">1</span>), (<span class="s">"C4"</span>, <span class="n">2</span>),
]
play(my_song, bpm=<span class="n">120</span>, gain=<span class="n">75</span>)</pre>

    <h3>note_freq(name)</h3>
    <code class="sig">note_freq(name)  # returns float (Hz)</code>
    <p class="desc">Return frequency in Hz for a note name. Uses A4 = 440 Hz equal temperament.</p>
<pre class="example">note_freq(<span class="s">"A4"</span>)   <span class="c"># 440.0</span>
note_freq(<span class="s">"C4"</span>)   <span class="c"># 261.63</span>
note_freq(<span class="s">"F#3"</span>)  <span class="c"># 184.99</span></pre>
  </div>
</div>

<!-- DRV2665 -->
<div class="card drv" id="doc-drv2665">
  <div class="card-h"><b>DRV2665</b> <span>drv2665 — Piezo Driver IC (I2C)</span></div>
  <div class="card-b">
    <p>Low-level I2C interface to the DRV2665 piezo driver IC. Usually accessed through PzActuator,
    but available for direct register control. All I2C access delegates to the pz_drive C module.</p>

<pre class="example"><span class="k">from</span> drv2665 <span class="k">import</span> DRV2665
drv = DRV2665()  <span class="c"># verifies device is present</span></pre>

    <div class="note">
      <b>Gain constants:</b>
      <code>DRV2665.GAIN_25</code> (28.8 dB, 25 Vpp),
      <code>DRV2665.GAIN_50</code> (34.8 dB, 50 Vpp),
      <code>DRV2665.GAIN_75</code> (38.4 dB, 75 Vpp),
      <code>DRV2665.GAIN_100</code> (40.7 dB, 100 Vpp)
    </div>

    <h3>init_analog(gain) / init_digital(gain)</h3>
    <code class="sig">drv.init_analog(gain=DRV2665.GAIN_100)</code>
    <code class="sig">drv.init_digital(gain=DRV2665.GAIN_100)</code>
    <p class="desc">Configure the DRV2665 for analog input mode (PWM signal on IN+) or digital FIFO mode (I2C sample streaming). Sets gain and enables the output stage.</p>

    <h3>standby()</h3>
    <code class="sig">drv.standby()</code>
    <p class="desc">Enter standby mode. Disables the output stage to save power. Called automatically by <code>PzActuator.stop()</code>.</p>

    <h3>status()</h3>
    <code class="sig">drv.status()  # returns int</code>
    <p class="desc">Read the STATUS register. Bit 0 = FIFO full, bit 1 = FIFO empty.</p>
  </div>
</div>

<!-- ShiftRegister -->
<div class="card hv" id="doc-shiftreg">
  <div class="card-h"><b>ShiftRegister</b> <span>shift_register — HV509 Pin Control</span></div>
  <div class="card-b">
    <p>Driver for two daisy-chained HV509 high-voltage shift registers. Controls 20 output channels
    (pins 0–19) that route the DRV2665 output to individual piezo actuators. SPI writes are
    synchronized to waveform events (zero-crossing in fullwave mode, cycle start otherwise) via a
    stage/latch mechanism.</p>

<pre class="example"><span class="k">from</span> shift_register <span class="k">import</span> ShiftRegister
sr = ShiftRegister()

<span class="c"># Or access via PzActuator</span>
pa.shift_register.set_pin(<span class="n">0</span>, <span class="k">True</span>)</pre>

    <h3>set_pin(pin, value, latch=True)</h3>
    <code class="sig">sr.set_pin(pin, value, latch=True)</code>
    <p class="desc">Set a single pin (0–19) on or off. If latch=True (default), immediately commits the change via SPI. If latch=False, stages the change for later commit with latch().</p>
<pre class="example"><span class="c"># Enable pin 5</span>
sr.set_pin(<span class="n">5</span>, <span class="k">True</span>)

<span class="c"># Batch updates (faster — one SPI write)</span>
sr.set_pin(<span class="n">0</span>, <span class="k">True</span>, latch=<span class="k">False</span>)
sr.set_pin(<span class="n">1</span>, <span class="k">True</span>, latch=<span class="k">False</span>)
sr.set_pin(<span class="n">2</span>, <span class="k">True</span>, latch=<span class="k">False</span>)
sr.latch()  <span class="c"># commit all at once</span></pre>

    <h3>get_pin(pin)</h3>
    <code class="sig">sr.get_pin(pin)  # returns bool</code>
    <p class="desc">Read the current state of a pin (0–19) from the Python-side buffer.</p>

    <h3>set_all(value, latch=True)</h3>
    <code class="sig">sr.set_all(value, latch=True)</code>
    <p class="desc">Set all 20 pins to the same value (True = on, False = off).</p>
<pre class="example"><span class="c"># Enable all channels</span>
sr.set_all(<span class="k">True</span>)

<span class="c"># Disable all channels</span>
sr.set_all(<span class="k">False</span>)</pre>

    <h3>get_all()</h3>
    <code class="sig">sr.get_all()  # returns tuple of 20 ints (0 or 1)</code>
    <p class="desc">Read the state of all 20 pins as a tuple.</p>

    <h3>set_pins(values, latch=True)</h3>
    <code class="sig">sr.set_pins(values, latch=True)</code>
    <p class="desc">Set all pins from a list/tuple of values. Length up to 20. Each value is truthy/falsy.</p>
<pre class="example"><span class="c"># Set specific pattern</span>
sr.set_pins([<span class="n">1</span>,<span class="n">0</span>,<span class="n">1</span>,<span class="n">0</span>,<span class="n">1</span>,<span class="n">0</span>,<span class="n">1</span>,<span class="n">0</span>,<span class="n">1</span>,<span class="n">0</span>,
             <span class="n">1</span>,<span class="n">0</span>,<span class="n">1</span>,<span class="n">0</span>,<span class="n">1</span>,<span class="n">0</span>,<span class="n">1</span>,<span class="n">0</span>,<span class="n">1</span>,<span class="n">0</span>])</pre>

    <h3>latch()</h3>
    <code class="sig">sr.latch()</code>
    <p class="desc">Commit pending pin changes via SPI. If the ISR/FIFO task is running, the write is synchronized to the next waveform event. If stopped, writes immediately.</p>
  </div>
</div>

</body>
</html>
```

**Step 2: Commit**

```bash
git add web/docs.html
git commit -m "feat(web): add API reference documentation page"
```

---

### Task 2: Add "API Docs" link to terminal card in index.html

**Files:**
- Modify: `web/index.html:267-269`

**Step 1: Add the docs link**

In the terminal toggle bar, add a link between the "Terminal" label and the chevron. Replace lines 267-269:

```html
  <div class="term-toggle" id="term-toggle" onclick="toggleTerm()">
    <b>Terminal</b>
    <span class="chev">&#9660;</span>
  </div>
```

With:

```html
  <div class="term-toggle" id="term-toggle" onclick="toggleTerm()">
    <b>Terminal</b>
    <span style="display:flex;align-items:center;gap:8px">
      <a href="/web/docs.html" target="_blank" id="docs-link"
         onclick="event.stopPropagation()"
         style="font-size:11px;color:var(--esp);text-decoration:none;font-weight:600;
                padding:2px 6px;border:1px solid var(--esp);border-radius:4px">API Docs</a>
      <span class="chev">&#9660;</span>
    </span>
  </div>
```

Note: `event.stopPropagation()` prevents the link click from also toggling the terminal panel.

**Step 2: Commit**

```bash
git add web/index.html
git commit -m "feat(web): add API Docs link to terminal card"
```

---

### Task 3: Add docs.html existence check to CI

**Files:**
- Modify: `.github/workflows/ci.yml:13-18`

**Step 1: Add a check step to the lint-python job**

After the existing ruff step in the `lint-python` job, add:

```yaml
      - name: Check docs exist
        run: test -f web/docs.html
```

The full `lint-python` job should look like:

```yaml
  lint-python:
    name: Lint Python
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: astral-sh/ruff-action@v3
      - name: Check docs exist
        run: test -f web/docs.html
```

**Step 2: Commit**

```bash
git add .github/workflows/ci.yml
git commit -m "ci: add docs.html existence check"
```

---

### Task 4: Update flash skill to upload docs.html

**Files:**
- Modify: `.claude/skills/flash/SKILL.md`

**Step 1: Add web/ files to upload commands**

In the "Fast Path" section (line 38), change the mpremote command to also copy web files:

Before:
```bash
mpremote cp python/music.py python/web_server.py python/wifi.py :
```

After:
```bash
mpremote cp python/music.py python/web_server.py python/wifi.py :
mpremote cp web/index.html web/wifi.html web/docs.html :web/
```

In "Step 4: Upload filesystem Python files" (line 82-84), add the same web file copy:

Before:
```bash
mpremote cp python/music.py python/web_server.py python/wifi.py :
```

After:
```bash
mpremote cp python/music.py python/web_server.py python/wifi.py :
mpremote cp web/index.html web/wifi.html web/docs.html :web/
```

**Step 2: Commit**

```bash
git add .claude/skills/flash/SKILL.md
git commit -m "docs(flash): add web/ files to upload steps"
```

---

### Task 5: Add filesystem artifact to CI

**Files:**
- Modify: `.github/workflows/ci.yml`

**Step 1: Add a filesystem artifact upload step**

After the existing "Upload firmware" step in the `build` job, add a new step:

```yaml
      - name: Upload filesystem
        uses: actions/upload-artifact@v4
        with:
          name: filesystem
          path: |
            python/music.py
            python/web_server.py
            python/wifi.py
            web/
```

This creates a separate `filesystem` artifact alongside the existing `firmware` artifact. The filesystem artifact contains the non-frozen Python modules and all web files that need to be uploaded to the board's filesystem via `mpremote cp`.

**Step 2: Commit**

```bash
git add .github/workflows/ci.yml
git commit -m "ci: add filesystem artifact with Python + web files"
```

---

### Task 6: Upload and test

**Step 1: Upload docs.html to board**

```bash
mpremote cp web/docs.html :web/docs.html
```

**Step 2: Upload updated index.html to board**

```bash
mpremote cp web/index.html :web/index.html
```

**Step 3: Test in browser**

Open the web UI. Verify:
- The terminal card shows an "API Docs" link next to the chevron
- Clicking "API Docs" opens `/web/docs.html` in a new tab
- Clicking the terminal toggle bar still opens/closes the terminal (link click doesn't toggle)
- The docs page loads with all four module cards
- The "Control Panel" back link returns to the main UI
- Code examples are readable with dark background styling
