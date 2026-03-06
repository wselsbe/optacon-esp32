"""Playwright UI tests -- all JavaScript behavior in index.html."""

import re

import pytest
from playwright.sync_api import Page, expect


def _goto_no_ws(page: Page, url: str):
    """Navigate to page and block WebSocket so state updates don't interfere."""
    # Override WebSocket before the page's JS runs, so connect() is a no-op
    page.add_init_script("window.WebSocket = function(){this.readyState=3};")
    page.goto(url)
    # Wait for JS to execute (pin grid creation, etc.)
    page.locator("#pin-grid .pin-btn").first.wait_for(timeout=3000)


# ---------------------------------------------------------------------------
# Tab Switching
# ---------------------------------------------------------------------------


def test_tabs_exist(page: Page, page_url: str):
    """All 3 tab buttons are visible."""
    page.goto(page_url)
    expect(page.locator("#tab-btn-signal")).to_be_visible()
    expect(page.locator("#tab-btn-music")).to_be_visible()
    expect(page.locator("#tab-btn-speech")).to_be_visible()


def test_signal_tab_active_by_default(page: Page, page_url: str):
    """Signal tab is selected by default, its panel visible, others hidden."""
    page.goto(page_url)
    expect(page.locator("#tab-btn-signal")).to_have_class(re.compile(r"sel"))
    expect(page.locator("#tab-signal")).to_be_visible()
    expect(page.locator("#tab-music")).to_be_hidden()
    expect(page.locator("#tab-speech")).to_be_hidden()


def test_switch_to_music_tab(page: Page, page_url: str):
    """Clicking Music tab selects it and shows its panel."""
    page.goto(page_url)
    page.click("#tab-btn-music")
    expect(page.locator("#tab-btn-music")).to_have_class(re.compile(r"sel"))
    expect(page.locator("#tab-btn-signal")).not_to_have_class(re.compile(r"sel"))
    expect(page.locator("#tab-music")).to_be_visible()
    expect(page.locator("#tab-signal")).to_be_hidden()


def test_switch_to_speech_tab(page: Page, page_url: str):
    """Clicking Speech tab selects it and shows its panel."""
    page.goto(page_url)
    page.click("#tab-btn-speech")
    expect(page.locator("#tab-btn-speech")).to_have_class(re.compile(r"sel"))
    expect(page.locator("#tab-btn-signal")).not_to_have_class(re.compile(r"sel"))
    expect(page.locator("#tab-speech")).to_be_visible()
    expect(page.locator("#tab-signal")).to_be_hidden()


# ---------------------------------------------------------------------------
# Waveform Selector
# ---------------------------------------------------------------------------


def test_waveform_buttons_exist(page: Page, page_url: str):
    """All 4 waveform buttons are visible."""
    page.goto(page_url)
    for wf_id in ("#wf-dc", "#wf-sine", "#wf-triangle", "#wf-square"):
        expect(page.locator(wf_id)).to_be_visible()


def test_sine_selected_by_default(page: Page, page_url: str):
    """Sine waveform is selected by default."""
    page.goto(page_url)
    expect(page.locator("#wf-sine")).to_have_class(re.compile(r"sel"))
    expect(page.locator("#wf-dc")).not_to_have_class(re.compile(r"sel"))


def test_waveform_per_tab_memory(page: Page, page_url: str):
    """Each tab remembers its waveform selection independently."""
    _goto_no_ws(page, page_url)
    # Signal tab: select triangle (use evaluate to ensure onclick fires)
    page.evaluate("document.getElementById('wf-triangle').click()")
    expect(page.locator("#wf-triangle")).to_have_class(re.compile(r"sel"))

    # Switch to Music, select square
    page.locator("#tab-btn-music").click()
    page.evaluate("document.getElementById('wf-square').click()")
    expect(page.locator("#wf-square")).to_have_class(re.compile(r"sel"))

    # Switch back to Signal -- triangle should be remembered
    page.locator("#tab-btn-signal").click()
    expect(page.locator("#wf-triangle")).to_have_class(re.compile(r"sel"))
    expect(page.locator("#wf-square")).not_to_have_class(re.compile(r"sel"))

    # Switch back to Music -- square should be remembered
    page.locator("#tab-btn-music").click()
    expect(page.locator("#wf-square")).to_have_class(re.compile(r"sel"))


def test_dc_disabled_on_music_tab(page: Page, page_url: str):
    """DC button is disabled on Music tab, sine is pre-selected."""
    _goto_no_ws(page, page_url)
    page.locator("#tab-btn-music").click()
    # DC is visually disabled and not interactive
    expect(page.locator("#wf-dc")).to_have_attribute(
        "style", re.compile(r"pointer-events:\s*none")
    )
    # DC is not the selected waveform on music tab
    expect(page.locator("#wf-dc")).not_to_have_class(re.compile(r"sel"))
    # Sine is selected by default on music tab
    expect(page.locator("#wf-sine")).to_have_class(re.compile(r"sel"))


def test_speech_tab_waveform_disabled(page: Page, page_url: str):
    """All waveform buttons are non-interactive on Speech tab, square pre-selected."""
    page.goto(page_url)
    page.locator("#tab-btn-speech").click()
    wf_btns = page.locator(".wf-btn")
    for i in range(wf_btns.count()):
        expect(wf_btns.nth(i)).to_have_attribute(
            "style", re.compile(r"pointer-events:\s*none")
        )
    expect(page.locator("#wf-square")).to_have_class(re.compile(r"sel"))


# ---------------------------------------------------------------------------
# Signal Tab
# ---------------------------------------------------------------------------


def test_frequency_input_and_slider_sync(page: Page, page_url: str):
    """Typing a frequency value syncs with the slider on change."""
    _goto_no_ws(page, page_url)
    # Use evaluate to set value and fire change event (fill + dispatchEvent
    # doesn't reliably trigger the onchange handler for number inputs)
    page.evaluate("""
        var el = document.getElementById('freq');
        el.value = '300';
        el.onchange();
    """)
    expect(page.locator("#freq-slider")).to_have_value("300")


def test_amplitude_slider_updates_label(page: Page, page_url: str):
    """Moving the amplitude slider updates the label text."""
    page.goto(page_url)
    # Use evaluate to set slider value and fire oninput handler
    page.evaluate("""
        var el = document.getElementById('amplitude');
        el.value = '75';
        el.oninput();
    """)
    expect(page.locator("#amp-val")).to_have_text("75%")


# ---------------------------------------------------------------------------
# DRV2665 Card
# ---------------------------------------------------------------------------


def test_gain_buttons_exist(page: Page, page_url: str):
    """All 4 gain buttons are visible."""
    page.goto(page_url)
    for gid in ("#gain-25", "#gain-50", "#gain-75", "#gain-100"):
        expect(page.locator(gid)).to_be_visible()


def test_gain_100_selected_by_default(page: Page, page_url: str):
    """100 Vpp gain is selected by default."""
    page.goto(page_url)
    expect(page.locator("#gain-100")).to_have_class(re.compile(r"sel"))


def test_gain_button_click_selects(page: Page, page_url: str):
    """Clicking a gain button selects it and deselects others."""
    page.goto(page_url)
    page.click("#gain-50")
    expect(page.locator("#gain-50")).to_have_class(re.compile(r"sel"))
    expect(page.locator("#gain-100")).not_to_have_class(re.compile(r"sel"))


def test_start_button_label_per_tab(page: Page, page_url: str):
    """Start button label changes per active tab."""
    page.goto(page_url)
    start_btn = page.locator("#start-stop")

    # Signal tab
    expect(start_btn).to_have_text("START")

    # Speech tab
    page.click("#tab-btn-speech")
    expect(start_btn).to_have_text("SPEAK")

    # Music tab
    page.click("#tab-btn-music")
    expect(start_btn).to_have_text("PLAY")


# ---------------------------------------------------------------------------
# Pin Grid
# ---------------------------------------------------------------------------


def test_pin_grid_has_20_buttons(page: Page, page_url: str):
    """Pin grid contains exactly 20 pin buttons."""
    page.goto(page_url)
    expect(page.locator(".pin-btn")).to_have_count(20)


def test_set_all_clear_all_buttons(page: Page, page_url: str):
    """SET ALL and CLEAR ALL buttons are visible."""
    page.goto(page_url)
    expect(page.locator("#set-all")).to_be_visible()
    expect(page.locator("#clear-all")).to_be_visible()


# ---------------------------------------------------------------------------
# Speech Tab
# ---------------------------------------------------------------------------


def test_speech_text_input(page: Page, page_url: str):
    """Speech text input accepts and retains text."""
    page.goto(page_url)
    page.click("#tab-btn-speech")
    page.fill("#tts-text", "Hello world")
    expect(page.locator("#tts-text")).to_have_value("Hello world")


def test_speech_speed_pitch_sliders(page: Page, page_url: str):
    """Speed and pitch sliders are visible on Speech tab."""
    page.goto(page_url)
    page.click("#tab-btn-speech")
    expect(page.locator("#tts-speed")).to_be_visible()
    expect(page.locator("#tts-pitch")).to_be_visible()


# ---------------------------------------------------------------------------
# Music Tab
# ---------------------------------------------------------------------------


def test_music_notes_textarea(page: Page, page_url: str):
    """Music notes textarea accepts and retains text."""
    page.goto(page_url)
    page.click("#tab-btn-music")
    page.fill("#music-notes", "C4:4 D4:4 E4:4")
    expect(page.locator("#music-notes")).to_have_value("C4:4 D4:4 E4:4")


def test_music_bpm_input(page: Page, page_url: str):
    """BPM input accepts and retains a value."""
    page.goto(page_url)
    page.click("#tab-btn-music")
    bpm = page.locator("#music-bpm")
    bpm.fill("140")
    expect(bpm).to_have_value("140")


# ---------------------------------------------------------------------------
# Terminal
# ---------------------------------------------------------------------------


def test_terminal_toggle(page: Page, page_url: str):
    """Clicking terminal toggle adds 'open' class."""
    page.goto(page_url)
    term = page.locator("#term")
    expect(term).not_to_have_class(re.compile(r"open"))
    page.click("#term-toggle")
    expect(term).to_have_class(re.compile(r"open"))


# ---------------------------------------------------------------------------
# Header
# ---------------------------------------------------------------------------


def _assert_brand(page: Page):
    """Assert OPTACON brand text is visible (shared across all pages)."""
    expect(page.locator(".brand")).to_have_text("OPTACON")


def test_header_elements(page: Page, page_url: str):
    """Header has OPTACON brand text, badge, and WiFi link."""
    page.goto(page_url)
    _assert_brand(page)
    expect(page.locator("#badge")).to_be_visible()
    expect(page.locator("#wifi-link")).to_be_visible()


# ---------------------------------------------------------------------------
# Footer
# ---------------------------------------------------------------------------


def test_footer_visible(page: Page, page_url: str):
    """Footer is visible."""
    page.goto(page_url)
    expect(page.locator("#footer")).to_be_visible()


# ---------------------------------------------------------------------------
# WiFi Page
# ---------------------------------------------------------------------------


def test_wifi_page_loads(page: Page, page_url: str):
    """WiFi page loads with brand and footer."""
    page.goto(page_url + "/wifi")
    _assert_brand(page)
    expect(page.locator("#footer")).to_be_visible()


# ---------------------------------------------------------------------------
# Update Page
# ---------------------------------------------------------------------------


def test_update_page_loads(page: Page, page_url: str):
    """Update page loads with brand and footer."""
    page.goto(page_url + "/update")
    _assert_brand(page)
    expect(page.locator(".footer")).to_be_visible()


# ---------------------------------------------------------------------------
# Docs Page
# ---------------------------------------------------------------------------


def test_docs_page_loads(page: Page, page_url: str):
    """Docs page loads with brand and footer."""
    page.goto(page_url + "/web/docs.html")
    _assert_brand(page)
    expect(page.locator(".footer")).to_be_visible()
