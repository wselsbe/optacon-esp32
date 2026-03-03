import json
import time

import network

HOSTNAME = "esp-optacon"
STA_TIMEOUT = 10  # seconds to wait for station connection
AP_SSID = "esp-optacon"
AP_PASSWORD = "123456"

ip = None
mode = None
ssid = None


def connect():
    """Try STA mode from wifi_config.json, fall back to AP. Sets mDNS hostname."""
    global ip, mode, ssid
    network.hostname(HOSTNAME)

    # Try station mode
    try:
        with open("wifi_config.json") as f:
            cfg = json.load(f)
        sta = network.WLAN(network.STA_IF)
        sta.active(True)
        sta.connect(cfg["ssid"], cfg.get("password", ""))
        for _ in range(STA_TIMEOUT * 10):
            if sta.isconnected():
                ip = sta.ifconfig()[0]
                mode = "sta"
                ssid = cfg["ssid"]
                print("WiFi STA connected:", ip)
                print("Web UI: http://" + HOSTNAME + ".local")
                return ip, mode, ssid
            time.sleep_ms(100)
        sta.active(False)
        print("WiFi STA failed, starting AP")
    except OSError:
        print("No wifi_config.json, starting AP")

    # Fallback: access point
    ap = network.WLAN(network.AP_IF)
    ap.active(True)
    ap.config(essid=AP_SSID, password=AP_PASSWORD)
    ip = ap.ifconfig()[0]
    mode = "ap"
    ssid = AP_SSID
    print("WiFi AP started:", AP_SSID, "IP:", ip)
    print("Web UI: http://" + HOSTNAME + ".local")
    return ip, mode, ssid


def get_status():
    """Return WiFi status dict."""
    return {
        "mode": mode,
        "ssid": ssid,
        "ip": ip,
        "hostname": HOSTNAME + ".local",
    }


def reconnect():
    """Disconnect current WiFi and reconnect with saved config."""
    sta = network.WLAN(network.STA_IF)
    sta.disconnect()
    sta.active(False)
    ap = network.WLAN(network.AP_IF)
    ap.active(False)
    return connect()


def save_config(ssid, password=""):
    """Write WiFi credentials to wifi_config.json."""
    with open("wifi_config.json", "w") as f:
        json.dump({"ssid": ssid, "password": password}, f)
