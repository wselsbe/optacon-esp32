# Pi Claude — SSH into optacon-pi tmux session with bidirectional clipboard sync.
# Double-click pi-claude.cmd to launch.
#
# Modes (internal):
#   -ClipboardWatcher   Windows→Pi: polls Windows clipboard, POSTs to Pi:8224
#   -ClipboardServer    Pi→Windows: HTTP listener on :8225, sets Windows clipboard

param(
    [switch]$ClipboardWatcher,
    [switch]$ClipboardServer
)

# --- Windows → Pi: clipboard watcher --------------------------------------

if ($ClipboardWatcher) {
    Add-Type -AssemblyName System.Windows.Forms
    Add-Type -AssemblyName System.Drawing

    $lastHash = ""
    $url = "http://localhost:8224/clipboard"

    function Get-ClipHash($bytes) {
        $sha = [System.Security.Cryptography.SHA256]::Create()
        [System.BitConverter]::ToString($sha.ComputeHash($bytes))
    }

    while ($true) {
        Start-Sleep -Milliseconds 500
        try {
            $img = [System.Windows.Forms.Clipboard]::GetImage()
            if ($img) {
                $ms = New-Object System.IO.MemoryStream
                $img.Save($ms, [System.Drawing.Imaging.ImageFormat]::Png)
                $bytes = $ms.ToArray()
                $ms.Dispose()
                $img.Dispose()
                $hash = Get-ClipHash $bytes
                if ($hash -ne $lastHash) {
                    $lastHash = $hash
                    Invoke-WebRequest -Uri $url -Method POST -Body $bytes `
                        -ContentType "image/png" -TimeoutSec 3 | Out-Null
                }
                continue
            }
            $text = [System.Windows.Forms.Clipboard]::GetText()
            if ($text) {
                $bytes = [System.Text.Encoding]::UTF8.GetBytes($text)
                $hash = Get-ClipHash $bytes
                if ($hash -ne $lastHash) {
                    $lastHash = $hash
                    Invoke-WebRequest -Uri $url -Method POST -Body $bytes `
                        -ContentType "text/plain;charset=utf-8" -TimeoutSec 3 | Out-Null
                }
            }
        } catch {}
    }
    return
}

# --- Pi → Windows: clipboard server ---------------------------------------

if ($ClipboardServer) {
    Add-Type -AssemblyName System.Windows.Forms
    Add-Type -AssemblyName System.Drawing

    $listener = New-Object System.Net.HttpListener
    $listener.Prefixes.Add("http://127.0.0.1:8225/")
    $listener.Start()

    while ($listener.IsListening) {
        try {
            $ctx = $listener.GetContext()
            $len = [int]$ctx.Request.ContentLength64
            $buf = New-Object byte[] $len
            $offset = 0
            while ($offset -lt $len) {
                $offset += $ctx.Request.InputStream.Read($buf, $offset, $len - $offset)
            }
            $ct = $ctx.Request.ContentType

            if ($ct -match "^image/") {
                $ms = New-Object System.IO.MemoryStream(, $buf)
                $img = [System.Drawing.Image]::FromStream($ms)
                [System.Windows.Forms.Clipboard]::SetImage($img)
                $img.Dispose()
                $ms.Dispose()
            } else {
                $text = [System.Text.Encoding]::UTF8.GetString($buf)
                [System.Windows.Forms.Clipboard]::SetText($text)
            }

            $ctx.Response.StatusCode = 200
            $ctx.Response.Close()
        } catch {}
    }
    return
}

# --- Main mode: launch helpers + SSH ---------------------------------------

$Host.UI.RawUI.WindowTitle = "Claude Code (optacon-pi)"

# Launch Chrome with remote debugging (reuse existing if already running)
$chrome = Get-Process chrome -ErrorAction SilentlyContinue |
    Where-Object { $_.CommandLine -match "remote-debugging-port" }
if (-not $chrome) {
    Start-Process "C:\Program Files\Google\Chrome\Application\chrome.exe" -ArgumentList `
        "--remote-debugging-port=9222",
        "--user-data-dir=$env:TEMP\chrome-debug",
        "--no-first-run",
        "--no-default-browser-check",
        "--window-size=1280,900"
    Write-Host "Chrome CDP started (port 9222)" -ForegroundColor DarkGray
} else {
    Write-Host "Chrome CDP already running" -ForegroundColor DarkGray
}

$watcherProc = Start-Process powershell -ArgumentList `
    "-NoProfile -WindowStyle Hidden -Sta -File `"$PSCommandPath`" -ClipboardWatcher" `
    -PassThru -WindowStyle Hidden

$serverProc = Start-Process powershell -ArgumentList `
    "-NoProfile -WindowStyle Hidden -Sta -File `"$PSCommandPath`" -ClipboardServer" `
    -PassThru -WindowStyle Hidden

Write-Host "Clipboard sync started (watcher=$($watcherProc.Id), server=$($serverProc.Id))" `
    -ForegroundColor DarkGray

try {
    while ($true) {
        ssh -t `
            -R 9222:127.0.0.1:9222 `
            -L 8224:127.0.0.1:8224 `
            -R 8225:127.0.0.1:8225 `
            optacon-pi "tmux attach -t claude"
        Write-Host ""
        Write-Host "Connection lost. Reconnecting in 10 seconds... (Ctrl+C to quit)"
        Start-Sleep -Seconds 10
    }
} finally {
    foreach ($p in @($watcherProc, $serverProc)) {
        if (!$p.HasExited) { Stop-Process -Id $p.Id -ErrorAction SilentlyContinue }
    }
}
