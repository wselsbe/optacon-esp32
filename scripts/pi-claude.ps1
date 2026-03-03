# Pi Claude — SSH into optacon-pi tmux session with bidirectional clipboard sync.
# Double-click pi-claude.cmd to launch.
#
# Modes (internal):
#   -ClipboardWatcher   Windows→Pi: listens for clipboard changes, POSTs to Pi:8224
#   -ClipboardServer    Pi→Windows: HTTP listener on :8225, sets Windows clipboard

param(
    [switch]$ClipboardWatcher,
    [switch]$ClipboardServer
)

# --- Windows → Pi: clipboard watcher --------------------------------------

if ($ClipboardWatcher) {
    Add-Type -AssemblyName System.Windows.Forms
    Add-Type -AssemblyName System.Drawing

    Add-Type -ReferencedAssemblies System.Windows.Forms, System.Drawing -TypeDefinition @"
using System;
using System.Drawing;
using System.Drawing.Imaging;
using System.IO;
using System.Net;
using System.Runtime.InteropServices;
using System.Security.Cryptography;
using System.Text;
using System.Windows.Forms;

public class ClipboardWatcherForm : Form {
    [DllImport("user32.dll", SetLastError = true)]
    [return: MarshalAs(UnmanagedType.Bool)]
    static extern bool AddClipboardFormatListener(IntPtr hwnd);

    [DllImport("user32.dll", SetLastError = true)]
    [return: MarshalAs(UnmanagedType.Bool)]
    static extern bool RemoveClipboardFormatListener(IntPtr hwnd);

    const int WM_CLIPBOARDUPDATE = 0x031D;
    string lastHash = "";
    string url = "http://localhost:8224/clipboard";

    public ClipboardWatcherForm() {
        ShowInTaskbar = false;
        FormBorderStyle = FormBorderStyle.None;
        Size = new Size(1, 1);
        Opacity = 0;
        AddClipboardFormatListener(Handle);
    }

    protected override void WndProc(ref Message m) {
        if (m.Msg == WM_CLIPBOARDUPDATE) OnClipboardChanged();
        base.WndProc(ref m);
    }

    string ComputeHash(byte[] data) {
        using (var sha = SHA256.Create())
            return BitConverter.ToString(sha.ComputeHash(data));
    }

    void OnClipboardChanged() {
        try {
            if (Clipboard.ContainsImage()) {
                using (var img = Clipboard.GetImage())
                using (var ms = new MemoryStream()) {
                    if (img == null) return;
                    img.Save(ms, ImageFormat.Png);
                    var bytes = ms.ToArray();
                    var hash = ComputeHash(bytes);
                    if (hash == lastHash) return;
                    lastHash = hash;
                    Post(bytes, "image/png");
                }
            } else if (Clipboard.ContainsText()) {
                var text = Clipboard.GetText();
                if (string.IsNullOrEmpty(text)) return;
                var bytes = Encoding.UTF8.GetBytes(text);
                var hash = ComputeHash(bytes);
                if (hash == lastHash) return;
                lastHash = hash;
                Post(bytes, "text/plain;charset=utf-8");
            }
        } catch {}
    }

    void Post(byte[] body, string contentType) {
        try {
            var req = (HttpWebRequest)WebRequest.Create(url);
            req.Method = "POST";
            req.ContentType = contentType;
            req.Timeout = 3000;
            req.ContentLength = body.Length;
            using (var s = req.GetRequestStream()) s.Write(body, 0, body.Length);
            using (req.GetResponse()) {}
        } catch {}
    }

    protected override void OnFormClosed(FormClosedEventArgs e) {
        RemoveClipboardFormatListener(Handle);
        base.OnFormClosed(e);
    }
}
"@

    $form = New-Object ClipboardWatcherForm
    [System.Windows.Forms.Application]::Run($form)
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
        "--window-size=1280,900" `
        -WindowStyle Minimized
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
            optacon-pi 'n=0; while ! tmux has-session -t claude 2>/dev/null; do sleep 1; n=$((n+1)); if [ $n -ge 30 ]; then echo "Timeout waiting for claude session"; exit 1; fi; done; exec tmux attach -t claude'
        Write-Host ""
        Write-Host "Disconnected. Reconnecting in 3 seconds... (Ctrl+C to quit)"
        Start-Sleep -Seconds 3
    }
} finally {
    foreach ($p in @($watcherProc, $serverProc)) {
        if (!$p.HasExited) { Stop-Process -Id $p.Id -ErrorAction SilentlyContinue }
    }
}
