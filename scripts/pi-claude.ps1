# Pi Claude — SSH into optacon-pi tmux session with clipboard sync.
# Double-click to launch (or right-click > Run with PowerShell).

$Host.UI.RawUI.WindowTitle = "Claude Code (optacon-pi)"

Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing

# --- Clipboard watcher (background job) -----------------------------------

$clipJob = Start-Job -ScriptBlock {
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
}

Write-Host "Clipboard sync started (background)" -ForegroundColor DarkGray

# --- SSH reconnect loop ----------------------------------------------------

try {
    while ($true) {
        ssh -t -R 9222:127.0.0.1:9222 -L 8224:127.0.0.1:8224 optacon-pi "tmux attach -t claude"
        Write-Host ""
        Write-Host "Connection lost. Reconnecting in 10 seconds... (Ctrl+C to quit)"
        Start-Sleep -Seconds 10
    }
} finally {
    Stop-Job $clipJob -ErrorAction SilentlyContinue
    Remove-Job $clipJob -Force -ErrorAction SilentlyContinue
}
