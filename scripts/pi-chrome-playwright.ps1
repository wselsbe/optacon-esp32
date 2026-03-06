# Pi Chrome Playwright - Start SSH tunnel (port 9222) and Chrome CDP for remote Playwright.
# Double-click pi-chrome-playwright.cmd to launch.
# Safe to run multiple times: PID files prevent duplicate instances.

$Host.UI.RawUI.WindowTitle = "Chrome Playwright (optacon-pi)"

$tunnelPidFile = "$PSScriptRoot\ssh-tunnel.pid"
$chromePidFile = "$PSScriptRoot\chrome-cdp.pid"

# --- SSH tunnel (port 9222 reverse) -----------------------------------------

$tunnelRunning = $false
if (Test-Path $tunnelPidFile) {
    $savedTunnelPid = [int](Get-Content $tunnelPidFile -ErrorAction SilentlyContinue)
    if ($savedTunnelPid -and (Get-Process -Id $savedTunnelPid -ErrorAction SilentlyContinue)) {
        $tunnelRunning = $true
    }
}

if (-not $tunnelRunning) {
    # Start tunnel with stderr captured so we can detect bind failures
    $stderrFile = "$env:TEMP\ssh-tunnel-stderr.txt"
    $tunnelProc = Start-Process ssh -ArgumentList `
        "-N -o ExitOnForwardFailure=yes -R 9222:127.0.0.1:9222 optacon-pi" `
        -PassThru -WindowStyle Hidden `
        -RedirectStandardError $stderrFile

    # Wait for tunnel to establish or fail
    Start-Sleep -Seconds 3

    if ($tunnelProc.HasExited) {
        $stderr = Get-Content $stderrFile -Raw -ErrorAction SilentlyContinue
        Write-Host ""
        Write-Host "ERROR: SSH tunnel failed to start." -ForegroundColor Red
        Write-Host ""
        if ($stderr -match "remote port forwarding failed") {
            # Query who's holding the port
            $portInfo = ssh optacon-pi 'bash ~/projects/optacon-firmware/scripts/pi-check-tunnel-port.sh 9222' 2>$null
            if ($portInfo -match "^(\d+)\s+(\S+)") {
                $remotePid = $Matches[1]
                $remoteIp = $Matches[2]
                $myIp = (ssh optacon-pi 'echo $SSH_CLIENT' 2>$null) -replace '\s.*', ''

                Write-Host "Port 9222 on optacon-pi is held by SSH from $remoteIp (sshd pid=$remotePid)" -ForegroundColor Red
                if ($remoteIp -ne $myIp) {
                    Write-Host "This is a DIFFERENT machine (you are $myIp)." -ForegroundColor Yellow
                    Write-Host ""
                    $answer = Read-Host "Kill the remote tunnel? [y/N]"
                    if ($answer -eq 'y') {
                        ssh optacon-pi 'bash ~/projects/optacon-firmware/scripts/pi-check-tunnel-port.sh 9222 --kill' 2>$null | Out-Null
                        Write-Host "Killed. Re-run this script to start the tunnel." -ForegroundColor Green
                    }
                } else {
                    Write-Host "This is from YOUR machine - a stale tunnel." -ForegroundColor Yellow
                    Write-Host ""
                    $answer = Read-Host "Kill the stale tunnel? [y/N]"
                    if ($answer -eq 'y') {
                        ssh optacon-pi 'bash ~/projects/optacon-firmware/scripts/pi-check-tunnel-port.sh 9222 --kill' 2>$null | Out-Null
                        Write-Host "Killed. Re-run this script to start the tunnel." -ForegroundColor Green
                    }
                }
            } else {
                Write-Host "Port 9222 is already in use on optacon-pi (could not identify holder)." -ForegroundColor Red
            }
        } else {
            Write-Host "SSH stderr:" -ForegroundColor Red
            Write-Host $stderr -ForegroundColor Red
        }
        Write-Host ""
        Write-Host "Press any key to exit..."
        $null = $Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")
        exit 1
    }

    $tunnelProc.Id | Set-Content $tunnelPidFile
    Write-Host "SSH tunnel started (port 9222, pid=$($tunnelProc.Id))" -ForegroundColor Green
} else {
    Write-Host "SSH tunnel already running (pid=$savedTunnelPid)" -ForegroundColor DarkGray
}

# --- Chrome CDP --------------------------------------------------------------

$chromeRunning = $false
if (Test-Path $chromePidFile) {
    $savedPid = [int](Get-Content $chromePidFile -ErrorAction SilentlyContinue)
    if ($savedPid -and (Get-Process -Id $savedPid -ErrorAction SilentlyContinue)) {
        $chromeRunning = $true
    }
}

if (-not $chromeRunning) {
    $chromeProc = Start-Process "C:\Program Files\Google\Chrome\Application\chrome.exe" -ArgumentList `
        "--remote-debugging-port=9222",
        "--user-data-dir=$env:TEMP\chrome-debug",
        "--no-first-run",
        "--no-default-browser-check",
        "--window-size=1280,900" `
        -PassThru -WindowStyle Minimized
    $chromeProc.Id | Set-Content $chromePidFile
    Write-Host "Chrome CDP started (port 9222, pid=$($chromeProc.Id))" -ForegroundColor Green
} else {
    Write-Host "Chrome CDP already running (pid=$savedPid)" -ForegroundColor DarkGray
}

Write-Host ""
Write-Host "Playwright remote ready. Close this window or Ctrl+C to leave running." -ForegroundColor Cyan
Write-Host "Chrome and tunnel will keep running in the background." -ForegroundColor DarkGray
Write-Host ""
Write-Host "Press any key to exit..."
$null = $Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")
