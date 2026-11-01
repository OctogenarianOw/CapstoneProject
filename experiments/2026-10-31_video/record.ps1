# Record the screen while an RViz window of WSLg is open, one file per
# window, for the video of the filmed missions (shoot.sh).
#
# Role: WSLg windows are composited by Windows, so ffmpeg's gdigrab of
# the window itself captures black (tested 31 Oct); the desktop capture
# shows them. This script waits for an RViz window, maximises it and
# brings it to the front, measures the offset between the WSL clock
# (which stamps the mission log) and the Windows clock (which stamps the
# recording), records the desktop until the window closes, then stops
# ffmpeg cleanly through its stdin. A JSON next to each recording holds
# the Windows time of its start and the clock offset, so that compose.py
# can put the mission's own margin curve in step with the picture.
# The display is kept on while recording (SetThreadExecutionState).
#
#   powershell -File record.ps1 -FFmpeg <ffmpeg.exe> -OutDir <dir> -Until 21:00
#   -Title '*Gazebo*' records the Gazebo GUI window instead (gazebo_follow)
param([string]$FFmpeg, [string]$OutDir, [string]$Until = '21:00', [string]$Title = '*RViz*', [int]$Width = 0, [int]$Height = 0)
$ErrorActionPreference = 'Stop'
Add-Type @"
using System;
using System.Runtime.InteropServices;
public class Win {
  [DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr h);
  [DllImport("user32.dll")] public static extern bool ShowWindow(IntPtr h, int c);
  [DllImport("kernel32.dll")] public static extern uint SetThreadExecutionState(uint f);
  [DllImport("user32.dll")] public static extern bool MoveWindow(IntPtr h, int x, int y, int w, int ht, bool r);
}
"@
New-Item -ItemType Directory -Force $OutDir | Out-Null
$deadline = [datetime]::ParseExact($Until, 'HH:mm', $null)
$log = Join-Path $OutDir 'record.log'
function Say($m) { "$(Get-Date -Format HH:mm:ss) $m" | Tee-Object -Append $log }
function Rviz { Get-Process msrdc -ErrorAction SilentlyContinue |
    Where-Object { $_.MainWindowTitle -like $Title } | Select-Object -First 1 }
function NowMs { [DateTimeOffset]::Now.ToUnixTimeMilliseconds() }

Say "recorder up until $Until"
while ((Get-Date) -lt $deadline -and -not (Test-Path (Join-Path $OutDir 'stop_recorder'))) {
  $p = Rviz
  if (-not $p) { Start-Sleep -Milliseconds 500; continue }
  # ES_CONTINUOUS | ES_SYSTEM_REQUIRED | ES_DISPLAY_REQUIRED
  [Win]::SetThreadExecutionState([uint32]2147483651) | Out-Null
  Start-Sleep 2
  # -Width/-Height: a window of that size at the top left instead of
  # maximised (a smaller software render takes less CPU from the sim).
  if ($Width -gt 0) { [Win]::MoveWindow($p.MainWindowHandle, 0, 0, $Width, $Height, $true) | Out-Null }
  else { [Win]::ShowWindow($p.MainWindowHandle, 3) | Out-Null }
  [Win]::SetForegroundWindow($p.MainWindowHandle) | Out-Null
  Start-Sleep 2
  $t1 = NowMs
  $wsl = (wsl.exe -d Ubuntu-24.04 --exec date +%s.%N).Trim()
  $t2 = NowMs
  $offset = [double]$wsl * 1000.0 - ($t1 + $t2) / 2.0
  $name = 'rec_' + (Get-Date -Format 'yyyyMMdd_HHmmss')
  $mkv = Join-Path $OutDir "$name.mkv"
  $psi = New-Object System.Diagnostics.ProcessStartInfo
  $psi.FileName = $FFmpeg
  $psi.Arguments = "-hide_banner -loglevel error -f gdigrab -framerate 15 -draw_mouse 0 -i desktop -c:v libx264 -preset ultrafast -crf 20 -pix_fmt yuv420p `"$mkv`""
  $psi.UseShellExecute = $false
  $psi.RedirectStandardInput = $true
  $psi.CreateNoWindow = $true
  $proc = [System.Diagnostics.Process]::Start($psi)
  $start = NowMs
  @{ file = "$name.mkv"; win_start_ms = $start; wsl_minus_win_ms = $offset;
     title = $p.MainWindowTitle; rtt_ms = ($t2 - $t1) } |
    ConvertTo-Json | Set-Content -Encoding utf8 (Join-Path $OutDir "$name.json")
  Say "recording $name (clock offset $([math]::Round($offset)) ms, rtt $($t2 - $t1) ms)"
  while (Rviz) { Start-Sleep -Milliseconds 500 }
  $proc.StandardInput.Write('q')
  $proc.StandardInput.Close()
  if (-not $proc.WaitForExit(20000)) { $proc.Kill(); Say "ffmpeg killed" }
  [Win]::SetThreadExecutionState([uint32]2147483648) | Out-Null
  Say "stopped $name, $([math]::Round((Get-Item $mkv).Length / 1MB, 1)) MB"
}
Say "recorder done"
