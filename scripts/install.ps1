<#
.SYNOPSIS
    Registers (or removes) the gs_usb J2534 driver for 64-bit and 32-bit J2534 applications.

.DESCRIPTION
    J2534 applications discover PassThru drivers in the registry:
      64-bit applications: HKLM\SOFTWARE\PassThruSupport.04.04\<name>
      32-bit applications: HKLM\SOFTWARE\WOW6432Node\PassThruSupport.04.04\<name>
    and load the DLL named there, which must match their bitness. Both DLLs are copied to
    %ProgramFiles%\GsUsbJ2534 (x64) and %ProgramFiles(x86)%\GsUsbJ2534 (x86) unless -InPlace.
    Writing to HKLM needs administrator rights; the script re-launches itself elevated (UAC prompt).

.EXAMPLE
    .\install.ps1                 # install both builds from ..\target
    .\install.ps1 -Arch x86       # only the 32-bit driver
    .\install.ps1 -InPlace        # register the DLLs where they are, do not copy
    .\install.ps1 -Status         # what is registered, is it current, who has it loaded
    .\install.ps1 -Uninstall
    .\install.ps1 -TraceFile C:\ProgramData\GsUsbJ2534\trace.log   # trace every application
    .\install.ps1 -NoTrace
    .\install.ps1 -TxBrs honor    # transmit FD frames with the BRS bit the application asks for
    .\install.ps1 -TxBrs strip    # default: send FD frames at the nominal rate (reliable on the test bench)
#>
[CmdletBinding()]
param(
    [string]$Name = 'candleLightFD J2534 (gs-usb-j2534)',
    [ValidateSet('both', 'x64', 'x86')][string]$Arch = 'both',
    [string]$DllPath = '',
    [string]$DllPath32 = '',
    [string]$Root = 'HKLM:\SOFTWARE\PassThruSupport.04.04',
    [string]$Root32 = 'HKLM:\SOFTWARE\WOW6432Node\PassThruSupport.04.04',
    [string]$InstallDir = (Join-Path $env:ProgramFiles 'GsUsbJ2534'),
    [string]$InstallDir32 = (Join-Path ${env:ProgramFiles(x86)} 'GsUsbJ2534'),
    [string]$LogFile = '',
    [string]$TraceFile = '',
    [ValidateSet('', 'honor', 'strip')][string]$TxBrs = '',
    [switch]$InPlace,
    [switch]$Uninstall,
    [switch]$Status,
    [switch]$NoTrace
)

$ErrorActionPreference = 'Stop'

# $PSScriptRoot is empty in some -File invocations; derive the location robustly.
$ScriptPath = $MyInvocation.MyCommand.Path
$ScriptDir = Split-Path -Parent $ScriptPath

# A 32-bit PowerShell (e.g. started directly by a 32-bit make) sees HKLM:\SOFTWARE redirected to
# WOW6432Node and would register the 64-bit DLL in the 32-bit hive. Re-run natively instead.
if (-not [Environment]::Is64BitProcess -and [Environment]::Is64BitOperatingSystem) {
    $native = Join-Path $env:windir 'sysnative\WindowsPowerShell\v1.0\powershell.exe'
    $forward = @('-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', $ScriptPath)
    foreach ($entry in $PSBoundParameters.GetEnumerator()) {
        $forward += "-$($entry.Key)"
        if ($entry.Value -isnot [System.Management.Automation.SwitchParameter]) { $forward += [string]$entry.Value }
    }
    & $native @forward
    exit $LASTEXITCODE
}

if (-not $DllPath) { $DllPath = Join-Path $ScriptDir '..\target\x86_64-pc-windows-msvc\release\gs_usb_j2534.dll' }
if (-not $DllPath32) { $DllPath32 = Join-Path $ScriptDir '..\target\i686-pc-windows-msvc\release\gs_usb_j2534.dll' }

$traceKey = 'HKLM:\SOFTWARE\GsUsbJ2534'

# The two flavours; the 64-bit view of the registry is addressed explicitly for both.
$flavours = @(
    @{ Arch = 'x64'; Build = $DllPath;   Root = $Root;   Dir = $InstallDir;   File = 'gs_usb_j2534_x64.dll' },
    @{ Arch = 'x86'; Build = $DllPath32; Root = $Root32; Dir = $InstallDir32; File = 'gs_usb_j2534_x86.dll' }
) | Where-Object { $Arch -eq 'both' -or $Arch -eq $_.Arch }

function Say([string]$text) {
    Write-Host $text
    if ($LogFile) { Add-Content -Path $LogFile -Value $text }
}

function Test-Admin {
    ([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole(
        [Security.Principal.WindowsBuiltInRole]::Administrator)
}

# Processes that currently have the DLL mapped (they keep it locked).
function Get-DllHolders([string]$path) {
    Get-Process -ErrorAction SilentlyContinue | Where-Object {
        try { $_.Modules.FileName -contains $path } catch { $false }
    } | ForEach-Object { "$($_.ProcessName) (PID $($_.Id))" }
}

if ($Status) {
    foreach ($f in $flavours) {
        $key = Join-Path $f.Root $Name
        Say "[$($f.Arch)]"
        if (Test-Path $key) {
            $reg = Get-ItemProperty $key
            Say "  registered: $key"
            Say "  protocols : CAN=$($reg.CAN) ISO15765=$($reg.ISO15765) FD_CAN_PS=$($reg.FD_CAN_PS) FD_ISO15765_PS=$($reg.FD_ISO15765_PS)"
            $lib = $reg.FunctionLibrary
            Say "  library   : $lib"
            if (Test-Path $lib) {
                $item = Get-Item $lib
                Say "  file      : $($item.Length) bytes, $($item.LastWriteTime)"
                if (Test-Path $f.Build) {
                    $same = (Get-FileHash $lib).Hash -eq (Get-FileHash $f.Build).Hash
                    Say ("  build     : " + $(if ($same) { 'identical to the latest build' } else { 'DIFFERS from the latest build -> run the install target' }))
                } else {
                    Say '  build     : no local build found'
                }
                $holders = @(Get-DllHolders $lib)
                Say ("  loaded by : " + $(if ($holders.Count) { $holders -join ', ' } else { '(nobody)' }))
            } else {
                Say '  file      : MISSING (registry points to a DLL that does not exist)'
            }
        } else {
            Say "  not registered at $key"
        }
    }
    $trace = (Get-ItemProperty $traceKey -ErrorAction SilentlyContinue).TraceFile
    Say ("[trace] " + $(if ($trace) { $trace } else { 'off (use -TraceFile, or GSUSB_J2534_LOG per process)' }))
    $brs = (Get-ItemProperty $traceKey -ErrorAction SilentlyContinue).TxBrs
    Say ("[tx brs] " + $(if ($brs -eq 1) { 'honored (frames are sent with the requested BRS)' } else { 'stripped (default: FD frames are sent at the nominal rate)' }))
    exit 0
}

$hklm = ($Root -like 'HKLM:*') -or ($Root32 -like 'HKLM:*')
$needsAdmin = $hklm -or $TraceFile -or $NoTrace -or $TxBrs -or (-not $InPlace -and -not $Uninstall)
if ($needsAdmin -and -not (Test-Admin)) {
    Write-Host 'Administrator rights are required; requesting elevation (UAC prompt)...'
    # The elevated instance runs in its own window; it reports through a log file.
    $log = Join-Path $env:TEMP ("gs-usb-j2534-install-{0}.log" -f [guid]::NewGuid().ToString('N'))
    $argList = @('-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', "`"$ScriptPath`"", '-LogFile', "`"$log`"")
    foreach ($entry in $PSBoundParameters.GetEnumerator()) {
        if ($entry.Key -eq 'LogFile') { continue }
        $argList += "-$($entry.Key)"
        if ($entry.Value -isnot [System.Management.Automation.SwitchParameter]) { $argList += "`"$($entry.Value)`"" }
    }
    $p = Start-Process -FilePath (Get-Process -Id $PID).Path -ArgumentList $argList -Verb RunAs -Wait -PassThru
    if (Test-Path $log) { Get-Content $log | Write-Host; Remove-Item $log -Force }
    exit $p.ExitCode
}

if ($TxBrs) {
    if ($TxBrs -eq 'honor') {
        New-Item -Path $traceKey -Force | Out-Null
        New-ItemProperty -Path $traceKey -Name TxBrs -PropertyType DWord -Value 1 -Force | Out-Null
        Say 'FD frames are now sent with the BRS bit the application asks for (applies to programs started from now on).'
    } else {
        Remove-ItemProperty -Path $traceKey -Name TxBrs -ErrorAction SilentlyContinue
        Say 'FD frames are now sent at the nominal rate, BRS requests are dropped (default).'
    }
    exit 0
}

if ($TraceFile -or $NoTrace) {
    if ($NoTrace) {
        Remove-ItemProperty -Path $traceKey -Name TraceFile -ErrorAction SilentlyContinue
        Say 'Driver trace is off.'
    } else {
        $dir = Split-Path -Parent $TraceFile
        if ($dir -and -not (Test-Path $dir)) { New-Item -ItemType Directory -Force -Path $dir | Out-Null }
        if ($dir) { & icacls.exe $dir /grant '*S-1-5-32-545:(OI)(CI)M' | Out-Null }  # Users may write the trace
        New-Item -Path $traceKey -Force | Out-Null
        New-ItemProperty -Path $traceKey -Name TraceFile -PropertyType String -Value $TraceFile -Force | Out-Null
        Say "Driver trace on: $TraceFile (applies to programs started from now on; the file grows fast, turn it off afterwards)"
    }
    exit 0
}

$exit = 0

if ($Uninstall) {
    foreach ($f in $flavours) {
        $key = Join-Path $f.Root $Name
        if (Test-Path $key) {
            Remove-Item -Path $key -Recurse -Force
            Say "[$($f.Arch)] removed $key"
        } else {
            Say "[$($f.Arch)] nothing registered at $key"
        }
        if (-not $InPlace -and (Test-Path $f.Dir)) {
            try {
                Remove-Item -Path $f.Dir -Recurse -Force
                Say "[$($f.Arch)] removed $($f.Dir)"
            } catch {
                Say "[$($f.Arch)] could not remove $($f.Dir) (in use by: $((Get-DllHolders (Join-Path $f.Dir $f.File)) -join ', ')); it is no longer registered."
                $exit = 2
            }
        }
    }
    if (Test-Path $traceKey) { Remove-Item -Path $traceKey -Recurse -Force }
    exit $exit
}

foreach ($f in $flavours) {
    if (-not (Test-Path $f.Build)) {
        Say "[$($f.Arch)] DLL not found: $($f.Build) (run 'make build' first)"
        if ($Arch -ne 'both') { exit 1 }
        $exit = 1
        continue
    }
    $dll = (Resolve-Path $f.Build).Path
    if (-not $InPlace) {
        New-Item -ItemType Directory -Force -Path $f.Dir | Out-Null
        $target = Join-Path $f.Dir $f.File
        try {
            Copy-Item -Path $dll -Destination $target -Force
        } catch {
            Say "[$($f.Arch)] cannot replace ${target}: it is in use."
            $holders = @(Get-DllHolders $target)
            if ($holders.Count) { Say ("    close these programs and retry: " + ($holders -join ', ')) }
            $exit = 2
            continue
        }
        $dll = $target
    }

    $key = Join-Path $f.Root $Name
    # Earlier versions registered under this name; avoid listing the driver twice.
    $legacy = Join-Path $f.Root 'gs_usb candleLight(FD)'
    if ((Test-Path $legacy) -and $legacy -ne $key) { Remove-Item -Path $legacy -Recurse -Force }

    New-Item -Path $key -Force | Out-Null
    $values = [ordered]@{
        Name              = @('String', $Name)
        Vendor            = @('String', 'candleLight / gs_usb')
        FunctionLibrary   = @('String', $dll)
        ConfigApplication = @('String', '')
        CAN               = @('DWord', 1)
        ISO15765          = @('DWord', 1)
        # J2534-2 protocol ids as values (like PEAK): non-zero means supported, and the value
        # doubles as the ProtocolID for tools that read it from the registry.
        FD_CAN_PS         = @('DWord', 0x8011)
        FD_ISO15765_PS    = @('DWord', 0x8012)
    }
    foreach ($entry in $values.GetEnumerator()) {
        New-ItemProperty -Path $key -Name $entry.Key -PropertyType $entry.Value[0] -Value $entry.Value[1] -Force | Out-Null
    }
    Say "[$($f.Arch)] registered '$Name' -> $dll"
    Say "      key: $key"
}
exit $exit
