<#
.SYNOPSIS
    Builds the release package: one zip with the x64 and the x86 DLL, the installer and the docs.

.EXAMPLE
    .\package.ps1                 # version from Cargo.toml, DLLs from ..\target (run `cargo build --release` for both targets first)
    .\package.ps1 -Version 0.2.0  # explicit version
#>
[CmdletBinding()]
param([string]$Version = '')

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
if (-not $Version) {
    $Version = (Select-String -Path (Join-Path $root 'Cargo.toml') -Pattern '^version = "(.*)"' | Select-Object -First 1).Matches[0].Groups[1].Value
}

$x64 = Join-Path $root 'target\x86_64-pc-windows-msvc\release\gs_usb_j2534.dll'
$x86 = Join-Path $root 'target\i686-pc-windows-msvc\release\gs_usb_j2534.dll'
foreach ($f in $x64, $x86) { if (-not (Test-Path $f)) { throw "missing build output: $f" } }

$name = "gs-usb-j2534-$Version-windows"
$dist = Join-Path $root 'dist'
$pkg = Join-Path $dist $name
if (Test-Path $pkg) { Remove-Item -Recurse -Force $pkg }
New-Item -ItemType Directory -Force -Path (Join-Path $pkg 'scripts') | Out-Null

Copy-Item $x64 (Join-Path $pkg 'gs_usb_j2534_x64.dll')
Copy-Item $x86 (Join-Path $pkg 'gs_usb_j2534_x86.dll')
Copy-Item (Join-Path $root 'scripts\install.ps1') (Join-Path $pkg 'scripts')
Copy-Item (Join-Path $root 'packaging\install.cmd'), (Join-Path $root 'packaging\uninstall.cmd') $pkg
Copy-Item (Join-Path $root 'README.md'), (Join-Path $root 'LICENSE') $pkg

$zip = Join-Path $dist "$name.zip"
if (Test-Path $zip) { Remove-Item -Force $zip }
Compress-Archive -Path (Join-Path $pkg '*') -DestinationPath $zip
$hash = (Get-FileHash $zip -Algorithm SHA256).Hash.ToLower()
"$hash  $name.zip" | Set-Content -Encoding ascii (Join-Path $dist "$name.zip.sha256")
Write-Host $zip
Write-Host "sha256 $hash"
