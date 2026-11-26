@echo off
rem Installs the gs_usb J2534 driver (64-bit and 32-bit) for all users. Asks for administrator rights.
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\install.ps1" %*
echo.
pause
