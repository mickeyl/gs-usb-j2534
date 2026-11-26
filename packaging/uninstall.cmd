@echo off
rem Removes the gs_usb J2534 driver for all users. Asks for administrator rights.
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\install.ps1" -Uninstall
echo.
pause
