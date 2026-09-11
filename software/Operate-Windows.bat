@echo off
setlocal
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" goto missing
".venv\Scripts\python.exe" start.py operate
goto done
:missing
echo Run Install-Windows.bat first.
:done
pause
