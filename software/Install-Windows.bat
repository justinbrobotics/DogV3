@echo off
setlocal
cd /d "%~dp0"
where py >nul 2>nul
if errorlevel 1 goto python
py -3 start.py install
goto done
:python
python start.py install
:done
if errorlevel 1 echo Install failed. Install Python 3.11 or newer and review the message above.
pause
