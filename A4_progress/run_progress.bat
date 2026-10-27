@echo off
REM Runs the current (half built) version: perception steps 1.1 - 1.5
cd /d "%~dp0src"
"%USERPROFILE%\micromamba\envs\a4_2026\python.exe" main.py
pause
