@echo off
REM Runs the full assignment pipeline (main.py) in the a4_2026 environment.
REM To see the pointcloud after every step, set VISUALIZE = True at the top of A4\src\perception.py
cd /d "%~dp0A4\src"
"%USERPROFILE%\micromamba\envs\a4_2026\python.exe" main.py
pause
