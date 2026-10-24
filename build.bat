@echo off
rem Build WarpNine (release). Close WarpNine first: Windows locks the running .exe.
cd /d "%~dp0"
tasklist /FI "IMAGENAME eq warpnine.exe" | find /I "warpnine.exe" >nul
if not errorlevel 1 (
    echo WarpNine is running. Close it, then run build.bat again.
    pause
    exit /b 1
)
cargo build --release
if errorlevel 1 (
    echo.
    echo BUILD FAILED - see errors above.
    pause
    exit /b 1
)
echo.
echo Built: %~dp0target\release\warpnine.exe
if /I "%1"=="run" start "" "%~dp0target\release\warpnine.exe"
