@echo off
setlocal
call "D:\Program Files\Microsoft Visual Studio\2022\Preview\VC\Auxiliary\Build\vcvarsall.bat" x86
if errorlevel 1 exit /b 1

cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo .
if errorlevel 1 exit /b 1

cmake --build build
if errorlevel 1 exit /b 1

rem Deploy to the local HLDS test install, so test_nav_goal.ps1 always runs the DLL that was just
rem built. Skipped without complaint when hlds\ is absent -- it is gitignored, not everyone has it.
if exist "%~dp0hlds\valve\addons\crossbot\" (
    copy /y "%~dp0build\crossbot_mm.dll" "%~dp0hlds\valve\addons\crossbot\crossbot_mm.dll" >nul
    if errorlevel 1 (
        echo build.bat: FAILED to copy crossbot_mm.dll into hlds -- is the server still running?
        exit /b 1
    )
    echo build.bat: deployed crossbot_mm.dll to hlds\valve\addons\crossbot
)
