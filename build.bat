@echo off
setlocal
set CROSSBOT_DEPLOY=D:\hlds\crossplay\addons\crossbot
call "D:\Program Files\Microsoft Visual Studio\2022\Preview\VC\Auxiliary\Build\vcvarsall.bat" x86
if errorlevel 1 exit /b 1

cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo %* .
if errorlevel 1 exit /b 1

cmake --build build
if errorlevel 1 exit /b 1

if exist "%CROSSBOT_DEPLOY%\" (
    copy /y "%~dp0build\crossbot_mm.dll" "%CROSSBOT_DEPLOY%\crossbot_mm.dll" >nul
    if errorlevel 1 (
        echo build.bat: FAILED to copy crossbot_mm.dll into hlds -- is the server still running?
        exit /b 1
    )

    if not exist "%CROSSBOT_DEPLOY%\nav\" mkdir "%CROSSBOT_DEPLOY%\nav"
    if not exist "%CROSSBOT_DEPLOY%\trace\" mkdir "%CROSSBOT_DEPLOY%\trace"

    echo build.bat: deployed crossbot_mm.dll to %CROSSBOT_DEPLOY%
)
