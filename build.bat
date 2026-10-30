@echo off
setlocal
set CROSSBOT_DEPLOY=D:\hlds\crossplay\addons\crossbot
set VCVARSALL=D:\Program Files\Microsoft Visual Studio\2022\Preview\VC\Auxiliary\Build\vcvarsall.bat

if /i "%~1"=="standalone" (
    call "%VCVARSALL%" x86
    if errorlevel 1 exit /b 1
    cmake -B build-standalone -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCROSSBOT_STANDALONE=ON .
    if errorlevel 1 exit /b 1
    cmake --build build-standalone
    exit /b %errorlevel%
)

setlocal
call "%VCVARSALL%" x64
if errorlevel 1 exit /b 1
cmake -B build-x64 -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo %* .
if errorlevel 1 exit /b 1
cmake --build build-x64
if errorlevel 1 exit /b 1
endlocal

call "%VCVARSALL%" x86
if errorlevel 1 exit /b 1
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo %* .
if errorlevel 1 exit /b 1
cmake --build build
if errorlevel 1 exit /b 1

if exist "%CROSSBOT_DEPLOY%\" (
    copy /y "%~dp0names.txt" "%CROSSBOT_DEPLOY%\names.txt" >nul
    copy /y "%~dp0places.txt" "%CROSSBOT_DEPLOY%\places.txt" >nul
    if not exist "%CROSSBOT_DEPLOY%\brain.json" copy /y "%~dp0brain.example.json" "%CROSSBOT_DEPLOY%\brain.json" >nul
    copy /y "%~dp0build\crossbot_mm.dll" "%CROSSBOT_DEPLOY%\crossbot_mm.dll" >nul
    if errorlevel 1 (
        echo build.bat: FAILED to copy crossbot_mm.dll into hlds -- is the server still running?
        exit /b 1
    )

    if not exist "%CROSSBOT_DEPLOY%\nav\" mkdir "%CROSSBOT_DEPLOY%\nav"
    if not exist "%CROSSBOT_DEPLOY%\trace\" mkdir "%CROSSBOT_DEPLOY%\trace"

    echo build.bat: deployed crossbot_mm.dll to %CROSSBOT_DEPLOY%
)
