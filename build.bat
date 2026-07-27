@echo off
setlocal
call "D:\Program Files\Microsoft Visual Studio\2022\Preview\VC\Auxiliary\Build\vcvarsall.bat" x86
if errorlevel 1 exit /b 1

cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo .
if errorlevel 1 exit /b 1

cmake --build build --target navgen
if errorlevel 1 exit /b 1
