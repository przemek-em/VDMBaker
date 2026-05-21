@echo off
setlocal

set "APP_NAME=VDMBaker"
set "SCRIPT_DIR=%~dp0"
set "SRC=%SCRIPT_DIR%VDMBaker.cpp"
set "BUILD_DIR=%SCRIPT_DIR%build"
set "OUT=%BUILD_DIR%\%APP_NAME%.exe"

if not exist "%SRC%" (
    echo [FAIL] Source file not found:
    echo        %SRC%
    exit /b 1
)

where cl.exe >nul 2>nul
if errorlevel 1 (
    set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
    if exist "%VSWHERE%" (
        for /f "usebackq tokens=*" %%I in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VSINSTALL=%%I"
    )

    if defined VSINSTALL (
        call "%VSINSTALL%\Common7\Tools\VsDevCmd.bat" -arch=x64 >nul
    ) else if exist "%ProgramFiles%\Microsoft Visual Studio\2022\Community\Common7\Tools\VsDevCmd.bat" (
        call "%ProgramFiles%\Microsoft Visual Studio\2022\Community\Common7\Tools\VsDevCmd.bat" -arch=x64 >nul
    ) else if exist "%ProgramFiles%\Microsoft Visual Studio\2022\BuildTools\Common7\Tools\VsDevCmd.bat" (
        call "%ProgramFiles%\Microsoft Visual Studio\2022\BuildTools\Common7\Tools\VsDevCmd.bat" -arch=x64 >nul
    ) else if exist "%ProgramFiles%\Microsoft Visual Studio\2022\Professional\Common7\Tools\VsDevCmd.bat" (
        call "%ProgramFiles%\Microsoft Visual Studio\2022\Professional\Common7\Tools\VsDevCmd.bat" -arch=x64 >nul
    ) else if exist "%ProgramFiles%\Microsoft Visual Studio\2022\Enterprise\Common7\Tools\VsDevCmd.bat" (
        call "%ProgramFiles%\Microsoft Visual Studio\2022\Enterprise\Common7\Tools\VsDevCmd.bat" -arch=x64 >nul
    )
)

where cl.exe >nul 2>nul
if errorlevel 1 (
    echo [FAIL] MSVC cl.exe was not found.
    echo        Install Visual Studio C++ tools or run this from an x64 Native Tools Command Prompt.
    exit /b 1
)

if not exist "%BUILD_DIR%" mkdir "%BUILD_DIR%"

echo Building %APP_NAME%...
cl /nologo /W4 /EHsc /O2 /MT /std:c++17 /DUNICODE /D_UNICODE ^
    /Fe:"%OUT%" /Fo:"%BUILD_DIR%\\" "%SRC%" ^
    /link /SUBSYSTEM:WINDOWS User32.lib Gdi32.lib Comdlg32.lib Comctl32.lib

if errorlevel 1 (
    echo [FAIL] Build failed.
    exit /b 1
)

echo [OK] Built "%OUT%"
endlocal
