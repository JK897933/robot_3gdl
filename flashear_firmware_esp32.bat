@echo off
chcp 65001 >nul
set PYTHONUTF8=1
title Flashear Firmware ESP32-S3 (Robot 3 GDL - Portable)
echo ==============================================================================
echo   Flasheo Portable de Firmware ESP32-S3 - Robot 3 GDL
echo   (Funciona en cualquier PC con Python + instalar_dependencias.bat)
echo ==============================================================================
cd /d "%~dp0"

set PY_CMD=python
where python >nul 2>&1
if %ERRORLEVEL% NEQ 0 (
    where py >nul 2>&1
    if %ERRORLEVEL% EQU 0 set PY_CMD=py
)

set DETECTED_PORT=
for /f "delims=" %%p in ('%PY_CMD% -c "import serial.tools.list_ports as lp; ps=list(lp.comports()); esp=[p.device for p in ps if '303A' in (p.hwid or '').upper() or 'ESP32' in (p.description or '').upper()]; usb=[p.device for p in ps if 'USB' in (p.description or '').upper() and 'BLUETOOTH' not in (p.description or '').upper()]; print((esp+usb)[0] if (esp+usb) else '')" 2^>nul') do set DETECTED_PORT=%%p

if "%DETECTED_PORT%"=="" set DETECTED_PORT=COM12
echo Puerto detectado automaticamente: %DETECTED_PORT%
set COM_PORT=%DETECTED_PORT%
set /p COM_PORT_INPUT="Ingresa el puerto COM del ESP32-S3 (Enter para usar %COM_PORT%): "
if not "%COM_PORT_INPUT%"=="" set COM_PORT=%COM_PORT_INPUT%

set BIN_DIR=%~dp0firmware_bin
if not exist "%BIN_DIR%\robot_3gdl_gato.bin" set BIN_DIR=%~dp0build

if not exist "%BIN_DIR%\robot_3gdl_gato.bin" (
    echo [ERROR] No se encontro 'robot_3gdl_gato.bin' en firmware_bin ni en build.
    pause
    exit /b 1
)

echo.
echo Flasheando binarios desde: %BIN_DIR% en %COM_PORT%...
%PY_CMD% -m esptool --chip esp32s3 -p %COM_PORT% -b 460800 --before default-reset --after hard-reset write-flash --flash-mode dio --flash-freq 80m --flash-size 2MB 0x0 "%BIN_DIR%\bootloader.bin" 0x8000 "%BIN_DIR%\partition-table.bin" 0x10000 "%BIN_DIR%\robot_3gdl_gato.bin"

echo.
echo ==============================================================================
echo   Proceso de flasheo finalizado.
echo ==============================================================================
pause
