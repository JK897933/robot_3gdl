@echo off
chcp 65001 >nul
set PYTHONUTF8=1
title Flashear Firmware ESP32-S3 (Robot 3 GDL - CONTROL)
echo ==============================================================================
echo   Flasheo de Firmware C (ESP-IDF) para ESP32-S3 - Robot 3 GDL (CONTROL)
echo ==============================================================================
cd /d "%~dp0"
set COM_PORT=COM7
set /p COM_PORT_INPUT="Ingresa el puerto COM del ESP32-S3 (Enter para usar %COM_PORT%): "
if not "%COM_PORT_INPUT%"=="" set COM_PORT=%COM_PORT_INPUT%

echo.
echo Flasheando binario en %COM_PORT%...

set ESP_BUILD=C:\esp_build
if not exist "%ESP_BUILD%\robot_3gdl_gato.bin" set ESP_BUILD=%~dp0build

if exist "%ESP_BUILD%\robot_3gdl_gato.bin" (
    echo Usando binarios desde: %ESP_BUILD%
    if exist "C:\Users\hp\.espressif\python_env\idf6.1_py3.13_env\Scripts\python.exe" (
        "C:\Users\hp\.espressif\python_env\idf6.1_py3.13_env\Scripts\python.exe" -m esptool --chip esp32s3 -p %COM_PORT% -b 115200 --no-stub write-flash --flash-mode dio --flash-freq 80m --flash-size 2MB 0x0 "%ESP_BUILD%\bootloader\bootloader.bin" 0x8000 "%ESP_BUILD%\partition_table\partition-table.bin" 0x10000 "%ESP_BUILD%\robot_3gdl_gato.bin"
    ) else (
        powershell -Command "& 'C:\esp\v6.1\esp-idf\export.ps1'; idf.py -p %COM_PORT% flash"
    )
) else (
    echo Compilando y flasheando con ESP-IDF...
    powershell -Command "& 'C:\esp\v6.1\esp-idf\export.ps1'; idf.py -p %COM_PORT% flash"
)

echo.
echo ==============================================================================
echo   Proceso de flasheo finalizado.
echo ==============================================================================
pause
