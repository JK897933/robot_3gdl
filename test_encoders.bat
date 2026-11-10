@echo off
chcp 65001 >nul
set PYTHONUTF8=1
title Diagnostico y Prueba de Encoders AS5600 - Robot 3 GDL
echo ==============================================================================
echo   Iniciando Diagnostico de Encoders AS5600 (ESP32-S3)...
echo ==============================================================================
cd /d "%~dp0"
python test_encoders.py
if %ERRORLEVEL% NEQ 0 (
    echo.
    echo [ERROR] No se pudo iniciar el script.
    pause
)
