@echo off
title Flashear Firmware ESP32-S3 (Robot 3 GDL)
echo ==============================================================================
echo   Flasheo de Firmware C (ESP-IDF) para ESP32-S3 - Robot 3 GDL
echo ==============================================================================
set /p COM_PORT="Ingresa el puerto COM del ESP32-S3 (ej. COM4 o COM12): "
echo Flasheando binario en %COM_PORT%...
powershell -Command "& 'C:\esp\v6.1\esp-idf\export.ps1'; idf.py -p %COM_PORT% flash"
echo.
echo ==============================================================================
echo   Proceso de flasheo finalizado.
echo ==============================================================================
pause
