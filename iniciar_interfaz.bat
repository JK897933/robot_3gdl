@echo off
title Robot 3 GDL - Panel de Control Cinemático y Juego de Gato
echo ==============================================================================
echo   Iniciando Interfaz Clasica de Control del Robot 3 GDL...
echo ==============================================================================
python gui_robot_gato.py
if %ERRORLEVEL% NEQ 0 (
    echo.
    echo [ERROR] Hubo un problema al iniciar la interfaz. Asegurate de haber ejecutado
    echo         primero 'instalar_dependencias.bat'.
    pause
)
