@echo off
chcp 65001 >nul
set PYTHONUTF8=1
title Robot 3 GDL - Panel de Control Cinemático y Juego de Gato (Lazo Cerrado)
echo ==============================================================================
echo   Iniciando Interfaz de Control (Nueva Dinámica + Lazo Cerrado AS5600)...
echo ==============================================================================
cd /d "%~dp0"
python gui_robot_gato.py
if %ERRORLEVEL% NEQ 0 (
    echo.
    echo [ERROR] Hubo un problema al iniciar la interfaz. Asegurate de haber ejecutado
    echo         primero 'instalar_dependencias.bat'.
    pause
)
