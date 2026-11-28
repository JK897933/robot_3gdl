@echo off
chcp 65001 >nul
set PYTHONUTF8=1
title Robot 3 GDL - Panel de Control Cinemático y Juego de Gato
echo ==============================================================================
echo   Iniciando Interfaz de Control - Robot 3 GDL
echo ==============================================================================
cd /d "%~dp0"
set PY_CMD=python
where python >nul 2>&1
if %ERRORLEVEL% NEQ 0 (
    where py >nul 2>&1
    if %ERRORLEVEL% EQU 0 set PY_CMD=py
)
%PY_CMD% gui_robot_gato.py
if %ERRORLEVEL% NEQ 0 (
    echo.
    echo [ERROR] Hubo un problema al iniciar la interfaz. Ejecuta primero
    echo         'instalar_dependencias.bat'.
    pause
)
