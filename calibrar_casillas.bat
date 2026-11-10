@echo off
chcp 65001 >nul
set PYTHONUTF8=1
title Calibrador de Casillas - Robot 3 GDL
echo ==============================================================================
echo   Iniciando Calibrador de Casillas (Teach-In 1:1)...
echo ==============================================================================
cd /d "%~dp0"
python calibrador_casillas.py
if %ERRORLEVEL% NEQ 0 (
    echo.
    echo [ERROR] No se pudo iniciar el script.
    pause
)
