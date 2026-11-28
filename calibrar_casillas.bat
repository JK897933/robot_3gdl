@echo off
chcp 65001 >nul
set PYTHONUTF8=1
title Calibrador de Casillas - Robot 3 GDL
echo ==============================================================================
echo   Iniciando Calibrador de Casillas (Teach-In 1:1)...
echo ==============================================================================
cd /d "%~dp0"
set PY_CMD=python
where python >nul 2>&1
if %ERRORLEVEL% NEQ 0 (
    where py >nul 2>&1
    if %ERRORLEVEL% EQU 0 set PY_CMD=py
)
%PY_CMD% calibrador_casillas.py
if %ERRORLEVEL% NEQ 0 (
    echo.
    echo [ERROR] No se pudo iniciar el script. Ejecuta primero
    echo         'instalar_dependencias.bat'.
    pause
)
