@echo off
chcp 65001 >nul
set PYTHONUTF8=1
title Instalador de Dependencias - Robot 3 GDL
echo ==============================================================================
echo   Instalando dependencias de Python (OpenCV, PySerial, NumPy, Pillow, esptool)
echo ==============================================================================
cd /d "%~dp0"

set PY_CMD=python
where python >nul 2>&1
if %ERRORLEVEL% NEQ 0 (
    where py >nul 2>&1
    if %ERRORLEVEL% EQU 0 (
        set PY_CMD=py
    ) else (
        echo [ERROR] No se encontro Python instalado en esta PC.
        echo         Descargalo desde https://www.python.org/downloads/ y marca "Add Python to PATH".
        pause
        exit /b 1
    )
)

%PY_CMD% -m pip install --upgrade pip
%PY_CMD% -m pip install -r requirements.txt
echo.
echo ==============================================================================
echo   Instalacion completada. Ya puedes ejecutar 'iniciar_interfaz.bat' o
echo   'calibrar_casillas.bat' en cualquier PC.
echo ==============================================================================
pause
