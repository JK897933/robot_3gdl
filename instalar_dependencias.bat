@echo off
chcp 65001 >nul
set PYTHONUTF8=1
title Instalador de Dependencias - Robot 3 GDL
echo ==============================================================================
echo   Instalando dependencias de Python (OpenCV, PySerial, NumPy, ReportLab)
echo ==============================================================================
cd /d "%~dp0"
python -m pip install --upgrade pip
pip install -r requirements.txt
echo.
echo ==============================================================================
echo   Instalacion completada con exito. Ya puedes ejecutar 'iniciar_interfaz.bat'.
echo ==============================================================================
pause
