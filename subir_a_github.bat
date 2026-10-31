@echo off
title Subir Robot 3 GDL a GitHub
echo ==============================================================================
echo   Subir Repositorio a GitHub (up230061@alumnos.upa.edu.mx)
echo ==============================================================================
echo Asegurate de haber creado el repositorio vacio en https://github.com/new
echo (sin marcar 'Add README', 'Add .gitignore' ni licencia).
echo.
set /p REPO_URL="Pega aqui el enlace HTTPS de tu repositorio (ej. https://github.com/tu_usuario/robot_3gdl.git): "
git remote remove origin 2>nul
git remote add origin %REPO_URL%
git branch -M main
echo.
echo Subiendo proyecto completo al 100%%...
git push -u origin main
if %ERRORLEVEL% EQU 0 (
    echo.
    echo ==============================================================================
    echo   ¡Exito! El proyecto ya esta en GitHub listo para descargar y funcionar.
    echo ==============================================================================
) else (
    echo.
    echo [ERROR] Hubo un detalle al subir. Asegurate de que el repositorio exista en GitHub
    echo         y que tu cuenta tenga permisos.
)
pause
