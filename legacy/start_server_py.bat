@echo off
title Pioneer 3-AT Modular Map Viewer Server

echo ============================================================
echo Pioneer 3-AT Modular Map Viewer
echo ============================================================
echo Starting localhost server...
echo Viewer: http://127.0.0.1:8000/map_viewer2_modular.html
echo WebSocket: ws://localhost:8765
echo.
echo Close this window or press Ctrl+C to stop.
echo ============================================================

py server.py

if errorlevel 1 (
    echo.
    echo Could not start the server with "py".
    echo Make sure Python 3 is installed.
    pause
)
