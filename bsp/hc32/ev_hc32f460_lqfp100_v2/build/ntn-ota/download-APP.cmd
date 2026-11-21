@echo off
cd /d "%~dp0"
python flash_station.py "flash-primary.hex" --sha256 fb8c9a2d1e67c3716aff1e55a5b5ea816c4a12e76eaaec3d26be510fc041c80a > "download-APP.log" 2>&1
set "result=%errorlevel%"
type "download-APP.log"
exit /b %result%
