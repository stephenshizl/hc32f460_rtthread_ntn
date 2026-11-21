@echo off
cd /d "%~dp0"
python flash_station.py "flash-bl2.hex" --sha256 c87ccf42941561fa1e89331d365902ab2cebca2a8be2518e402056445ae92853 > "download-BL2.log" 2>&1
set "result=%errorlevel%"
type "download-BL2.log"
exit /b %result%
