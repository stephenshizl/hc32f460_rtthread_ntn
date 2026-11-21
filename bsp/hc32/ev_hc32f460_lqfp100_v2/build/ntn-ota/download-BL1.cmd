@echo off
cd /d "%~dp0"
python flash_station.py "flash-bl1.hex" --sha256 727d9f293cdd237bb7c048f13f18f977595ba20f20c6d2c596fb7b0767dc2645 > "download-BL1.log" 2>&1
set "result=%errorlevel%"
type "download-BL1.log"
exit /b %result%
