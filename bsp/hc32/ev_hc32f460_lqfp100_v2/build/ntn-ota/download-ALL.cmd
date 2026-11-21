@echo off
cd /d "%~dp0"
python flash_station.py "ntn-ota-l0.hex" --sha256 e47a2a733bd5a06be846efae207cb66ad11a25754022f3a90f4951943cd995e7 > "download-ALL.log" 2>&1
set "result=%errorlevel%"
type "download-ALL.log"
exit /b %result%
