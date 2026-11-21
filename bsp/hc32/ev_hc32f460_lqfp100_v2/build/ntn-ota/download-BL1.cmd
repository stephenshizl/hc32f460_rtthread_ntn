@echo off
cd /d "%~dp0"
python flash_station.py "flash-bl1.hex" --sha256 ae0ce1db60a8f8310ded2ace5e771502367c76c0112d9abb633090922b6b36c5 > "download-BL1.log" 2>&1
set "result=%errorlevel%"
type "download-BL1.log"
exit /b %result%
