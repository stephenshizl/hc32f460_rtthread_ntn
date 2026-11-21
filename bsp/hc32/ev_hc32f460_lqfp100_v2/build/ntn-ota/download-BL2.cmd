@echo off
cd /d "%~dp0"
python flash_station.py "flash-bl2.hex" --sha256 7b0fa765c104ec56b26f75dd5d733514969428fb3b7b8e329e0d1a3a4ace2ee1 > "download-BL2.log" 2>&1
set "result=%errorlevel%"
type "download-BL2.log"
exit /b %result%
