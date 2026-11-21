@echo off
cd /d "%~dp0"
python flash_station.py "ntn-ota-l0.hex" --sha256 f872fb8d8c24f39fda90e77848d4c051e5e60512163b192c090d06695da67a60 > "download-ALL.log" 2>&1
set "result=%errorlevel%"
type "download-ALL.log"
exit /b %result%
