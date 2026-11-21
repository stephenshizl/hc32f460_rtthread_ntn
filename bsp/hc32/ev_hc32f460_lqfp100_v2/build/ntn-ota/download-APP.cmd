@echo off
cd /d "%~dp0"
python flash_station.py "flash-primary.hex" --sha256 0cef2c4ade7785bbbbdd02893ae689e9dfdf044676b7639e931ca982d6d8d858 > "download-APP.log" 2>&1
set "result=%errorlevel%"
type "download-APP.log"
exit /b %result%
