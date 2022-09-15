@echo off 

REM Performing PRE-BUILD actions

cd firmware
cd src

copy /b version.c +,,
cd..
cd..


echo "Pre-Build complete."

