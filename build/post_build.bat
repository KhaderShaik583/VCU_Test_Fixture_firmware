@echo off 

REM Performing POST-BUILD actions
REM Create base output directory
set "OUT_DIR=vcu_outputs"
set "SEC_BIN_PATH=..\..\UV_F77_VCU_FXP_DEV\build"

mkdir ..\%OUT_DIR%

REM Create directory based on build date-time
REM https://stackoverflow.com/questions/49299611/create-folder-with-current-date-and-time-as-suiffix

for /f "tokens=2 delims==" %%a in ('wmic OS Get localdatetime /value') do set "dt=%%a"
set "YY=%dt:~2,2%" & set "YYYY=%dt:~0,4%" & set "MM=%dt:~4,2%" & set "DD=%dt:~6,2%"
set "HH=%dt:~8,2%" & set "Min=%dt:~10,2%" & set "Sec=%dt:~12,2%"

set "datestamp=%YYYY%%MM%%DD%" & set "timestamp=%HH%%Min%%Sec%"
set "fullstamp=%YYYY%_%MM%_%DD%_%HH%_%Min%_%Sec%"


mkdir ..\%OUT_DIR%\build_%fullstamp%

fromelf --bincombined --output=vcu_fixture_fx.bin ./Objects/vcu_fixture_fx.axf
copy .\Objects\vcu_fixture_fx.axf ..\%OUT_DIR%\build_%fullstamp%\vcu_fixture_fx.axf 
copy .\Objects\vcu_fixture_fx.hex ..\%OUT_DIR%\build_%fullstamp%\vcu_fixture_fx.hex 
copy .\Listings\vcu_fixture_fx.map ..\%OUT_DIR%\build_%fullstamp%\vcu_fixture_fx.map
move vcu_fixture_fx.bin ..\%OUT_DIR%\build_%fullstamp%\vcu_fixture_fx.bin

REM Remove all .o & .d files
del /F .\Objects\*.o
del /F .\Objects\*.d

cd ..\%OUT_DIR%\build_%fullstamp%
%SEC_BIN_PATH%\SecBin_CC.exe --BUILD_UV_VCU

REM del /f vcu_fixture_fx.bin