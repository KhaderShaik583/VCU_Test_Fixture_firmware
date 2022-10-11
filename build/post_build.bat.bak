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

fromelf --bincombined --output=vcu_fw.bin ./Objects/vcu_fw.axf
copy .\Objects\vcu_fw.axf ..\%OUT_DIR%\build_%fullstamp%\vcu_fw.axf 
copy .\Objects\vcu_fw.hex ..\%OUT_DIR%\build_%fullstamp%\vcu_fw.hex 
copy .\Listings\vcu_fw.map ..\%OUT_DIR%\build_%fullstamp%\vcu_fw.map
move vcu_fw.bin ..\%OUT_DIR%\build_%fullstamp%\vcu_fw.bin

REM Remove all .o & .d files
del /F .\Objects\*.o
del /F .\Objects\*.d

cd ..\%OUT_DIR%\build_%fullstamp%
%SEC_BIN_PATH%\SecBin_CC.exe --BUILD_UV_VCU

REM del /f vcu_fw.bin