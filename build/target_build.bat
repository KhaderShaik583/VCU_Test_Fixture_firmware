@echo off

git rev-parse --short HEAD > tmp
set /p ch= < tmp
del /F tmp

gcc -I..\firmware\inc ..\firmware\src\version.c .\vsn.c -o vsn
vsn.exe > tmp
set /p fw= < tmp
del /F tmp
del /F vsn.exe

goto main

:create_image
UV4 -b ..\vcu_fw_s32k148.uvprojx -t%~1
set image_name="VCU_FAC_v%fw%_%~2"
csapacmpz.exe .\vcu_part_bl_app.cfg /imagecontent %image_name% /imagefile .\%image_name%.esap
cyclonecontrolconsole -cyclone=HAL9000 -addimageinternal=.\%image_name%.esap

set image_name="VCU_APP_v%fw%_%~2"
csapacmpz.exe .\vcu_app.cfg /imagecontent %image_name% /imagefile .\%image_name%.esap
cyclonecontrolconsole -cyclone=HAL9000 -addimageinternal=.\%image_name%.esap

goto :EOF

:main
call :create_image "VCU" "PROD"
call :create_image "VCU" "DESK"




