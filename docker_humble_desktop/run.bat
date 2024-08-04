@echo off
setlocal

REM Check if the Xauthority file exists and prepare Xauthority data
set XAUTH=%TEMP%\.docker.xauth
echo Preparing Xauthority data...

for /f "tokens=*" %%i in ('xauth nlist :0 2^>nul ^| tail -n 1 ^| sed -e "s/^..../ffff/"') do set xauth_list=%%i

if not exist %XAUTH% (
    if not "%xauth_list%"=="" (
        echo %xauth_list% | xauth -f %XAUTH% nmerge -
    ) else (
        type nul > %XAUTH%
    )
    icacls %XAUTH% /grant *S-1-1-0:R
)

echo Done.
echo.
echo Verifying file contents:
file %XAUTH%
echo --^> It should say "X11 Xauthority data."
echo.
echo Permissions:
icacls %XAUTH%
echo.
echo Running docker...

REM Get the current working directory
for /f %%i in ('cd') do set current_dir=%%i

REM Use dirname to get the parent directory
for %%i in ("%current_dir%") do set parent_dir=%%~dpi
set parent_dir=%parent_dir:~0,-1%

docker run -it ^
    --name base_humble_desktop ^
    --rm ^
    --privileged ^
    --net=host ^
    -e DISPLAY=unix!DISPLAY! ^
    -e QT_X11_NO_MITSHM=1 ^
    -e XAUTHORITY=%XAUTH% ^
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw ^
    -v %XAUTH%:%XAUTH% ^
    -v /run/user/1000/at-spi:/run/user/1000/at-spi ^
    -v /dev:/dev ^
    -v %parent_dir%:/home/xplore/dev_ws/src ^
    -v base_humble_desktop_home_volume:/home/xplore ^
    ghcr.io/epflxplore/base:humble-desktop

endlocal
pause
