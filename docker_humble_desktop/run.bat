@echo off
setlocal

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
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw ^
    -v /run/user/1000/at-spi:/run/user/1000/at-spi ^
    -v /dev:/dev ^
    -v %parent_dir%:/home/xplore/dev_ws/src ^
    -v base_humble_desktop_home_volume:/home/xplore ^
    ghcr.io/epflxplore/base:humble-desktop

endlocal
pause
