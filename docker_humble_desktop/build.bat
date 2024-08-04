@echo off
setlocal

REM Set the Docker image tag and Dockerfile path
set IMAGE_TAG=ghcr.io/epflxplore/base:humble-desktop
set DOCKERFILE_PATH=Dockerfile

REM Get the current working directory
for /f %%i in ('cd') do set current_dir=%%i

REM Use dirname to get the parent directory
for %%i in ("%current_dir%") do set parent_dir=%%~dpi
set parent_dir=%parent_dir:~0,-1%

echo Building Docker image %IMAGE_TAG% using Dockerfile at %DOCKERFILE_PATH%...

docker build --pull --no-cache --progress=plain -t %IMAGE_TAG% -f %DOCKERFILE_PATH% %parent_dir%

echo Docker build finished.
pause
endlocal
