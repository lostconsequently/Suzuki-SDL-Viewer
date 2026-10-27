@echo off
rem Builds dist\SDLView.exe (a single file, no Python needed to run it).
cd /d "%~dp0"
python -m pip install --user --upgrade pyserial pyinstaller || goto :failed
python -m PyInstaller --noconfirm --onefile --windowed --name SDLView ^
    --icon assets\sdlview.ico ^
    --add-data "assets\sdlview.ico;." ^
    --add-data "assets\sdlview.png;." ^
    sdlview.py || goto :failed
echo.
echo Done: %~dp0dist\SDLView.exe
goto :eof
:failed
echo.
echo Build failed.
exit /b 1
