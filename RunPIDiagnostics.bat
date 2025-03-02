@echo off
setlocal

python --version >nul 2>&1
if %errorlevel% neq 0 (
    echo Python is not installed. Installing Python 3.11...
    start https://www.python.org/ftp/python/3.11.0/python-3.11.0-amd64.exe
    echo Please install Python manually and restart this script.
    pause
    exit /b
)

python -m pip --version >nul 2>&1
if %errorlevel% neq 0 (
    echo Pip is not installed. Installing pip...
    python -m ensurepip --default-pip
    python -m pip install --upgrade pip
)

echo Installing dependencies...
pip install -r PiDiagnostics/requirements.txt

echo Running Diagnostic Dashboard...
python PiDiagnostics/pidiagnostics.py

endlocal
pause
