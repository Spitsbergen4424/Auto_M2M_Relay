$ErrorActionPreference = 'Stop'
Set-Location $PSScriptRoot

function Get-PythonLauncher {
    if (Get-Command py -ErrorAction SilentlyContinue) {
        foreach ($version in @('3.11', '3.10')) {
            & py "-$version" -c "import sys" | Out-Null
            if ($LASTEXITCODE -eq 0) {
                return @('py', "-$version")
            }
        }
    }

    if (Get-Command python -ErrorAction SilentlyContinue) {
        & python -c "import sys; assert sys.version_info[:2] in [(3,10),(3,11)]" | Out-Null
        if ($LASTEXITCODE -eq 0) {
            return @('python')
        }
    }

    throw 'Python 3.10 or 3.11 was not found.'
}

$python = Get-PythonLauncher

if (-not (Test-Path '.venv\Scripts\python.exe')) {
    Write-Host "Creating virtual environment..."
    & $python[0] $python[1] -m venv .venv
    if ($LASTEXITCODE -ne 0) { throw 'Failed to create .venv.' }
}
else {
    Write-Host "Reusing existing .venv."
}

if (-not (Test-Path 'config.yaml')) {
    Copy-Item 'config.example.yaml' 'config.yaml'
}

& .venv\Scripts\python.exe -m pip install --upgrade pip
if ($LASTEXITCODE -ne 0) { throw 'Failed to upgrade pip.' }

& .venv\Scripts\python.exe -m pip install -r requirements.txt
if ($LASTEXITCODE -ne 0) { throw 'Failed to install requirements.' }

& .venv\Scripts\python.exe -c "from ultralytics import YOLO; m=YOLO(r'models\best_detect.pt', task='detect'); print('Classes:', m.names)"
if ($LASTEXITCODE -ne 0) { throw 'Model verification failed.' }

Write-Host ''
Write-Host 'Installation completed successfully.'
