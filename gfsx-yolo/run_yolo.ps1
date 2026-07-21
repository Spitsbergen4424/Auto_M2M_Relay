$ErrorActionPreference = 'Stop'
Set-Location $PSScriptRoot

if (-not (Test-Path '.venv\Scripts\python.exe')) {
    throw 'Run install.ps1 first.'
}

if (-not (Test-Path 'config.yaml')) {
    Copy-Item 'config.example.yaml' 'config.yaml'
}

& .venv\Scripts\python.exe yolo_vision_node.py --config config.yaml
