$ErrorActionPreference = 'Stop'
Set-Location $PSScriptRoot

if (-not (Test-Path '.venv\Scripts\python.exe')) {
    throw 'Run install.ps1 first.'
}

& .venv\Scripts\python.exe udp_test_sender.py
