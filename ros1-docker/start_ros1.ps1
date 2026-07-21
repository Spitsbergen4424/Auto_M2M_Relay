$ErrorActionPreference = "Stop"

Push-Location $PSScriptRoot
try {
    docker info | Out-Null
    docker compose up --build --detach
    docker compose ps
    docker compose logs --tail 40 ros1
    Write-Host ""
    Write-Host "ROS1 Endpoint запускается на TCP-порту 10000."
    Write-Host "Проверьте статус через: .\status_ros1.ps1"
}
finally {
    Pop-Location
}
