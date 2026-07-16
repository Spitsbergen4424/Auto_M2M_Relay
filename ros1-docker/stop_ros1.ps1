$ErrorActionPreference = "Stop"

Push-Location $PSScriptRoot
try {
    docker compose down
}
finally {
    Pop-Location
}
