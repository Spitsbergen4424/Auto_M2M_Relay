$ErrorActionPreference = "Stop"

Push-Location $PSScriptRoot
try {
    docker compose ps
    docker compose logs --tail 80 ros1
}
finally {
    Pop-Location
}
