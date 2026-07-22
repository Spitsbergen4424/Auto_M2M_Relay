[CmdletBinding()]
param(
    [Alias('stream-url')]
    [string]$StreamUrl = 'http://192.168.2.154:8080/?action=stream',

    [Alias('model')]
    [string]$Model = 'models\best_detect.pt',

    [Alias('udp-ip')]
    [string]$UdpIp = '127.0.0.1',

    [Alias('udp-port')]
    [ValidateRange(1, 65535)]
    [int]$UdpPort = 5005,

    [ValidateRange(0.0, 1.0)]
    [double]$Confidence = 0.20,

    [Alias('ball-class')]
    [int]$BallClass = 0,

    [switch]$NoPreview
)

$ErrorActionPreference = 'Stop'
Set-Location $PSScriptRoot

$Python = Join-Path $PSScriptRoot '.venv\Scripts\python.exe'
if (-not (Test-Path -LiteralPath $Python -PathType Leaf)) {
    throw 'Python environment was not found. Run .\install.ps1 first.'
}

$ModelPath = if ([System.IO.Path]::IsPathRooted($Model)) {
    $Model
}
else {
    Join-Path $PSScriptRoot $Model
}

if (-not (Test-Path -LiteralPath $ModelPath)) {
    throw "YOLO model was not found: $ModelPath"
}

$YoloArguments = @(
    (Join-Path $PSScriptRoot 'yolo_vision_node.py')
    '--stream-url', $StreamUrl
    '--model', $ModelPath
    '--udp-ip', $UdpIp
    '--udp-port', $UdpPort
    '--confidence', $Confidence.ToString([System.Globalization.CultureInfo]::InvariantCulture)
    '--ball-class', $BallClass
)

if ($NoPreview) {
    $YoloArguments += '--no-preview'
}

Write-Host "Camera: $StreamUrl"
Write-Host "Model:  $ModelPath"
Write-Host "Unity:  ${UdpIp}:$UdpPort"

& $Python @YoloArguments
if ($LASTEXITCODE -ne 0) {
    throw "YOLO node exited with code $LASTEXITCODE."
}
