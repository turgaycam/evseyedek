param(
    [Parameter(Mandatory = $true)]
    [string]$BinPath,

    [Parameter(Mandatory = $true)]
    [string]$Version,

    [string]$TargetName = "uzaktanotali.bin",

    [string]$HealthUrl = "",

    [switch]$Push
)

$ErrorActionPreference = "Stop"

$projectRoot = Split-Path -Parent $PSScriptRoot
$firmwareDir = Join-Path $projectRoot "firmware"
$targetBin = Join-Path $firmwareDir $TargetName
$versionJson = Join-Path $projectRoot "version.json"

if (!(Test-Path -LiteralPath $BinPath)) {
    throw "Bin dosyasi bulunamadi: $BinPath"
}

if (!(Test-Path -LiteralPath $firmwareDir)) {
    New-Item -ItemType Directory -Path $firmwareDir | Out-Null
}

Copy-Item -LiteralPath $BinPath -Destination $targetBin -Force

$sizeBytes = (Get-Item -LiteralPath $targetBin).Length
$sha256 = (Get-FileHash -LiteralPath $targetBin -Algorithm SHA256).Hash.ToLowerInvariant()

$manifest = [ordered]@{
    version = $Version
    url = "https://raw.githubusercontent.com/turgaycam/evseyedek/backup-new-board-ota/firmware/$TargetName"
}
if (![string]::IsNullOrWhiteSpace($HealthUrl)) {
    $manifest.health_url = $HealthUrl
}

$manifest | ConvertTo-Json | Set-Content -LiteralPath $versionJson -Encoding utf8

Write-Host "Hazir:"
Write-Host "BIN: $targetBin"
Write-Host "Version: $Version"
Write-Host "URL: $($manifest.url)"
Write-Host "Size: $($manifest.size)"
Write-Host "SHA256: $($manifest.sha256)"
if ($manifest.Contains("health_url")) {
    Write-Host "Health URL: $($manifest.health_url)"
}

if ($Push) {
    Push-Location $projectRoot
    try {
        git add "firmware/$TargetName" "version.json"
        if ($LASTEXITCODE -ne 0) {
            throw "git add basarisiz"
        }

        git commit -m "OTA $Version"
        if ($LASTEXITCODE -ne 0) {
            throw "git commit basarisiz"
        }

        git push origin backup-new-board-ota
        if ($LASTEXITCODE -ne 0) {
            throw "git push basarisiz"
        }

        Write-Host ""
        Write-Host "GitHub'a gonderildi."
    }
    finally {
        Pop-Location
    }
} else {
    Write-Host ""
    Write-Host "Sonraki adim:"
    Write-Host "git add firmware/$TargetName version.json"
    Write-Host "git commit -m 'OTA $Version'"
    Write-Host "git push origin backup-new-board-ota"
}