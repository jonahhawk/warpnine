$ErrorActionPreference = 'Stop'
$taskRoot = Split-Path -Parent $PSScriptRoot
$pin = (Get-Content -LiteralPath (Join-Path $taskRoot 'vendor/opennurbs-pin.txt') -Raw).Trim()
$destination = Join-Path $taskRoot 'vendor/opennurbs'
if (Test-Path -LiteralPath $destination) {
    $current = (& git -C $destination rev-parse HEAD | Out-String).Trim()
    if ($LASTEXITCODE -ne 0 -or $current -ne $pin) { throw 'Existing SDK does not match the pinned revision. Move it aside before fetching; it has not been modified.' }
    Write-Host "openNURBS already pinned at $pin"
    exit 0
}
& git init $destination
if ($LASTEXITCODE -ne 0) { throw 'Could not initialize SDK checkout' }
& git -C $destination remote add origin https://github.com/mcneel/opennurbs.git
& git -C $destination fetch --depth 1 origin $pin
if ($LASTEXITCODE -ne 0) { throw 'SDK download failed' }
& git -C $destination checkout --detach $pin
if ($LASTEXITCODE -ne 0) { throw 'SDK checkout failed' }
