param([string]$OutputDirectory = 'target/cam-baseline')
$ErrorActionPreference = 'Stop'
$taskRoot = Split-Path -Parent $PSScriptRoot
Push-Location $taskRoot
try {
    New-Item -ItemType Directory -Force -Path $OutputDirectory | Out-Null
    $reportPath = Join-Path $OutputDirectory 'm0-measurements.json'
    & cargo run -p warp-cam --example cam_baseline --release -- $reportPath
    if ($LASTEXITCODE -ne 0) { throw 'Baseline runner failed' }

    $ramBytes = $null
    $ramDiagnostic = $null
    try { $ramBytes = (Get-CimInstance Win32_ComputerSystem).TotalPhysicalMemory }
    catch { $ramDiagnostic = $_.Exception.Message }
    $fixtureHashes = [ordered]@{}
    foreach ($file in Get-ChildItem -LiteralPath 'fixtures/cam-baseline' -File | Sort-Object Name) {
        $fixtureHashes[$file.Name] = (Get-FileHash -LiteralPath $file.FullName -Algorithm SHA256).Hash
    }
    $sourceHashes = [ordered]@{}
    $sourceFiles = @(Get-ChildItem -LiteralPath 'crates' -Recurse -File | Where-Object { $_.Extension -in '.rs', '.toml' })
    $sourceFiles += Get-Item -LiteralPath 'Cargo.toml', 'Cargo.lock', '.cargo/config.toml', 'scripts/generate_cam_baseline.py'
    foreach ($file in $sourceFiles | Sort-Object FullName) {
        $relative = [IO.Path]::GetRelativePath($taskRoot, $file.FullName).Replace('\', '/')
        $sourceHashes[$relative] = (Get-FileHash -LiteralPath $file.FullName -Algorithm SHA256).Hash
    }
    $environment = [ordered]@{
        captured_utc = [DateTime]::UtcNow.ToString('o')
        os = [Environment]::OSVersion.VersionString
        cpu = $env:PROCESSOR_IDENTIFIER
        logical_processors = [Environment]::ProcessorCount
        physical_ram_bytes = $ramBytes
        ram_diagnostic = $ramDiagnostic
        rustc_version = (& rustc --version | Out-String).Trim()
        cargo_version = (& cargo --version | Out-String).Trim()
        build_profile = 'release'
        rustflags = '-C target-feature=+crt-static (workspace .cargo/config.toml)'
        fixture_sha256 = $fixtureHashes
        source_sha256 = $sourceHashes
    }
    $environment | ConvertTo-Json -Depth 6 | Set-Content -LiteralPath (Join-Path $OutputDirectory 'environment.json') -Encoding utf8
    Write-Output "Baseline report and environment saved to $OutputDirectory"
}
finally { Pop-Location }
