# Fetches the pinned Open CASCADE Technology (OCCT) revision and builds the static
# modelling + STEP toolkits that crates/warp-import-step links against.
#
#   vendor/occt          pinned shallow checkout (git ignored)
#   vendor/occt-build    CMake build tree (git ignored)
#   vendor/occt-install  static libraries and headers (git ignored)
#
# One-time cost: about 20-40 minutes with 8 cores. Visualization, the OCAF
# application framework, Draw and all third-party dependencies are left out.
param([int]$Jobs = [Environment]::ProcessorCount)
$ErrorActionPreference = 'Stop'
$taskRoot = Split-Path -Parent $PSScriptRoot
$pin = (Get-Content -LiteralPath (Join-Path $taskRoot 'vendor/occt-pin.txt') -Raw).Trim()
$source = Join-Path $taskRoot 'vendor/occt'
$build = Join-Path $taskRoot 'vendor/occt-build'
$install = Join-Path $taskRoot 'vendor/occt-install'

if (Test-Path -LiteralPath $source) {
    $current = (& git -C $source rev-parse HEAD | Out-String).Trim()
    if ($LASTEXITCODE -ne 0 -or $current -ne $pin) { throw 'Existing OCCT checkout does not match the pinned revision. Move it aside before fetching; it has not been modified.' }
    Write-Host "OCCT already pinned at $pin"
} else {
    & git init $source
    if ($LASTEXITCODE -ne 0) { throw 'Could not initialize OCCT checkout' }
    & git -C $source remote add origin https://github.com/Open-Cascade-SAS/OCCT.git
    & git -C $source fetch --depth 1 origin $pin
    if ($LASTEXITCODE -ne 0) { throw 'OCCT download failed' }
    & git -C $source checkout --detach $pin
    if ($LASTEXITCODE -ne 0) { throw 'OCCT checkout failed' }
}

$cmake = $env:CMAKE
if (-not $cmake) {
    $vswhere = 'C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe'
    $vs = (& $vswhere -latest -products * -property installationPath | Out-String).Trim()
    $candidate = Join-Path $vs 'Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe'
    $cmake = if (Test-Path -LiteralPath $candidate) { $candidate } else { 'cmake' }
}

$pinFile = Join-Path $install 'warpnine-occt-pin.txt'
if ((Test-Path -LiteralPath $pinFile) -and ((Get-Content -LiteralPath $pinFile -Raw).Trim() -eq $pin)) {
    Write-Host "OCCT install already built from $pin"
    exit 0
}

& $cmake -S $source -B $build -A x64 `
    "-DINSTALL_DIR=$install" `
    '-DBUILD_LIBRARY_TYPE=Static' `
    '-DBUILD_MODULE_FoundationClasses=ON' `
    '-DBUILD_MODULE_ModelingData=ON' `
    '-DBUILD_MODULE_ModelingAlgorithms=ON' `
    '-DBUILD_MODULE_DataExchange=OFF' `
    '-DBUILD_MODULE_ApplicationFramework=OFF' `
    '-DBUILD_MODULE_Visualization=OFF' `
    '-DBUILD_MODULE_Draw=OFF' `
    '-DBUILD_ADDITIONAL_TOOLKITS=TKXSBase;TKDE;TKDESTEP' `
    '-DBUILD_DOC_Overview=OFF' `
    '-DBUILD_SAMPLES_MFC=OFF' '-DBUILD_SAMPLES_QT=OFF' `
    '-DBUILD_USE_PCH=ON' `
    '-DUSE_FREETYPE=OFF' '-DUSE_TK=OFF' '-DUSE_TBB=OFF' '-DUSE_RAPIDJSON=OFF' `
    '-DUSE_DRACO=OFF' '-DUSE_FREEIMAGE=OFF' '-DUSE_OPENGL=OFF' '-DUSE_GLES2=OFF' `
    '-DUSE_FFMPEG=OFF' '-DUSE_OPENVR=OFF' '-DUSE_VTK=OFF' '-DUSE_D3D=OFF' `
    '-DCMAKE_POLICY_DEFAULT_CMP0091=NEW' '-DCMAKE_MSVC_RUNTIME_LIBRARY=MultiThreaded' `
    '-DCMAKE_CXX_FLAGS_RELEASE=/MT /O2 /Ob2 /DNDEBUG' '-DCMAKE_C_FLAGS_RELEASE=/MT /O2 /Ob2 /DNDEBUG'
if ($LASTEXITCODE -ne 0) { throw 'OCCT configure failed' }
& $cmake --build $build --config Release --target install --parallel $Jobs
if ($LASTEXITCODE -ne 0) { throw 'OCCT build failed' }
Set-Content -LiteralPath $pinFile -Value $pin
Write-Host "OCCT installed to $install"
