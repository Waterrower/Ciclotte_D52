param(
    [ValidateSet("devkit_spi")]
    [string]$Profile = "devkit_spi",

    [ValidateSet("app", "dfu")]
    [string]$Target = "dfu",

    [switch]$Pristine
)

$ErrorActionPreference = "Stop"

$root = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$profileDir = Join-Path $root "hardware\$Profile"
$extraConf = Join-Path $profileDir "prj_extra.conf"
$zephyrBase = $env:ZEPHYR_BASE

if ([string]::IsNullOrWhiteSpace($zephyrBase)) {
    $zephyrBase = "C:\ncs\v3.3.1\zephyr"
}

if (!(Test-Path $profileDir)) {
    throw "Unknown hardware profile: $Profile"
}

$rootCmake = $root -replace "\\", "/"
$extraConfCmake = $extraConf -replace "\\", "/"
$zephyrBaseCmake = $zephyrBase -replace "\\", "/"
$pmStatic = Join-Path $root "pm_static.yml"
$pmStaticCmake = $pmStatic -replace "\\", "/"

# Sysbuild names the application image after the project folder, so the
# per-image overrides and sysbuild/<image>/ must use that name.
$imageName = Split-Path $root -Leaf
if (!(Test-Path (Join-Path $root "sysbuild\$imageName"))) {
    throw "sysbuild\$imageName not found - rename the sysbuild image folder to match the project folder name"
}

if ($Target -eq "dfu") {
    $buildDir = Join-Path $root "build_ciclotte_dfu"
    $overlay = Join-Path $profileDir "sysbuild_app.overlay"
    $sourceDir = Join-Path $zephyrBase "share\sysbuild"
    $sourceDirCmake = $sourceDir -replace "\\", "/"
    $overlayCmake = $overlay -replace "\\", "/"
    $cmakeArgs = @(
        "-S", $sourceDirCmake,
        "-B", $buildDir,
        "-GNinja",
        "-DBOARD=nrf52dk/nrf52832",
        "-DAPP_DIR=$rootCmake",
        "-DSB_CONF_FILE=$rootCmake/sysbuild.conf",
        "-DPM_STATIC_YML_FILE=$pmStaticCmake",
        "-D${imageName}_DTC_OVERLAY_FILE=$overlayCmake",
        "-Dmcuboot_DTC_OVERLAY_FILE=$overlayCmake",
        "-D${imageName}_EXTRA_CONF_FILE=$extraConfCmake"
    )
} else {
    $buildDir = Join-Path $root "build_ciclotte_app"
    $overlay = Join-Path $profileDir "app.overlay"
    $overlayCmake = $overlay -replace "\\", "/"
    $cmakeArgs = @(
        "-S", $rootCmake,
        "-B", $buildDir,
        "-GNinja",
        "-DBOARD=nrf52dk/nrf52832",
        "-DDTC_OVERLAY_FILE=$overlayCmake",
        "-DEXTRA_CONF_FILE=$extraConfCmake"
    )
}

Write-Host "Profile: $Profile"
Write-Host "Target : $Target"
Write-Host "Build  : $buildDir"
Write-Host "Overlay: $overlay"

if ($Pristine -and (Test-Path $buildDir)) {
    Remove-Item -Recurse -Force $buildDir
}

cmake @cmakeArgs
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}

cmake --build $buildDir
exit $LASTEXITCODE
