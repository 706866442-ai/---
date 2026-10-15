param(
    [ValidateSet('all','default','vision')][string]$Mode = 'all',
    [string]$Pio = 'C:/Users/0/.platformio/penv/Scripts/pio.exe',
    [string]$Python = 'C:/Users/0/.platformio/penv/Scripts/python.exe'
)
$ErrorActionPreference = 'Stop'
$project = Split-Path $PSScriptRoot -Parent
$scratch = 'D:/CodexWorkspace/scratch/line-follower-completion-20261015'
New-Item -ItemType Directory -Force -Path $scratch | Out-Null
$env:PLATFORMIO_CORE_DIR = 'C:/Users/0/AppData/Local/Temp/pio_core_car_task'
if ($Mode -ne 'vision') {
    & $Pio run -d $project *> "$scratch/default-build.log"
    if ($LASTEXITCODE -ne 0) { Get-Content "$scratch/default-build.log" -Tail 35; throw 'Default firmware failed' }
    Get-Content "$scratch/default-build.log" -Tail 7
}
if ($Mode -ne 'default') {
    $stage = Join-Path $scratch 'firmware-enabled-verify'
    New-Item -ItemType Directory -Force -Path "$stage/src", "$stage/lib" | Out-Null
    Copy-Item -LiteralPath "$project/src/main.cpp" -Destination "$stage/src/main.cpp" -Force
    Copy-Item -LiteralPath "$project/lib/WheelSpeedControl" -Destination "$stage/lib" -Recurse -Force
    & $Python "$project/tests/prepare_test_config.py" "$stage/include" vision
    if ($LASTEXITCODE -ne 0) { throw 'Synthetic configuration failed' }
    @"
[env:bluepill_f103c8]
platform = ststm32
board = bluepill_f103c8
framework = arduino
upload_protocol = stlink
[platformio]
core_dir = C:/Users/0/AppData/Local/Temp/pio_core_car_task
build_dir = $scratch/build-enabled-verify
"@ | Set-Content -LiteralPath "$stage/platformio.ini" -Encoding ascii
    'SYNTHETIC TEST VALUES. DO NOT UPLOAD THIS PROJECT OR FIRMWARE.' | Set-Content -LiteralPath "$stage/DO_NOT_UPLOAD.txt"
    & $Pio run -d $stage *> "$scratch/enabled-build.log"
    if ($LASTEXITCODE -ne 0) { Get-Content "$scratch/enabled-build.log" -Tail 35; throw 'Enabled firmware failed' }
    Get-Content "$scratch/enabled-build.log" -Tail 7
}
