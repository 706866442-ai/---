param([string]$Compiler = 'g++')
$ErrorActionPreference = 'Stop'
$project = Split-Path $PSScriptRoot -Parent
$out = 'D:/CodexWorkspace/scratch/line-follower-completion-20261015/host-tests'
New-Item -ItemType Directory -Force -Path $out | Out-Null
if (Test-Path -LiteralPath $Compiler) { $env:PATH = (Split-Path $Compiler) + ';' + $env:PATH }
foreach ($test in @('line_pid', 'parking', 'route', 'wheel_speed', 'speed_loop', 'mission', 'vision', 'telemetry')) {
    $arguments = @('-std=c++11', '-Wall', '-Wextra', '-I', "$project/include", '-I', "$project/lib/WheelSpeedControl/src", "$project/tests/${test}_test.cpp", '-o', "$out/${test}_test.exe")
    if ($test -eq 'wheel_speed' -or $test -eq 'speed_loop') { $arguments += "$project/lib/WheelSpeedControl/src/WheelSpeedControl.cpp" }
    & $Compiler @arguments
    if ($LASTEXITCODE -ne 0) { throw "Compilation failed: $test" }
    & "$out/${test}_test.exe"
    if ($LASTEXITCODE -ne 0) { throw "Test failed: $test" }
}

foreach ($variant in @('diagnostic', 'vision')) {
    $config = Join-Path $out "config-$variant"
    & 'C:/Users/0/.platformio/penv/Scripts/python.exe' "$project/tests/prepare_test_config.py" $config $variant
    if ($LASTEXITCODE -ne 0) { throw "Configuration generation failed: $variant" }
    & $Compiler -std=c++11 -Wall -Wextra -I "$project/tests/mocks" -I $config -I "$project/lib/WheelSpeedControl/src" "$project/tests/main_integration_test.cpp" "$project/lib/WheelSpeedControl/src/WheelSpeedControl.cpp" -o "$out/main_$variant.exe"
    if ($LASTEXITCODE -ne 0) { throw "Integration compilation failed: $variant" }
    & "$out/main_$variant.exe"
    if ($LASTEXITCODE -ne 0) { throw "Integration test failed: $variant" }
}

& 'C:/Users/0/.platformio/penv/Scripts/python.exe' -B "$project/tests/tools_test.py"
if ($LASTEXITCODE -ne 0) { throw 'Python tools test failed' }
