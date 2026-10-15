param([string]$Compiler = 'g++')
$ErrorActionPreference = 'Stop'
$project = Split-Path $PSScriptRoot -Parent
$out = Join-Path $project 'test-results'
New-Item -ItemType Directory -Force -Path $out | Out-Null
if (Test-Path -LiteralPath $Compiler) { $env:PATH = (Split-Path $Compiler) + ';' + $env:PATH }
foreach ($test in @('line_pid', 'parking', 'route', 'wheel_speed')) {
    $arguments = @('-std=c++11', '-Wall', '-Wextra', '-I', "$project/include", '-I', "$project/lib/WheelSpeedControl/src", "$project/tests/${test}_test.cpp", '-o', "$out/${test}_test.exe")
    if ($test -eq 'wheel_speed') { $arguments += "$project/lib/WheelSpeedControl/src/WheelSpeedControl.cpp" }
    & $Compiler @arguments
    if ($LASTEXITCODE -ne 0) { throw "Compilation failed: $test" }
    & "$out/${test}_test.exe"
    if ($LASTEXITCODE -ne 0) { throw "Test failed: $test" }
}
