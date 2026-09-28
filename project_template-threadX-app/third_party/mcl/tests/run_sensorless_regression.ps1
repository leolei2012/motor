# Run from any directory: pwsh -File tests/run_sensorless_regression.ps1
$ErrorActionPreference = 'Stop'
$mclRoot = Split-Path $PSScriptRoot -Parent
$sources = Get-ChildItem (Join-Path $mclRoot 'src') -Filter *.c | ForEach-Object FullName
foreach ($precision in @('FLOAT', 'MCL_USE_Q15', 'MCL_USE_Q31')) {
    $defines = @()
    if ($precision -ne 'FLOAT') { $defines = @("-D$precision") }
    $testExe = Join-Path $env:TEMP "config_value_$precision.exe"
    & gcc -std=c99 -O2 -Wall -Wextra @defines "-I$(Join-Path $mclRoot 'include')" (Join-Path $PSScriptRoot 'config_value_test.c') -o $testExe
    if ($LASTEXITCODE -ne 0) { throw "Compilation failed: $precision constants" }
    & $testExe
    if ($LASTEXITCODE -ne 0) { throw "Regression failed: $precision constants" }
    $protectionExe = Join-Path $env:TEMP "protection_module_$precision.exe"
    & gcc -std=c99 -O2 -Wall -Wextra @defines "-I$(Join-Path $mclRoot 'include')" (Join-Path $PSScriptRoot 'protection_module_test.c') (Join-Path $mclRoot 'src/mcl_protection.c') -lm -o $protectionExe
    if ($LASTEXITCODE -ne 0) { throw "Compilation failed: $precision standalone protection" }
    & $protectionExe
    if ($LASTEXITCODE -ne 0) { throw "Regression failed: $precision standalone protection" }
}
$testNames = @('smo_regression_test', 'sensorless_startup_test', 'applied_voltage_test',
               'protection_test', 'fault_recovery_test', 'avs_test')
foreach ($testName in $testNames) {
    $testSource = Join-Path $PSScriptRoot "$testName.c"
    $testExe = Join-Path $PSScriptRoot "$testName.exe"
    & gcc -std=c99 -O2 -Wall -Wextra "-I$(Join-Path $mclRoot 'include')" $testSource @sources -lm -o $testExe
    if ($LASTEXITCODE -ne 0) { throw "Compilation failed: $testName" }
    & $testExe
    if ($LASTEXITCODE -ne 0) { throw "Regression failed: $testName" }
}
Write-Host 'Sensorless regression suite passed.'
foreach ($precision in @('MCL_USE_Q15', 'MCL_USE_Q31')) {
    $testExe = Join-Path $env:TEMP "avs_$precision.exe"
    & gcc -std=c99 -O2 -Wall -Wextra "-D$precision" "-I$(Join-Path $mclRoot 'include')" (Join-Path $PSScriptRoot 'avs_precision_test.c') @sources -lm -o $testExe
    if ($LASTEXITCODE -ne 0) { throw "Compilation failed: $precision AVS" }
    & $testExe
    if ($LASTEXITCODE -ne 0) { throw "Regression failed: $precision AVS" }
}

$driverInclude = Join-Path $mclRoot '../../drivers/motor/include'
foreach ($precision in @('FLOAT', 'MCL_USE_Q15', 'MCL_USE_Q31')) {
    $defines = @()
    if ($precision -ne 'FLOAT') { $defines = @("-D$precision") }
    $testExe = Join-Path $env:TEMP "board_precision_$precision.exe"
    & gcc -std=c99 -O2 -Wall -Wextra @defines "-I$(Join-Path $mclRoot 'include')" "-I$driverInclude" (Join-Path $PSScriptRoot 'board_precision_test.c') @sources -lm -o $testExe
    if ($LASTEXITCODE -ne 0) { throw "Compilation failed: $precision board model" }
    & $testExe
    if ($LASTEXITCODE -ne 0) { throw "Regression failed: $precision board model" }
}
Write-Host 'All precision regressions passed.'
