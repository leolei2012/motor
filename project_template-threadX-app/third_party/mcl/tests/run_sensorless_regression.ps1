# Run from any directory: pwsh -File tests/run_sensorless_regression.ps1
$ErrorActionPreference = 'Stop'
$mclRoot = Split-Path $PSScriptRoot -Parent
$sources = Get-ChildItem (Join-Path $mclRoot 'src') -Filter *.c | ForEach-Object FullName
$testNames = @('smo_regression_test', 'sensorless_startup_test', 'applied_voltage_test',
               'protection_test', 'fault_recovery_test')
foreach ($testName in $testNames) {
    $testSource = Join-Path $PSScriptRoot "$testName.c"
    $testExe = Join-Path $PSScriptRoot "$testName.exe"
    & gcc -std=c99 -O2 -Wall -Wextra "-I$(Join-Path $mclRoot 'include')" $testSource @sources -lm -o $testExe
    if ($LASTEXITCODE -ne 0) { throw "Compilation failed: $testName" }
    & $testExe
    if ($LASTEXITCODE -ne 0) { throw "Regression failed: $testName" }
}
Write-Host 'Sensorless regression suite passed.'
