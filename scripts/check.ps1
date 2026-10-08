param(
    [string]$ArduinoCli,
    [string]$AvrCompiler
)

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$buildRoot = Join-Path $projectRoot 'build/verification'

if (-not $ArduinoCli) {
    $cliCommand = Get-Command arduino-cli -ErrorAction SilentlyContinue
    if ($cliCommand) {
        $ArduinoCli = $cliCommand.Source
    } else {
        $ArduinoCli = Join-Path $env:ProgramFiles 'Arduino IDE/resources/app/lib/backend/resources/arduino-cli.exe'
    }
}
if (-not (Test-Path -LiteralPath $ArduinoCli)) {
    throw 'Arduino CLI не найден. Установите Arduino IDE или передайте -ArduinoCli.'
}

if (-not $AvrCompiler) {
    $toolRoot = Join-Path $env:LOCALAPPDATA 'Arduino15/packages/arduino/tools/avr-gcc'
    $toolVersions = Get-ChildItem -LiteralPath $toolRoot -Directory | Sort-Object LastWriteTime -Descending
    foreach ($toolVersion in $toolVersions) {
        $candidate = Join-Path $toolVersion.FullName 'bin/avr-g++.exe'
        if (Test-Path -LiteralPath $candidate) {
            $AvrCompiler = $candidate
            break
        }
    }
}
if (-not $AvrCompiler -or -not (Test-Path -LiteralPath $AvrCompiler)) {
    throw 'AVR-компилятор не найден. Установите Arduino AVR Boards или передайте -AvrCompiler.'
}

New-Item -ItemType Directory -Force -Path $buildRoot | Out-Null
$compileArguments = @('compile', '--fqbn', 'arduino:avr:nano:cpu=atmega328')
$libraryPaths = @(
    (Join-Path ([Environment]::GetFolderPath('MyDocuments')) 'Arduino/libraries'),
    (Join-Path $env:LOCALAPPDATA 'Arduino15/libraries')
)
foreach ($libraryPath in $libraryPaths) {
    if (Test-Path -LiteralPath $libraryPath) {
        $compileArguments += @('--libraries', $libraryPath)
    }
}
$compileArguments += @('--build-path', $buildRoot, (Join-Path $projectRoot 'firmware/wing_tester'))
& $ArduinoCli @compileArguments
if ($LASTEXITCODE -ne 0) { throw 'Сборка прошивки не прошла.' }

foreach ($testName in @('zero_statistics', 'flow_detection', 'reconnect_detection')) {
    $testSource = Join-Path $projectRoot "tests/${testName}_test.cpp"
    $testObject = Join-Path $buildRoot "${testName}_test.o"
    & $AvrCompiler -std=gnu++14 -mmcu=atmega328p -Wall -Wextra -c $testSource -o $testObject
    if ($LASTEXITCODE -ne 0) { throw "Проверка $testName не прошла." }
    Write-Output "$testName : OK"
}
Write-Output 'Прошивка и все три набора проверок собраны успешно.'
