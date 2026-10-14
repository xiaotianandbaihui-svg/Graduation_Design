param([string]$Compiler = "gcc")
$ErrorActionPreference = "Stop"
$source = Join-Path $PSScriptRoot "..\mpu9250_simple.c"
$test = Join-Path $PSScriptRoot "test_mpu9250.c"
$binary = Join-Path ([System.IO.Path]::GetTempPath()) ("mpu9250-test-" + [guid]::NewGuid().ToString("N") + ".exe")
try {
    & $Compiler -std=c99 -Wall -Wextra -Werror -Wmissing-prototypes -Wstrict-prototypes -pedantic -fsyntax-only $source
    if ($LASTEXITCODE -ne 0) { throw "Driver syntax check failed." }
    & $Compiler -std=c99 -O2 -Wall -Wextra -Werror -pedantic $test -lm -o $binary
    if ($LASTEXITCODE -ne 0) { throw "Test build failed." }
    & $binary
    if ($LASTEXITCODE -ne 0) { throw "Driver tests failed." }
} finally {
    if (Test-Path -LiteralPath $binary) { Remove-Item -LiteralPath $binary }
}
