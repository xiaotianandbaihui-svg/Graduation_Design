param(
    [string]$Gcc = 'D:/LenovoSoftstore/VSCode/MinGW/bin/gcc.exe',
    [string]$ArmBin = 'D:/My_Keil/Keil_v5/ARM/ARMCLANG/bin',
    [string]$Device = 'D:/My_Keil/Keil_v5/Packs/Keil/STM32F1xx_DFP/2.4.1/Device',
    [string]$Cmsis = 'D:/My_Keil/Keil_v5/Packs/ARM/CMSIS/5.9.0/CMSIS/Core/Include'
)
$ErrorActionPreference = 'Stop'
$sourceRoot = Split-Path -Parent $PSScriptRoot
# 使用ASCII临时目录，兼容MinGW链接器；不删除用户文件。
$buildRoot = Join-Path $env:TEMP ('bds3620_validate_' + [Guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $buildRoot | Out-Null
$testExe = Join-Path $buildRoot 'test_motor.exe'
& $Gcc -std=c99 -O2 -Wall -Wextra -Werror -I $sourceRoot `
    (Join-Path $sourceRoot 'tle5012b.c') (Join-Path $sourceRoot 'motor.c') `
    (Join-Path $PSScriptRoot 'test_motor.c') -o $testExe
if ($LASTEXITCODE -ne 0) { throw 'Host compilation failed' }
& $testExe
if ($LASTEXITCODE -ne 0) { throw 'Host tests failed' }

$compiler = Join-Path $ArmBin 'armclang.exe'
$flags = @('--target=arm-arm-none-eabi','-mcpu=cortex-m3','-mthumb','-std=c99',
    '-O2','-ffunction-sections','-fdata-sections','-Wall','-Wextra','-Werror',
    '-DSTM32F10X_MD','-DUSE_STDPERIPH_DRIVER',
    '-I',$sourceRoot,'-I',"$Device/Include",'-I',"$Device/StdPeriph_Driver/inc",'-I',$Cmsis)
$objects = @()
foreach ($name in @('tle5012b.c','motor.c','motor_port_stm32f103.c','example_main.c')) {
    $obj = Join-Path $buildRoot "$name.o"
    & $compiler @flags -c (Join-Path $sourceRoot $name) -o $obj
    if ($LASTEXITCODE -ne 0) { throw "ARM compilation failed: $name" }
    $objects += $obj
}
foreach ($name in @('stm32f10x_rcc','stm32f10x_gpio','stm32f10x_spi','stm32f10x_tim','stm32f10x_exti','misc')) {
    $obj = Join-Path $buildRoot "$name.o"
    & $compiler @flags -c "$Device/StdPeriph_Driver/src/$name.c" -o $obj
    if ($LASTEXITCODE -ne 0) { throw "SPL compilation failed: $name" }
    $objects += $obj
}
$obj = Join-Path $buildRoot 'system_stm32f10x.o'
& $compiler @flags -c "$Device/Source/system_stm32f10x.c" -o $obj
if ($LASTEXITCODE -ne 0) { throw 'System compilation failed' }
$objects += $obj
$obj = Join-Path $buildRoot 'startup_stm32f10x_md.o'
& (Join-Path $ArmBin 'armasm.exe') --cpu Cortex-M3 "$Device/Source/ARM/startup_stm32f10x_md.s" -o $obj
if ($LASTEXITCODE -ne 0) { throw 'Startup assembly failed' }
$objects += $obj
Copy-Item -LiteralPath (Join-Path $PSScriptRoot 'f103c8.sct') -Destination (Join-Path $buildRoot 'f103c8.sct')
& (Join-Path $ArmBin 'armlink.exe') --cpu Cortex-M3 --entry Reset_Handler `
    --scatter (Join-Path $buildRoot 'f103c8.sct') --info sizes --map `
    --list (Join-Path $buildRoot 'motor.map') @objects -o (Join-Path $buildRoot 'motor.axf')
if ($LASTEXITCODE -ne 0) { throw 'ARM linking failed' }
# 再链接启用标定/运行分支的示例，确保这部分在真实ARM库中也能链接。
& $compiler @flags -DEXAMPLE_CALIBRATE_AND_RUN=1 -c (Join-Path $sourceRoot 'example_main.c') `
    -o (Join-Path $buildRoot 'example_main.c.o')
if ($LASTEXITCODE -ne 0) { throw 'Calibration example compilation failed' }
& (Join-Path $ArmBin 'armlink.exe') --cpu Cortex-M3 --entry Reset_Handler `
    --scatter (Join-Path $buildRoot 'f103c8.sct') --info sizes --map `
    --list (Join-Path $buildRoot 'motor_with_calibration.map') @objects `
    -o (Join-Path $buildRoot 'motor_with_calibration.axf')
if ($LASTEXITCODE -ne 0) { throw 'Calibration example linking failed' }
Write-Output "PASS: Both Cortex-M3/SPL examples link in 64KB Flash / 20KB RAM layout. Output: $buildRoot"
