param(
    [Parameter(Mandatory = $true)][string]$Version,
    [Parameter(Mandatory = $true)][ValidatePattern('^[0-9a-fA-F]{32}$')][string]$BuildId,
    [Parameter(Mandatory = $true)][string]$PrivateKey,
    [Parameter(Mandatory = $true)][string]$PublicKey,
    [string]$Uv4 = 'D:\soft\codeSoft\Keil_v5\UV4\UV4.exe'
)

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
Set-Location $root
foreach ($path in @($PrivateKey, $PublicKey, $Uv4)) {
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) { throw "文件不存在: $path" }
}
New-Item -ItemType Directory -Force -Path '.\build\ntn-ota' | Out-Null

function Build-Target([string]$Project) {
    $log = ".\build\ntn-ota\release-$($Project -replace '\.uvprojx$', '').log"
    $target = $Project -replace '\.uvprojx$', ''
    Remove-Item -LiteralPath $log -Force -ErrorAction SilentlyContinue
    $process = Start-Process -FilePath $Uv4 -ArgumentList @('-b', $Project, '-q', '-j0', '-t', $target, '-o', $log) -PassThru
    for ($second = 0; $second -lt 600; $second++) {
        if ((Test-Path -LiteralPath $log) -and (Select-String -LiteralPath $log -Pattern '[0-9]+ Error\(s\)' -Quiet)) {
            if (-not $process.HasExited) { Stop-Process -Id $process.Id -ErrorAction SilentlyContinue }
            if (Select-String -LiteralPath $log -Pattern '0 Error\(s\)' -Quiet) { return }
            throw "$Project 编译失败，请查看 $log"
        }
        Start-Sleep -Seconds 1
    }
    if (-not $process.HasExited) { Stop-Process -Id $process.Id -ErrorAction SilentlyContinue }
    throw "$Project 在 600 秒内没有生成构建结果，请查看 $log"
}

python .\boot\prepare_mdk.py --build-id $BuildId --public-key $PublicKey
Build-Target 'ntn-ota-bl2.uvprojx'
python .\boot\package_image.py extract-bl2
python .\boot\prepare_mdk.py --bl2-bin .\build\ntn-ota\bl2.bin --watchdog
Build-Target 'ntn-ota-bl1.uvprojx'
Build-Target 'ntn-ota-app.uvprojx'
python .\boot\package_image.py provision --version $Version --key $PrivateKey --public-key $PublicKey
Get-FileHash .\build\ntn-ota\ntn-ota-l0.hex, .\build\ntn-ota\app.signed.bin, .\build\ntn-ota\factory.signed.bin -Algorithm SHA256
