$ErrorActionPreference = 'Stop'
$compiler = Join-Path $env:WINDIR 'Microsoft.NET/Framework64/v4.0.30319/csc.exe'
$testDir = Join-Path ([IO.Path]::GetTempPath()) ('pano-installer-tests-' + [Guid]::NewGuid())
New-Item -ItemType Directory -Path $testDir | Out-Null
$target = Join-Path $testDir 'DetectionPolicyTests.exe'
try {
    $sources = 'Installer.cs','DetectionPolicy.cs','DeviceDiscovery.cs','DetectionPolicyTests.cs' | ForEach-Object { Join-Path $PSScriptRoot $_ }
    & $compiler /nologo /target:exe /main:DetectionPolicyTests /platform:anycpu /optimize+ /warnaserror+ /codepage:65001 /r:System.Management.dll "/out:$target" $sources
    if ($LASTEXITCODE) { throw 'Test compilation failed.' }
    & $target
    if ($LASTEXITCODE) { throw 'Installer logic tests failed.' }
} finally {
    Remove-Item -LiteralPath $testDir -Recurse -Force
}
