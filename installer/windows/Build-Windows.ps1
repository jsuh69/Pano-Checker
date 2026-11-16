$ErrorActionPreference = 'Stop'
$compiler = Join-Path $env:WINDIR 'Microsoft.NET/Framework64/v4.0.30319/csc.exe'
$target = Join-Path $PSScriptRoot 'ESP32-Installer.exe'
& $compiler /nologo /target:winexe /platform:anycpu /optimize+ /warnaserror+ /codepage:65001 /r:System.Management.dll "/out:$target" (Join-Path $PSScriptRoot 'Installer.cs') (Join-Path $PSScriptRoot 'DetectionPolicy.cs') (Join-Path $PSScriptRoot 'DeviceDiscovery.cs')
if ($LASTEXITCODE) { throw 'Windows installer compilation failed.' }
Write-Host "Built: $target"
