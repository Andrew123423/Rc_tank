# Run on Windows (or via powershell.exe from WSL). Does not stop running WSL sessions.
$ErrorActionPreference = 'Stop'
$TankSystem = Get-CimInstance Win32_ComputerSystem
$TankCpuCount = [int]$TankSystem.NumberOfLogicalProcessors
$TankRamMB = [int][Math]::Floor([double]$TankSystem.TotalPhysicalMemory / 1MB * 0.75)
$TankConfigPath = Join-Path $env:USERPROFILE '.wslconfig'
$script:TankConfig = ''
if (Test-Path $TankConfigPath) {
    Copy-Item $TankConfigPath "$TankConfigPath.backup-$(Get-Date -Format 'yyyyMMdd-HHmmss')"
    $script:TankConfig = [IO.File]::ReadAllText($TankConfigPath)
}
function Set-TankWslOption([string]$Name, [string]$Value) {
    $Section = [regex]::Match($script:TankConfig, '(?ims)(^\[wsl2\][ \t]*(?:\r?\n|\z))(.*?)(?=^\[|\z)')
    if (-not $Section.Success) {
        $script:TankConfig = $script:TankConfig.TrimEnd() + "`r`n[wsl2]`r`n$Name=$Value`r`n"
        return
    }
    $Body = $Section.Groups[2].Value
    $KeyPattern = '(?im)^[ \t]*' + [regex]::Escape($Name) + '[ \t]*=.*$'
    if ([regex]::IsMatch($Body, $KeyPattern)) {
        $Body = [regex]::Replace($Body, $KeyPattern, "$Name=$Value")
    } else {
        $Body = $Body.TrimEnd() + "`r`n$Name=$Value`r`n"
    }
    $script:TankConfig = $script:TankConfig.Substring(0, $Section.Index) + $Section.Groups[1].Value + $Body + $script:TankConfig.Substring($Section.Index + $Section.Length)
}
Set-TankWslOption 'processors' "$TankCpuCount"
Set-TankWslOption 'memory' "${TankRamMB}MB"
Set-TankWslOption 'guiApplications' 'true'
[IO.File]::WriteAllText($TankConfigPath, $script:TankConfig, [Text.UTF8Encoding]::new($false))
Write-Host "Updated: $TankConfigPath"
Write-Host "Logical CPUs: $TankCpuCount; RAM limit: ${TankRamMB}MB (75% of physical RAM)."
Write-Host 'Save work in ALL WSL distributions, then run wsl --shutdown in Windows PowerShell.'
