$script:CbScriptsDir = $PSScriptRoot

function Invoke-CbRcon {
    param([int]$Port, [string]$Command)
    $out = try { & "$script:CbScriptsDir\rcon.ps1" -Port $Port -Command $Command } catch { "" }
    if ($null -eq $out) { return "" }
    return ($out | Out-String)
}

function Invoke-CbRconQuiet {
    param([int]$Port, [string]$Command)
    try { & "$script:CbScriptsDir\rcon.ps1" -Port $Port -Command $Command | Out-Null } catch { }
}

function Get-CbDistance3D {
    param($ax, $ay, $az, $bx, $by, $bz)
    return [math]::Sqrt(($ax-$bx)*($ax-$bx) + ($ay-$by)*($ay-$by) + ($az-$bz)*($az-$bz))
}

function Copy-CbMap {
    param([string]$MapPath, [string]$HldsMapPath)
    if ((Resolve-Path $MapPath).Path -ne $HldsMapPath) { Copy-Item $MapPath $HldsMapPath -Force }
}

function Ensure-CbNav {
    param([string]$HldsMap, [string]$NavPath, [string]$NavGenExe, [switch]$Force)
    $navDir = Split-Path $NavPath -Parent
    if (-not (Test-Path $navDir)) { New-Item -ItemType Directory -Path $navDir | Out-Null }
    if ($Force -or -not (Test-Path $NavPath)) {
        & $NavGenExe $HldsMap -o $NavPath 2>&1 |
            Select-Object -Last 3 | ForEach-Object { Write-Host "navgen: $_" }
    }
}

function Get-CbServerPid {
    param([string]$StartResult)
    if ($StartResult -match "^EXITED") { Write-Host "FAIL: $StartResult"; exit 1 }
    return [int]($StartResult -replace "PID=", "")
}

function Assert-Cb {
    param([bool]$Ok, [string]$What, [System.Collections.ArrayList]$FailureList = $null)
    if ($null -eq $FailureList) { $FailureList = Get-Variable -Name failures -ValueOnly -ErrorAction SilentlyContinue }
    $tag = if ($Ok) { "PASS" } else { "FAIL" }
    Write-Host "  [$tag] $What"
    if (-not $Ok -and $null -ne $FailureList) { [void]$FailureList.Add($What) }
}

function Measure-CbCount {
    param($Lines, [string]$Pattern)
    return @($Lines | Select-String -Pattern $Pattern).Count
}

function Invoke-CbPhase {
    param([string]$Name, [string]$Map, [int]$Port, [string[]]$Boot,
        [scriptblock]$Body, [string]$LogDir, [string]$HldsDir,
        [string]$LogPrefix, [string]$ConfigPrefix,
        [System.Collections.ArrayList]$Failures, [string[]]$Games = @("valve"))
    $log = Join-Path $LogDir "${LogPrefix}_$Name.log"
    if (Test-Path $log) { Remove-Item $log }

    Write-Host ""
    Write-Host "=== $Name ($Map) ==="

    $start = & "$script:CbScriptsDir\test_server_start.ps1" -Map $Map -Port $Port -LogFile $log `
        -WaitSeconds 90 -ConfigName "${ConfigPrefix}_$Name.cfg" -Commands $Boot -HLDSDir $HldsDir `
        -Games $Games -RequireVerbose
    if ($start -notmatch 'PID=(\d+)') {
        Assert-Cb $false "$Name server started ($start)" $Failures
        return @()
    }
    $srv = [int]$Matches[1]

    try {
        & $Body $Port
    } finally {
        & "$script:CbScriptsDir\test_server_stop.ps1" -Id $srv | Out-Null
    }

    if (-not (Test-Path $log)) { return @() }
    return Get-Content $log
}

function Get-CbSpawns {
    param([string]$Log)
    $spawns = @()
    foreach ($m in [regex]::Matches($Log, "CrossBot: spawn (\S+) \(([-\d.]+),([-\d.]+),([-\d.]+)\)")) {
        $spawns += [pscustomobject]@{
            Classname = $m.Groups[1].Value
            X = [double]$m.Groups[2].Value
            Y = [double]$m.Groups[3].Value
            Z = [double]$m.Groups[4].Value
        }
    }
    return @($spawns | Sort-Object X,Y,Z -Unique)
}

function Get-CbFarthestPair {
    param($Spawns)
    $bestDist = -1; $bestA = $null; $bestB = $null
    for ($i = 0; $i -lt $Spawns.Count; $i++) {
        for ($j = $i + 1; $j -lt $Spawns.Count; $j++) {
            $d = Get-CbDistance3D $Spawns[$i].X $Spawns[$i].Y $Spawns[$i].Z $Spawns[$j].X $Spawns[$j].Y $Spawns[$j].Z
            if ($d -gt $bestDist) { $bestDist = $d; $bestA = $Spawns[$i]; $bestB = $Spawns[$j] }
        }
    }
    return @{ A = $bestA; B = $bestB; Dist = $bestDist }
}
