param(
    [Parameter(Mandatory=$true)]
    [string]$Log,
    [int]$Top = 12,
    [int]$Frames = 14,
    [string]$HLDSDir = "D:\hlds",
    [string]$Cdb = "D:\Windows Kits\10\Debuggers\x86\cdb.exe",
    [string]$SymbolPath = "cache*C:\ProgramData\dbg\sym;C:\ProgramData\dbg\sym"
)

$ErrorActionPreference = "Stop"
if (-not (Test-Path $Cdb)) { throw "cdb.exe not found at $Cdb" }

$lines = Select-String -Path $Log -Pattern "CrossBot HITCH STACK stalled (\d+)ms (inside crossbot StartFrame|outside crossbot): (.*)$" |
    ForEach-Object { [pscustomobject]@{ Ms = [int]$_.Matches[0].Groups[1].Value; Where = $_.Matches[0].Groups[2].Value; Frames = ($_.Matches[0].Groups[3].Value.Trim() -split '\s+') | Select-Object -First $Frames } }
if (-not $lines) { Write-Host "resolve_stall_stacks: no HITCH STACK lines in $Log"; return }

$byModule = @{}
foreach ($l in $lines) {
    foreach ($f in $l.Frames) {
        if ($f -match '^(.+)\+0x([0-9a-fA-F]+)$') {
            $m = $matches[1]
            if (-not $byModule.ContainsKey($m)) { $byModule[$m] = New-Object System.Collections.Generic.HashSet[string] }
            [void]$byModule[$m].Add($matches[2])
        }
    }
}

$names = @{}
foreach ($m in $byModule.Keys) {
    $path = Get-ChildItem -Path $HLDSDir -Recurse -Filter $m -File -ErrorAction SilentlyContinue | Select-Object -First 1
    if (-not $path) { $path = Get-Item (Join-Path "$env:windir\SysWOW64" $m) -ErrorAction SilentlyContinue }
    if (-not $path) { continue }
    if ($path.LinkTarget) { $path = Get-Item ([System.IO.Path]::GetFullPath($path.LinkTarget, $path.DirectoryName)) }
    $base = [System.IO.Path]::GetFileNameWithoutExtension($m)
    $offs = @($byModule[$m])
    $cmds = ($offs | ForEach-Object { ".echo @@$_;ln $base+0x$_" }) -join ";"
    $cur = $null
    & $Cdb -y "$SymbolPath;$($path.DirectoryName)" -z $path.FullName -c ".symopt+0x40;$cmds;q" 2>&1 | ForEach-Object {
        $line = "$_"
        if ($line -match '^@@([0-9a-fA-F]+)$') { $cur = $matches[1] }
        elseif ($cur -and $line -match '^\(') {
            $names["$m+0x$cur"] = (($line -split '\|')[0] -replace '^\([0-9a-f]+\)\s+', '').Trim()
            $cur = $null
        }
    }
}

$groups = $lines | Group-Object { $_.Where + "|" + (($_.Frames | Select-Object -First 4) -join ' ') } |
    Sort-Object Count -Descending | Select-Object -First $Top
foreach ($g in $groups) {
    $first = $g.Group[0]
    $maxMs = ($g.Group | Measure-Object Ms -Maximum).Maximum
    Write-Host ("=== {0} samples, {1}, up to {2}ms" -f $g.Count, $first.Where, $maxMs)
    foreach ($f in $first.Frames) {
        $n = $names[$f]
        Write-Host ("    {0,-28} {1}" -f $f, $(if ($n) { $n } else { "?" }))
    }
}
