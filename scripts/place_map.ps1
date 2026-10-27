param(
    [Parameter(Mandatory = $true)][string]$Map,
    [string]$HLDSDir = "D:\hlds",
    [string]$OutDir = "",
    [int]$Grid = 256,
    [switch]$Czero,
    [switch]$Items,
    [switch]$Rebuild,
    [string]$Crop = "",
    [int]$Pixels = 1500
)

$ErrorActionPreference = "Stop"
$repoRoot = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
if (-not $OutDir) { $OutDir = Join-Path $repoRoot "nav_debug\places" }
New-Item -ItemType Directory -Path $OutDir -Force | Out-Null

$bsp = $null
foreach ($game in @("cstrike", "czero", "dod", "tfc", "valve", "gearbox", "dmc")) {
    $p = Join-Path $HLDSDir "$game\maps\$Map.bsp"
    if (Test-Path $p) { $bsp = $p; break }
}
if (-not $bsp) { throw "no $Map.bsp under $HLDSDir" }

$obj = Join-Path $OutDir "$Map.obj"
if ($Rebuild -or -not (Test-Path $obj)) {
    $navgen = Join-Path $repoRoot "build-x64\tools\navgen\navgen.exe"
    & $navgen $bsp -o (Join-Path $OutDir "$Map.nav") --dump-obj $obj | Out-Null
    if (-not (Test-Path $obj)) { throw "navgen wrote no $obj" }
}

$inv = [Globalization.CultureInfo]::InvariantCulture
function F([double]$v) { $v.ToString("0.#", $inv) }

$verts = New-Object System.Collections.Generic.List[double[]]
$faces = New-Object System.Collections.Generic.List[object]
foreach ($line in [IO.File]::ReadLines($obj)) {
    if ($line.StartsWith("v ")) {
        $p = $line.Split(" ")
        $verts.Add(@([double]::Parse($p[1], $inv), [double]::Parse($p[2], $inv), [double]::Parse($p[3], $inv)))
    } elseif ($line.StartsWith("f ")) {
        $faces.Add(@($line.Substring(2).Split(" ", [StringSplitOptions]::RemoveEmptyEntries) | ForEach-Object { [int]($_.Split("/")[0]) - 1 }))
    }
}

$bytes = [IO.File]::ReadAllBytes($bsp)
$entOff = [BitConverter]::ToInt32($bytes, 4); $entLen = [BitConverter]::ToInt32($bytes, 8)
$modOff = [BitConverter]::ToInt32($bytes, 4 + 14 * 8)
$entText = [Text.Encoding]::ASCII.GetString($bytes, $entOff, $entLen)
$ents = @()
foreach ($m in [regex]::Matches($entText, "\{([^}]*)\}")) {
    $e = @{}
    foreach ($kv in [regex]::Matches($m.Groups[1].Value, '"([^"]*)"\s*"([^"]*)"')) { $e[$kv.Groups[1].Value] = $kv.Groups[2].Value }
    $pos = $null
    if ($e.origin) {
        $o = $e.origin.Split(" ", [StringSplitOptions]::RemoveEmptyEntries)
        $pos = @([double]::Parse($o[0], $inv), [double]::Parse($o[1], $inv), [double]::Parse($o[2], $inv))
    } elseif ($e.model -and $e.model.StartsWith("*")) {
        $b = $modOff + 64 * [int]$e.model.Substring(1)
        $pos = @(0..2 | ForEach-Object { ([BitConverter]::ToSingle($bytes, $b + 4 * $_) + [BitConverter]::ToSingle($bytes, $b + 12 + 4 * $_)) / 2 })
    }
    if ($pos) { $e["_pos"] = $pos; $ents += $e }
}

$keep = '^(info_player_|info_vip_start|func_bomb_target|info_bomb_target|func_hostage_rescue|info_hostage_rescue|hostage_entity|func_escapezone|func_vip_safetyzone|dod_control_point|dod_capture_area|dod_object|info_tfgoal|item_tfgoal|func_button|trigger_teleport|func_tank|func_ladder|func_plat|func_train|func_door$)'
if ($Items) { $keep = $keep.TrimEnd(')') + '|weapon_|item_|ammo_)' }
$marks = @()
foreach ($e in $ents) {
    if ($e.classname -notmatch $keep) { continue }
    $name = @($e.point_name, $e.area_name, $e.netname, $e.targetname, $e.team_no) | Where-Object { $_ } | Select-Object -First 1
    $label = ($e.classname -replace '^(info_player_|info_|func_|item_|weapon_)', '') + $(if ($name) { ":$name" } else { "" })
    $marks += @{ pos = $e._pos; label = $label }
}

$places = @()
$placesFile = Join-Path $repoRoot "places.txt"
foreach ($line in Get-Content $placesFile) {
    $p = $line.Split(" ", 5)
    if ($p.Count -eq 5 -and $p[0] -eq $Map) {
        $places += @{ pos = @([double]::Parse($p[1], $inv), [double]::Parse($p[2], $inv), [double]::Parse($p[3], $inv)); label = $p[4] }
    }
}

$cz = @()
$czFile = Join-Path $repoRoot "graveyard\czero_places\$Map.txt"
if ($Czero -and (Test-Path $czFile)) {
    foreach ($line in Get-Content $czFile) {
        $p = $line.Split(" ", 4)
        $cz += @{ pos = @([double]::Parse($p[0], $inv), [double]::Parse($p[1], $inv), [double]::Parse($p[2], $inv)); label = $p[3] }
    }
}

$xs = $verts | ForEach-Object { $_[0] }; $ys = $verts | ForEach-Object { $_[1] }; $zs = $verts | ForEach-Object { $_[2] }
$minX = ($xs | Measure-Object -Minimum).Minimum - 128; $maxX = ($xs | Measure-Object -Maximum).Maximum + 128
$minY = ($ys | Measure-Object -Minimum).Minimum - 128; $maxY = ($ys | Measure-Object -Maximum).Maximum + 128
$minZ = ($zs | Measure-Object -Minimum).Minimum; $maxZ = ($zs | Measure-Object -Maximum).Maximum
if ($Crop) {
    $c = $Crop.Split(" ", [StringSplitOptions]::RemoveEmptyEntries) | ForEach-Object { [double]$_ }
    $minX = $c[0]; $minY = $c[1]; $maxX = $c[2]; $maxY = $c[3]
}
$w = $maxX - $minX; $h = $maxY - $minY
$scale = $Pixels / [Math]::Max($w, $h)
$W = [int]($w * $scale); $H = [int]($h * $scale) + 40
function SX([double]$x) { [float](($x - $minX) * $scale) }
function SY([double]$y) { [float](($maxY - $y) * $scale) }

Add-Type -AssemblyName System.Drawing
function Hue([double]$hue, [double]$s, [double]$v) {
    $hue = ($hue % 360 + 360) % 360; $c = $v * $s; $x = $c * (1 - [Math]::Abs((($hue / 60) % 2) - 1)); $m = $v - $c
    $rgb = switch ([int][Math]::Floor($hue / 60)) { 0 { $c, $x, 0 } 1 { $x, $c, 0 } 2 { 0, $c, $x } 3 { 0, $x, $c } 4 { $x, 0, $c } default { $c, 0, $x } }
    [Drawing.Color]::FromArgb([int](255 * ($rgb[0] + $m)), [int](255 * ($rgb[1] + $m)), [int](255 * ($rgb[2] + $m)))
}
function Ramp([double]$z) { $t = if ($maxZ -gt $minZ) { ($z - $minZ) / ($maxZ - $minZ) } else { 0.5 }; Hue (240 - 240 * $t) 0.75 0.75 }

$bmp = New-Object Drawing.Bitmap $W, $H
$g = [Drawing.Graphics]::FromImage($bmp)
$g.SmoothingMode = [Drawing.Drawing2D.SmoothingMode]::AntiAlias
$g.TextRenderingHint = [Drawing.Text.TextRenderingHint]::AntiAliasGridFit
$g.Clear([Drawing.Color]::FromArgb(17, 17, 17))

$sorted = $faces | ForEach-Object { @{ f = $_; z = ($_ | ForEach-Object { $verts[$_][2] } | Measure-Object -Average).Average } } | Sort-Object { $_.z }
foreach ($s in $sorted) {
    $pts = [Drawing.PointF[]]@($s.f | ForEach-Object { New-Object Drawing.PointF((SX $verts[$_][0]), (SY $verts[$_][1])) })
    if ($pts.Count -ge 3) { $g.FillPolygon((New-Object Drawing.SolidBrush (Ramp $s.z)), $pts) }
}

$small = New-Object Drawing.Font("Arial", 9)
$mid = New-Object Drawing.Font("Arial", 10, [Drawing.FontStyle]::Bold)
function Label([string]$text, [float]$x, [float]$y, $color, $font) {
    $black = [Drawing.Brushes]::Black
    foreach ($d in @(@(-1, 0), @(1, 0), @(0, -1), @(0, 1))) { $g.DrawString($text, $font, $black, $x + $d[0], $y + $d[1]) }
    $g.DrawString($text, $font, (New-Object Drawing.SolidBrush $color), $x, $y)
}

$step = $Grid; while ($w / $step -gt 40) { $step *= 2 }
$faint = New-Object Drawing.Pen([Drawing.Color]::FromArgb(50, 255, 255, 255), 1)
$strong = New-Object Drawing.Pen([Drawing.Color]::FromArgb(110, 255, 255, 255), 1)
for ($x = [Math]::Ceiling($minX / $step) * $step; $x -le $maxX; $x += $step) {
    $g.DrawLine($(if ($x % ($step * 4) -eq 0) { $strong } else { $faint }), (SX $x), 0, (SX $x), (SY $minY))
    if ($x % ($step * 2) -eq 0) { Label "$x" ((SX $x) + 2) 2 ([Drawing.Color]::White) $small }
}
for ($y = [Math]::Ceiling($minY / $step) * $step; $y -le $maxY; $y += $step) {
    $g.DrawLine($(if ($y % ($step * 4) -eq 0) { $strong } else { $faint }), 0, (SY $y), (SX $maxX), (SY $y))
    if ($y % ($step * 2) -eq 0) { Label "$y" 2 ((SY $y) - 14) ([Drawing.Color]::White) $small }
}

$czNames = @($cz | ForEach-Object { $_.label } | Sort-Object -Unique)
foreach ($c in $cz) {
    $col = Hue (360 * [Array]::IndexOf($czNames, $c.label) / [Math]::Max(1, $czNames.Count)) 0.6 1
    $g.FillEllipse((New-Object Drawing.SolidBrush $col), (SX $c.pos[0]) - 2, (SY $c.pos[1]) - 2, 4, 4)
}
foreach ($name in $czNames) {
    $pts = @($cz | Where-Object { $_.label -eq $name })
    $mx = ($pts | ForEach-Object { $_.pos[0] } | Measure-Object -Average).Average
    $my = ($pts | ForEach-Object { $_.pos[1] } | Measure-Object -Average).Average
    Label "($name)" (SX $mx) (SY $my) (Hue (360 * [Array]::IndexOf($czNames, $name) / [Math]::Max(1, $czNames.Count)) 0.6 1) $small
}
foreach ($m in $marks) {
    $g.FillRectangle([Drawing.Brushes]::White, (SX $m.pos[0]) - 3, (SY $m.pos[1]) - 3, 6, 6)
    Label "$($m.label) z$([int]$m.pos[2])" ((SX $m.pos[0]) + 4) ((SY $m.pos[1]) - 6) ([Drawing.Color]::LightGray) $small
}
$placeNames = @($places | ForEach-Object { $_.label } | Sort-Object -Unique)
foreach ($p in $places) {
    $col = Hue (360 * [Array]::IndexOf($placeNames, $p.label) / [Math]::Max(1, $placeNames.Count)) 0.55 1
    $g.FillEllipse((New-Object Drawing.SolidBrush $col), (SX $p.pos[0]) - 5, (SY $p.pos[1]) - 5, 10, 10)
    $g.DrawEllipse([Drawing.Pens]::Black, (SX $p.pos[0]) - 5, (SY $p.pos[1]) - 5, 10, 10)
}
foreach ($name in $placeNames) {
    $pts = @($places | Where-Object { $_.label -eq $name })
    $mx = ($pts | ForEach-Object { $_.pos[0] } | Measure-Object -Average).Average
    $my = ($pts | ForEach-Object { $_.pos[1] } | Measure-Object -Average).Average
    $c = $pts | Sort-Object { ($_.pos[0] - $mx) * ($_.pos[0] - $mx) + ($_.pos[1] - $my) * ($_.pos[1] - $my) } | Select-Object -First 1
    Label "$name z$([int]$c.pos[2])" ((SX $c.pos[0]) + 6) ((SY $c.pos[1]) - 7) (Hue (360 * [Array]::IndexOf($placeNames, $name) / [Math]::Max(1, $placeNames.Count)) 0.55 1) $mid
}
for ($i = 0; $i -le 10; $i++) {
    $z = $minZ + ($maxZ - $minZ) * $i / 10
    $g.FillRectangle((New-Object Drawing.SolidBrush (Ramp $z)), [float]($i * $W / 11), [float]($H - 36), [float]($W / 11), 34)
    Label "z $([int]$z)" ([float]($i * $W / 11 + 3)) ([float]($H - 28)) ([Drawing.Color]::White) $small
}
$png = Join-Path $OutDir "$Map$(if ($Crop) { '_crop' }).png"
$bmp.Save($png, [Drawing.Imaging.ImageFormat]::Png)
$g.Dispose(); $bmp.Dispose()
"wrote $png ($($faces.Count) faces, $($marks.Count) entities, $($places.Count) place points, $($cz.Count) czero points)"
