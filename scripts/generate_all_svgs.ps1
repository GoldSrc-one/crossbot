
$ErrorActionPreference = "Stop"

$scriptDir = $PSScriptRoot
$projectRoot = Split-Path $scriptDir -Parent
$navgen = Join-Path $projectRoot "build\tools\navgen\navgen.exe"
$mapsDir = Join-Path $projectRoot "maps"
$outputDir = Join-Path $projectRoot "nav_debug"
$indexHtml = Join-Path $outputDir "index.html"

if (-not (Test-Path $navgen)) {
    Write-Error "navgen.exe not found at $navgen. Run .\build.bat first."
    exit 1
}

if (-not (Test-Path $mapsDir)) {
    Write-Error "Maps directory not found at $mapsDir"
    exit 1
}

if (-not (Test-Path $outputDir)) {
    New-Item -ItemType Directory -Path $outputDir | Out-Null
}

$maps = Get-ChildItem -Path $mapsDir -Filter "*.bsp" | Sort-Object Name

Write-Host "Found $($maps.Count) maps in $mapsDir"
Write-Host "Output directory: $outputDir"
Write-Host ""

$success = 0
$failed = 0
$processedMaps = @()

foreach ($map in $maps) {
    $mapName = $map.BaseName
    $svgPath = Join-Path $outputDir "$mapName.svg"
    $navPath = Join-Path $outputDir "$mapName.nav"

    Write-Host "Processing: $mapName ..." -NoNewline

    try {
        $result = & $navgen $map.FullName --dump-svg $svgPath -o $navPath 2>&1
        if ($LASTEXITCODE -eq 0) {
            Write-Host " OK" -ForegroundColor Green
            $success++
            $processedMaps += $mapName
        } else {
            Write-Host " FAILED" -ForegroundColor Red
            Write-Host $result
            $failed++
        }
    } catch {
        Write-Host " FAILED" -ForegroundColor Red
        Write-Host $_.Exception.Message
        $failed++
    }
}

Write-Host ""
Write-Host "Generating index.html..." -NoNewline
$cardLines = @()
foreach ($mapName in $processedMaps) {
    $cardLines += '<div class="card"><h2><a href="{0}.svg" target="_blank">{0}</a></h2><a href="{0}.svg" target="_blank"><img src="{0}.svg"></a></div>' -f $mapName
}

$html = @"
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>CrossBot nav-mesh debug gallery</title>
<style>
  body { background: #05070c; color: #e8eef4; font-family: monospace; margin: 0; padding: 16px; }
  h1 { font-size: 18px; margin: 0 0 12px 0; }
  .grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(360px, 1fr)); gap: 14px; }
  .card { background: #0b1220; border: 1px solid #2c3e50; border-radius: 6px; padding: 8px; }
  .card h2 { font-size: 13px; margin: 0 0 6px 0; }
  .card a { color: #9fd3ff; text-decoration: none; }
  .card img { width: 100%; height: auto; display: block; border-radius: 4px; background: #0b1220; }
</style>
</head>
<body>
<h1>CrossBot nav-mesh debug gallery (top-down, Z ignored)</h1>
<div class="grid">
$($cardLines -join "`n")
</div>
</body>
</html>
"@

$html | Set-Content -Path $indexHtml -Encoding UTF8
Write-Host " done" -ForegroundColor Green

Write-Host ""
Write-Host "=== Summary ==="
Write-Host "Success: $success"
Write-Host "Failed:  $failed"
Write-Host "SVGs written to: $outputDir"
Write-Host "Index updated at: $indexHtml"

if ($failed -gt 0) { exit 1 }
