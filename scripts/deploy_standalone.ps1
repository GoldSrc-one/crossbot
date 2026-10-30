param(
    [Parameter(Mandatory=$true)]
    [string]$Game,
    [string]$Map = "",
    [string]$HLDSDir = "D:\hlds"
)

$repo = Split-Path $PSScriptRoot -Parent
$dll = Join-Path $repo "build-standalone\crossbot_mm.dll"
$gamedata = Join-Path $HLDSDir "crossplay\gamedata\$Game-win.json"
$addon = Join-Path $HLDSDir "$Game\addons\crossbot"

if (-not (Test-Path $dll)) { throw "$dll not found, run: build.bat standalone" }
if (-not (Test-Path $gamedata)) { throw "no gamedata at $gamedata" }

foreach ($dir in @($addon, "$addon\gamedata", "$addon\nav", "$addon\trace")) {
    New-Item -ItemType Directory -Path $dir -Force | Out-Null
}

Copy-Item $dll $addon -Force
Copy-Item (Join-Path $repo "names.txt"), (Join-Path $repo "places.txt") $addon -Force
Copy-Item $gamedata "$addon\gamedata" -Force
if (-not (Test-Path "$addon\brain.json")) {
    Copy-Item (Join-Path $repo "brain.example.json") "$addon\brain.json"
}
"win32 addons/crossbot/crossbot_mm.dll" | Set-Content "$addon\plugins.ini"

$launchMap = if ($Map) { $Map } else { "<map>" }
if ($Map) {
    $nav = Join-Path $HLDSDir "crossplay\addons\crossbot\nav\$Map.nav"
    if (Test-Path $nav) {
        Copy-Item $nav "$addon\nav" -Force
    } else {
        Write-Warning "no $nav, generate $addon\nav\$Map.nav with navgen"
    }
}

"deployed to $addon"
"$HLDSDir\hlds.exe -console -game $Game -dll addons\metamod\metamod.dll +localinfo mm_pluginsfile addons/crossbot/plugins.ini +map $launchMap"
