param(
	[Parameter(Mandatory = $true)][string]$Out,
	[string]$Maps = 'maps',
	[string]$NavGen = 'build\tools\navgen\navgen.exe'
)

if (-not (Test-Path $NavGen))
{
	throw "navgen not found at $NavGen - build first with: & .\build.bat"
}

$lines = @()

foreach ($bsp in (Get-ChildItem "$Maps\*.bsp" | Sort-Object Name))
{
	$txt = & $NavGen $bsp.FullName 2>&1 | Out-String

	$areas = [regex]::Match($txt, '-> (\d+) areas').Groups
	$leaves = [regex]::Match($txt, '(\d+) hull-3 empty leaves').Groups
	$conns = [regex]::Match($txt, '(\d+) walk connections, (\d+) air move').Groups
	$gate = [regex]::Match($txt, '(\d+) connections have no gate').Groups
	$ill = [regex]::Match($txt, '(\d+) areas have an ill-formed').Groups
	$route = [regex]::Matches($txt, 'no route').Count

	$g = if ($gate[1].Success) { $gate[1].Value } else { '0' }
	$i = if ($ill[1].Success) { $ill[1].Value } else { '0' }

	$lines += "{0}: leaves={1} areas={2} walk={3} air={4} nogate={5} illformed={6} noroute={7}" -f `
		$bsp.Name, $leaves[1].Value, $areas[1].Value, $conns[1].Value, $conns[2].Value, $g, $i, $route
}

$lines | Set-Content $Out
Write-Host "swept $($lines.Count) maps -> $Out"
