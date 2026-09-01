<#
.SYNOPSIS
    Reads an AMD uProf session as a 1ms timeline and reports stalls.

.DESCRIPTION
    AMDuProfCLI's own `report` has no time dimension, so it cannot answer "which frame was slow".
    The translated database can: `UnifiedSampleSeries.windowId` is a 1ms bucket index, and the
    dedicated server's game loop is one thread, so one row per windowId is a per-millisecond trace
    of what that thread was doing.

    Two different stalls fall out of that, and they need opposite fixes:

      RUNS  - consecutive milliseconds inside one module. The thread was on-CPU and busy. A long
              run is compute; the module and RVA say where.
      GAPS  - milliseconds with no sample at all. The thread was off-CPU: blocked on I/O, a lock,
              a page fault. No amount of optimising the surrounding code helps.

    See doc/testing.md for what uProf can and cannot do here, and why crossbot_perf_spike is still
    the tool for attributing a frame.

.PARAMETER Session
    uProf session directory (the -o argument of `AMDuProfCLI collect`, which profile_hlds.ps1
    writes to nav_debug/perf/uprof_<map>_<stamp>). Defaults to the newest one under nav_debug/perf.

.PARAMETER MinRunMs
    Report on-CPU runs at least this long. Default 20.

.PARAMETER MinGapMs
    Report off-CPU gaps at least this long. Default 4.

.PARAMETER Top
    Rows per table. Default 15.
#>
param(
    [string]$Session = "",
    [int]$MinRunMs = 20,
    [int]$MinGapMs = 4,
    [int]$Top = 15,
    [string]$UProf = "D:\Program Files\AMD\AMDuProf\bin\AMDuProfCLI.exe",
    [string]$DuckDB = "D:\Program Files\AMD\AMDuProf\bin\duckdb.exe"
)

$ErrorActionPreference = "Stop"
$repoRoot = Split-Path $PSScriptRoot -Parent

if (-not (Test-Path $DuckDB)) {
    throw "duckdb.exe not found at $DuckDB. Download the Windows CLI from duckdb.org and drop it next to AMDuProfCLI.exe, or pass -DuckDB."
}

if (-not $Session) {
    $perfDir = Join-Path $repoRoot "nav_debug\perf"
    $newest = Get-ChildItem -Path $perfDir -Directory -Filter "uprof_*" -ErrorAction SilentlyContinue |
        Sort-Object Name | Select-Object -Last 1
    if (-not $newest) { throw "no uprof_* session under $perfDir; run profile_hlds.ps1 without -NoProfiler first" }
    $Session = $newest.FullName
}
if (-not (Test-Path $Session)) { throw "no such session directory: $Session" }

$db = Join-Path $Session "cpu.db"
if (-not (Test-Path $db)) {
    Write-Host "uprof_timeline: translating $Session"
    & $UProf translate -i $Session | Out-Null
    if (-not (Test-Path $db)) { throw "translate produced no cpu.db in $Session" }
}

# A collect that caught nothing looks identical to a quiet server. Say which it was.
$prd = Get-ChildItem -Path (Join-Path $Session "cpu") -Filter "*.prd" -ErrorAction SilentlyContinue |
    Select-Object -First 1
if ($prd -and $prd.Length -lt 100KB) {
    Write-Warning ("raw sample file is only {0} bytes -- the target probably died before the collect finished. Check the server log for 'FATAL ERROR'." -f $prd.Length)
}

Write-Host "uprof_timeline: $Session`n"

$sql = @"
-- MSVC decorated names carry the whole parameter list and run to 400+ chars, which makes every
-- table unreadable. Keep the qualified name and drop the signature.
create or replace temp view gt as
select u.windowId w,
       list_extract(string_split(m.path,'\'),-1) module,
       case when f.name is null then '?'
            else substr(regexp_replace(regexp_replace(f.name, '\(.*', ''), '^.*[ >]', ''), 1, 58)
       end fn,
       cast(u.moduleRVA as bigint) rva
from UnifiedSampleSeries u
left join Module m on m.id = u.moduleId
left join Function f on f.id = u.functionId
where u.threadId = (select threadId from UnifiedSampleSeries
                    group by threadId order by count(*) desc limit 1);

create or replace temp view perms as
select distinct on (w) w, module, fn, rva from gt order by w;

.print === busiest thread: where the milliseconds went
select module, count(*) ms, round(100.0*count(*)/sum(count(*)) over (),1) pct
from gt group by 1 order by ms desc limit $Top;

.print
.print === on-CPU runs (consecutive ms in one module; compute stalls)
select module, start_ms, run_ms from (
  select module, min(w) start_ms, count(*) run_ms
  from (select *, row_number() over (order by w)
               - row_number() over (partition by module order by w) g from perms)
  group by module, g
) where run_ms >= $MinRunMs order by run_ms desc limit $Top;

.print
.print === off-CPU gaps (no sample at all; blocked, not slow)
select w gap_start_ms, next_w - w gap_ms, module module_before, fn fn_before
from (select w, module, fn, lead(w) over (order by w) next_w from perms)
where next_w - w >= $MinGapMs order by gap_ms desc limit $Top;

.print
.print === hottest code (swds.dll has no symbols, so it reports as module+RVA)
select module, case when fn = '?' then printf('rva 0x%06X', rva // 4096 * 4096) else fn end site,
       count(*) ms, round(100.0*count(*)/sum(count(*)) over (),1) pct
from gt group by 1,2 order by ms desc limit $Top;
"@

$sqlFile = Join-Path ([IO.Path]::GetTempPath()) ("uprof_timeline_{0}.sql" -f [Guid]::NewGuid())
Set-Content -Path $sqlFile -Value $sql -Encoding UTF8
try {
    & $DuckDB -readonly $db -init $sqlFile -c ".quit"
} finally {
    Remove-Item $sqlFile -ErrorAction SilentlyContinue
}
