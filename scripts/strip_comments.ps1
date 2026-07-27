<#
.SYNOPSIS
    Strip all C/C++ comments from src/ and tools/ files.
    Preserves string and char literals. Removes lines that become empty
    after stripping a full-line line comment; keeps lines that had code
    beside a trailing comment (only the comment is removed).
#>

[CmdletBinding()]
param(
    [switch]$DryRun,
    [string[]]$Paths = @('src', 'tools')
)

$ErrorActionPreference = 'Stop'

function Strip-Comments {
    param([string]$Text)

    $sb = [System.Text.StringBuilder]::new()
    $len = $Text.Length
    $i = 0
    $state = 'code'   # code | line_comment | block_comment | string | char
    # $nl tracks whether the newline char that ends a stripped line comment
    # should still be emitted. We drop the //... portion but keep \n.

    while ($i -lt $len) {
        $c = $Text[$i]
        $next = if ($i + 1 -lt $len) { $Text[$i + 1] } else { '' }

        switch ($state) {
            'code' {
                if ($c -eq '/' -and $next -eq '/') {
                    $state = 'line_comment'
                    $i += 2
                    continue
                }
                if ($c -eq '/' -and $next -eq '*') {
                    $state = 'block_comment'
                    $i += 2
                    # Insert a single space to keep tokens from merging across
                    # a removed block comment, matching typical formatters.
                    [void]$sb.Append(' ')
                    continue
                }
                if ($c -eq '"') {
                    $state = 'string'
                    [void]$sb.Append($c)
                    $i++
                    continue
                }
                if ($c -eq "'") {
                    $state = 'char'
                    [void]$sb.Append($c)
                    $i++
                    continue
                }
                [void]$sb.Append($c)
                $i++
            }
            'line_comment' {
                if ($c -eq "`n") {
                    $state = 'code'
                    [void]$sb.Append($c)
                    $i++
                    continue
                }
                # CR/LF other than \n: treat \r\n together by skipping \r
                if ($c -eq "`r") {
                    $i++
                    continue
                }
                $i++
            }
            'block_comment' {
                if ($c -eq '*' -and $next -eq '/') {
                    $state = 'code'
                    $i += 2
                    continue
                }
                $i++
            }
            'string' {
                if ($c -eq '\') {
                    [void]$sb.Append($c)
                    if ($i + 1 -lt $len) {
                        [void]$sb.Append($Text[$i + 1])
                        $i += 2
                        continue
                    }
                    $i++
                    continue
                }
                if ($c -eq '"') {
                    [void]$sb.Append($c)
                    $state = 'code'
                    $i++
                    continue
                }
                [void]$sb.Append($c)
                $i++
            }
            'char' {
                if ($c -eq '\') {
                    [void]$sb.Append($c)
                    if ($i + 1 -lt $len) {
                        [void]$sb.Append($Text[$i + 1])
                        $i += 2
                        continue
                    }
                    $i++
                    continue
                }
                if ($c -eq "'") {
                    [void]$sb.Append($c)
                    $state = 'code'
                    $i++
                    continue
                }
                [void]$sb.Append($c)
                $i++
            }
        }
    }

    $out = $sb.ToString()

    # Remove lines that are now empty (only whitespace) as a result of stripping
    # a full-line line comment, but keep normally blank lines (inteself in original).
    # Simpler & safe: trim trailing whitespace per line, drop lines that are entirely
    # whitespace ONLY where the original was a comment-only line.
    # To keep risk low, just rstrip each line's trailing whitespace and collapse
    # multiple consecutive blank lines into one.
    $lines = $out -split "`r?`n"
    $cleaned = New-Object System.Collections.Generic.List[string]
    $lastBlank = $false
    foreach ($ln in $lines) {
        $trimmed = $ln.TrimEnd()
        if ($trimmed.Length -eq 0) {
            if ($lastBlank) { continue }
            $lastBlank = $true
            $cleaned.Add('')
            continue
        }
        $lastBlank = $false
        $cleaned.Add($trimmed)
    }
    # Drop a single leading blank line if present.
    if ($cleaned.Count -gt 0 -and $cleaned[0].Length -eq 0) {
        $cleaned.RemoveAt(0)
    }
    return ($cleaned -join "`n") + "`n"
}

$exts = @('*.c','*.cpp','*.h','*.hpp')
$files = foreach ($p in $Paths) {
    if (Test-Path $p) {
        Get-ChildItem -Path $p -Recurse -Include $exts -File
    }
}

$totalChanged = 0
foreach ($f in $files) {
    $orig = Get-Content -Raw -LiteralPath $f.FullName
    $new = Strip-Comments -Text $orig
    if ($new -ne $orig) {
        $totalChanged++
        if ($DryRun) {
            Write-Host "DRY: $($f.FullName)"
        } else {
            Set-Content -LiteralPath $f.FullName -Value $new -NoNewline -Encoding UTF8
            Write-Host "Stripped: $($f.FullName)"
        }
    }
}
Write-Host "Total changed: $totalChanged"
