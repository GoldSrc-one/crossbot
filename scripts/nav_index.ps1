function Update-IndexHtml {
    param(
        [string]$MapName,
        [string]$Result,
        [string]$OutputDir
    )

    $indexHtml = Join-Path $OutputDir "index.html"
    if (-not (Test-Path $indexHtml)) {
        Write-Host "Index.html not found at $indexHtml, skipping update"
        return
    }

    $html = Get-Content $indexHtml -Raw
    $color = if ($Result -eq "PASS") { "#1a5c1a" } else { "#5c1a1a" }
    $borderColor = if ($Result -eq "PASS") { "#2e8b2e" } else { "#8b2e2e" }
    $label = if ($Result -eq "PASS") { "✓ PASS" } else { "✗ FAIL" }

    # Match either a virgin card from generate_all_svgs.ps1 or one this function
    # already rewrote, so repeat runs replace the mark instead of stacking cards.
    $esc = [regex]::Escape($MapName)
    $pattern = '<div class="card"[^>]*><h2><a href="' + $esc + '\.svg" target="_blank">' + $esc

    $newCard = '<div class="card" style="background: {0}; border-color: {1};"><h2><a href="{2}.svg" target="_blank">{2} <span style="float:right; color:{3};">{4}</span></a></h2><a href="{2}.svg" target="_blank"><img src="{2}.svg"></a></div>' -f $color, $borderColor, $MapName, $borderColor, $label

    if ($html -match $pattern) {
        $regex = [regex]::new($pattern + '.*?</div>', 'Singleline')
        $html = $regex.Replace($html, $newCard, 1)
    } else {
        $html = $html -replace '(<div class="grid">)', "`$1`n$newCard"
    }

    $html | Set-Content -Path $indexHtml -Encoding UTF8
    Write-Host "Updated index.html: $MapName -> $label"
}
