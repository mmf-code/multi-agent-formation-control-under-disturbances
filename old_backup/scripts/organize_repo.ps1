param(
  [switch]$WhatIf
)

Write-Host "Organizing repository outputs..."
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$repo = Resolve-Path (Join-Path $root "..")
Set-Location $repo

function Move-Safe {
  param([string]$Path,[string]$Dest)
  if (Test-Path $Path) {
    New-Item -ItemType Directory -Force -Path (Split-Path $Dest) | Out-Null
    if ($WhatIf) { Write-Host "Would move $Path -> $Dest" }
    else { Move-Item -Force -Path $Path -Destination $Dest }
  }
}

# Consolidate legacy into docs/old
$docsOld = Join-Path $repo "docs/old"
New-Item -ItemType Directory -Force -Path $docsOld | Out-Null

# 1) Legacy output directories -> docs/old
Move-Safe -Path (Join-Path $repo "simulation_outputs") -Dest (Join-Path $docsOld "simulation_outputs")
Move-Safe -Path (Join-Path $repo "results") -Dest (Join-Path $docsOld "results")
Move-Safe -Path (Join-Path $repo "final_project_results") -Dest (Join-Path $docsOld "final_project_results")

# Merge any previous top-level 'old' into docs/old, then remove it
if (Test-Path (Join-Path $repo "old")) {
  Get-ChildItem -Force (Join-Path $repo "old") | ForEach-Object {
    $dest = Join-Path $docsOld $_.Name
    if ($WhatIf) { Write-Host "Would move $($_.FullName) -> $dest" }
    else { Move-Item -Force -Path $_.FullName -Destination $dest }
  }
  if (-not $WhatIf) { Remove-Item -Force -Recurse (Join-Path $repo "old") }
}

# 2) Images in repo (excluding docs/, outputs/, .git) -> docs/old/legacy_images
$imgs = Get-ChildItem -File -Path $repo -Include *.png,*.jpg,*.jpeg,*.gif,*.bmp,*.svg -Recurse |
  Where-Object { $_.FullName -notmatch "\\docs\\" -and $_.FullName -notmatch "\\outputs\\" -and $_.FullName -notmatch "\\.git\\" }

if ($imgs) {
  $imgDest = Join-Path $docsOld "legacy_images"
  New-Item -ItemType Directory -Force -Path $imgDest | Out-Null
  foreach ($f in $imgs) {
    $rel = $f.FullName.Substring($repo.Path.Length).TrimStart('\\','/')
    $to = Join-Path $imgDest $rel.Replace(':','_')
    New-Item -ItemType Directory -Force -Path (Split-Path $to) | Out-Null
    if ($WhatIf) { Write-Host "Would move $($f.FullName) -> $to" }
    else { Move-Item -Force -Path $f.FullName -Destination $to }
  }
}

# 3) Root-level CSV/TXT artifacts -> docs/old/legacy_root_files
$rootFiles = Get-ChildItem -File -Path $repo -Include *.csv,*.txt |
  Where-Object { $_.DirectoryName -eq $repo.Path }
if ($rootFiles) {
  $dest = Join-Path $docsOld "legacy_root_files"
  New-Item -ItemType Directory -Force -Path $dest | Out-Null
  foreach ($f in $rootFiles) {
    $to = Join-Path $dest $f.Name
    if ($WhatIf) { Write-Host "Would move $($f.FullName) -> $to" }
    else { Move-Item -Force -Path $f.FullName -Destination $to }
  }
}

# 4) PowerShell scripts at repo root (except scripts/) -> docs/old/powershell
$ps1Root = Get-ChildItem -File -Path $repo -Filter *.ps1 |
  Where-Object { $_.DirectoryName -eq $repo.Path }
if ($ps1Root) {
  $psDest = Join-Path $docsOld "powershell"
  New-Item -ItemType Directory -Force -Path $psDest | Out-Null
  foreach ($f in $ps1Root) {
    $to = Join-Path $psDest $f.Name
    if ($WhatIf) { Write-Host "Would move $($f.FullName) -> $to" }
    else { Move-Item -Force -Path $f.FullName -Destination $to }
  }
}

Write-Host "Done."

