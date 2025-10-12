# Multi-Agent Formation Control - Enhanced Automated Testing Suite
# No external dependencies - uses robust text processing for YAML modification
# Author: Generated for thesis automation

param(
    [switch]$RunAll,
    [string]$TestSuite = "system_comparison",
    [int]$TestNumber = 0,
    [switch]$Help,
    [switch]$DryRun,
    [string]$OutputDir = ""
)

Write-Host "Multi-Agent Formation Control - Enhanced Testing Suite" -ForegroundColor Cyan
Write-Host "Ready to run simulations!" -ForegroundColor Green

# Configuration
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$ConfigPath = Join-Path $ScriptDir "agent_control_pkg\config\simulation_params.yaml"
$ExePath = Join-Path $ScriptDir "agent_control_pkg\build\Release\multi_drone_pid_tester.exe"
$SimOutputDir = Join-Path $ScriptDir "agent_control_pkg\build\Release\simulation_outputs"

if ($OutputDir -eq "") {
    $OutputDir = Join-Path $ScriptDir "final_project_results\automated_testing_$(Get-Date -Format 'yyyyMMdd_HHmmss')"
}

# Test Configurations
$SystemComparisonConfigs = @{
    1 = @{
        Name = "Pure_PID"
        Description = "Baseline PID controller without additional features"
        PID = @{ Kp = 3.1; Ki = 0.4; Kd = 2.2 }
        FLS = $false
        Wind = $false
        FeedForward = $false
    }
    2 = @{
        Name = "PID_with_FLS"
        Description = "PID controller with Fuzzy Logic System"
        PID = @{ Kp = 3.1; Ki = 0.4; Kd = 2.2 }
        FLS = $true
        Wind = $false
        FeedForward = $false
    }
    3 = @{
        Name = "PID_with_Wind"
        Description = "PID controller under wind disturbances"
        PID = @{ Kp = 3.1; Ki = 0.4; Kd = 2.2 }
        FLS = $false
        Wind = $true
        FeedForward = $false
    }
    4 = @{
        Name = "FLS_with_Wind"
        Description = "PID + FLS under wind disturbances"
        PID = @{ Kp = 3.1; Ki = 0.4; Kd = 2.2 }
        FLS = $true
        Wind = $true
        FeedForward = $false
    }
    5 = @{
        Name = "Full_System"
        Description = "Complete system: PID + FLS + Wind + Feed-Forward"
        PID = @{ Kp = 3.1; Ki = 0.4; Kd = 2.2 }
        FLS = $true
        Wind = $true
        FeedForward = $true
    }
}

$ComprehensiveConfigs = @{
    1 = @{
        Name = "Conservative_PID"
        Description = "Conservative PID tuning baseline"
        PID = @{ Kp = 2.0; Ki = 0.2; Kd = 1.5 }
        FLS = $false; Wind = $false; FeedForward = $false
    }
    2 = @{
        Name = "Optimal_PID"
        Description = "Optimal PID tuning baseline"
        PID = @{ Kp = 3.1; Ki = 0.4; Kd = 2.2 }
        FLS = $false; Wind = $false; FeedForward = $false
    }
    3 = @{
        Name = "Aggressive_PID"
        Description = "Aggressive PID tuning baseline"
        PID = @{ Kp = 5.0; Ki = 0.8; Kd = 3.5 }
        FLS = $false; Wind = $false; FeedForward = $false
    }
    4 = @{
        Name = "FLS_Conservative"
        Description = "Conservative PID + FLS"
        PID = @{ Kp = 2.0; Ki = 0.2; Kd = 1.5 }
        FLS = $true; Wind = $false; FeedForward = $false
    }
    5 = @{
        Name = "FLS_Optimal"
        Description = "Optimal PID + FLS"
        PID = @{ Kp = 3.1; Ki = 0.4; Kd = 2.2 }
        FLS = $true; Wind = $false; FeedForward = $false
    }
    6 = @{
        Name = "Wind_Conservative"
        Description = "Conservative PID + Wind"
        PID = @{ Kp = 2.0; Ki = 0.2; Kd = 1.5 }
        FLS = $false; Wind = $true; FeedForward = $false
    }
    7 = @{
        Name = "Wind_Optimal"
        Description = "Optimal PID + Wind"
        PID = @{ Kp = 3.1; Ki = 0.4; Kd = 2.2 }
        FLS = $false; Wind = $true; FeedForward = $false
    }
    8 = @{
        Name = "Full_Conservative"
        Description = "Conservative PID + All Features"
        PID = @{ Kp = 2.0; Ki = 0.2; Kd = 1.5 }
        FLS = $true; Wind = $true; FeedForward = $true
    }
}

function Get-TestConfigs {
    param([string]$Suite)

    switch ($Suite.ToLower()) {
        "system_comparison" { return $SystemComparisonConfigs }
        "comprehensive" { return $ComprehensiveConfigs }
        default {
            Write-Host "Unknown test suite: $Suite" -ForegroundColor Red
            return $null
        }
    }
}

function Show-Help {
    Write-Host "Multi-Agent Formation Control - Enhanced Simulation Testing Suite" -ForegroundColor Cyan
    Write-Host "================================================================" -ForegroundColor Cyan
    Write-Host ""
    Write-Host "Usage:" -ForegroundColor Yellow
    Write-Host "  .\run_simulations_enhanced.ps1 -DryRun                        # Preview available tests"
    Write-Host "  .\run_simulations_enhanced.ps1 -TestSuite system_comparison   # Show system comparison tests"
    Write-Host "  .\run_simulations_enhanced.ps1 -TestNumber 1                  # Run specific test"
    Write-Host "  .\run_simulations_enhanced.ps1 -RunAll                        # Run all tests in suite"
    Write-Host ""
    Write-Host "Test Suites:" -ForegroundColor Green
    Write-Host "  system_comparison : Compare main system configurations (5 tests)"
    Write-Host "  comprehensive     : Extended parameter exploration (8 tests)"
    Write-Host ""
    Write-Host "Parameters:" -ForegroundColor Yellow
    Write-Host "  -TestSuite  : system_comparison | comprehensive"
    Write-Host "  -TestNumber : Run specific test number"
    Write-Host "  -DryRun     : Preview configurations without execution"
    Write-Host "  -RunAll     : Execute all tests in the selected suite"
    Write-Host "  -OutputDir  : Custom output directory"
}

function Backup-ConfigFile {
    param([string]$ConfigPath)

    $backupPath = "$ConfigPath.backup_$(Get-Date -Format 'yyyyMMdd_HHmmss')"
    Copy-Item $ConfigPath $backupPath
    Write-Host "Configuration backed up to: $backupPath" -ForegroundColor Green
    return $backupPath
}

function Update-YamlParameter {
    param(
        [string]$ConfigPath,
        [string]$Parameter,
        [object]$Value
    )

    $content = Get-Content $ConfigPath
    $valueStr = if ($Value -is [bool]) { $Value.ToString().ToLower() } else { $Value.ToString() }
    $updated = $false

    for ($i = 0; $i -lt $content.Length; $i++) {
        $line = $content[$i]

        switch ($Parameter) {
            "Kp" {
                if ($line -match "^\s*kp:\s*[\d\.]+.*" -and $content[$i-1] -match "pid:") {
                    $content[$i] = "    kp: $valueStr      # Updated by automation"
                    $updated = $true
                    break
                }
            }
            "Ki" {
                if ($line -match "^\s*ki:\s*[\d\.]+.*" -and $content[$i-2] -match "pid:") {
                    $content[$i] = "    ki: $valueStr      # Updated by automation"
                    $updated = $true
                    break
                }
            }
            "Kd" {
                if ($line -match "^\s*kd:\s*[\d\.]+.*" -and $content[$i-3] -match "pid:") {
                    $content[$i] = "    kd: $valueStr      # Updated by automation"
                    $updated = $true
                    break
                }
            }
            "FLS" {
                if ($line -match "^\s*enable:\s*(true|false)" -and $content[$i-1] -match "fls:") {
                    $content[$i] = "    enable: $valueStr"
                    $updated = $true
                    break
                }
            }
            "Wind" {
                if ($line -match "^\s*enable_wind:\s*(true|false)") {
                    $content[$i] = "  enable_wind: $valueStr"
                    $updated = $true
                    break
                }
            }
            "FeedForward" {
                if ($line -match "^\s*enable_feedforward:\s*(true|false)") {
                    $content[$i] = "    enable_feedforward: $valueStr"
                    $updated = $true
                    break
                }
            }
        }

        if ($updated) { break }
    }

    if (-not $updated) {
        Write-Host "Warning: Could not find parameter '$Parameter' in config file" -ForegroundColor Yellow
    }

    $content | Set-Content $ConfigPath
    return $updated
}

function Apply-TestConfiguration {
    param(
        [hashtable]$Config,
        [string]$ConfigPath
    )

    Write-Host "Applying configuration..." -ForegroundColor Blue

    # Apply PID parameters
    if ($Config.ContainsKey("PID")) {
        Update-YamlParameter -ConfigPath $ConfigPath -Parameter "Kp" -Value $Config.PID.Kp
        Update-YamlParameter -ConfigPath $ConfigPath -Parameter "Ki" -Value $Config.PID.Ki
        Update-YamlParameter -ConfigPath $ConfigPath -Parameter "Kd" -Value $Config.PID.Kd
    }

    # Apply boolean flags
    Update-YamlParameter -ConfigPath $ConfigPath -Parameter "FLS" -Value $Config.FLS
    Update-YamlParameter -ConfigPath $ConfigPath -Parameter "Wind" -Value $Config.Wind

    if ($Config.ContainsKey("FeedForward")) {
        Update-YamlParameter -ConfigPath $ConfigPath -Parameter "FeedForward" -Value $Config.FeedForward
    }

    Write-Host "Configuration applied successfully" -ForegroundColor Green
}

function Show-TestPreview {
    param(
        [hashtable]$Config,
        [int]$TestNum,
        [string]$SuiteName
    )

    Write-Host ""
    Write-Host "Test $TestNum : $($Config.Name)" -ForegroundColor Cyan
    Write-Host "  Description: $($Config.Description)" -ForegroundColor Gray
    Write-Host "  PID Gains: Kp=$($Config.PID.Kp), Ki=$($Config.PID.Ki), Kd=$($Config.PID.Kd)" -ForegroundColor White
    Write-Host "  FLS Enabled: $($Config.FLS)" -ForegroundColor $(if ($Config.FLS) { "Green" } else { "Red" })
    Write-Host "  Wind Enabled: $($Config.Wind)" -ForegroundColor $(if ($Config.Wind) { "Green" } else { "Red" })
    Write-Host "  Feed-Forward: $($Config.FeedForward)" -ForegroundColor $(if ($Config.FeedForward) { "Green" } else { "Red" })
}

function Run-SingleTest {
    param(
        [int]$TestNum,
        [hashtable]$TestConfigs,
        [string]$SuiteName
    )

    if (-not $TestConfigs.ContainsKey($TestNum)) {
        Write-Host "Invalid test number: $TestNum for suite: $SuiteName" -ForegroundColor Red
        return $false
    }

    $config = $TestConfigs[$TestNum]

    Write-Host ""
    Write-Host "Running Test $TestNum : $($config.Name)" -ForegroundColor Cyan
    Write-Host "=" * 60
    if ($config.ContainsKey("Description")) {
        Write-Host "Description: $($config.Description)" -ForegroundColor Yellow
    }

    Show-TestPreview -Config $config -TestNum $TestNum -SuiteName $SuiteName

    if ($DryRun) {
        Write-Host "DRY RUN - Configuration would be applied but simulation not executed" -ForegroundColor Magenta
        return $true
    }

    try {
        # Apply configuration
        Apply-TestConfiguration -Config $config -ConfigPath $ConfigPath

        # Brief pause to ensure file is written
        Start-Sleep -Seconds 2

        # Run the simulation
        Write-Host "Starting simulation..." -ForegroundColor Blue
        $startTime = Get-Date

        # Run simulation without output redirection to avoid hanging
        Push-Location (Split-Path $ExePath)

        try {
            # Run the executable directly without redirection
            $exitCode = & ".\multi_drone_pid_tester.exe"
            $exitCode = $LASTEXITCODE

            $endTime = Get-Date
            $duration = ($endTime - $startTime).TotalSeconds

            if ($exitCode -eq 0 -or $null -eq $exitCode) {
                Write-Host "Simulation completed successfully in $([math]::Round($duration, 1)) seconds" -ForegroundColor Green

                # Organize and copy results
                $success = Copy-SimulationResults -TestConfig $config -TestNumber $TestNum -SuiteName $SuiteName
                return $success
            }
            else {
                Write-Host "Simulation failed with exit code: $exitCode" -ForegroundColor Red
                return $false
            }
        }
        finally {
            Pop-Location
        }
    }
    catch {
        Write-Host "Test failed with error: $($_.Exception.Message)" -ForegroundColor Red
        return $false
    }
}

function Copy-SimulationResults {
    param(
        [hashtable]$TestConfig,
        [int]$TestNumber,
        [string]$SuiteName
    )

    try {
        # Create test-specific directory
        $testDir = Join-Path $OutputDir "$($TestNumber.ToString('00'))_$($TestConfig.Name)"
        if (-not (Test-Path $testDir)) {
            New-Item -ItemType Directory -Path $testDir -Force | Out-Null
        }

        # Find and copy recent simulation output files
        if (Test-Path $SimOutputDir) {
            $cutoffTime = (Get-Date).AddMinutes(-2)  # Files created in last 2 minutes

            # Copy CSV files
            Get-ChildItem "$SimOutputDir\*.csv" | Where-Object { $_.LastWriteTime -gt $cutoffTime } | ForEach-Object {
                $newName = "$($TestConfig.Name)_$($_.Name)"
                $destPath = Join-Path $testDir $newName
                Copy-Item $_.FullName -Destination $destPath
                Write-Host "Copied: $($_.Name) -> $newName" -ForegroundColor Blue
            }

            # Copy metrics files
            Get-ChildItem "$SimOutputDir\*.txt" | Where-Object { $_.LastWriteTime -gt $cutoffTime } | ForEach-Object {
                $newName = "$($TestConfig.Name)_$($_.Name)"
                $destPath = Join-Path $testDir $newName
                Copy-Item $_.FullName -Destination $destPath
                Write-Host "Copied: $($_.Name) -> $newName" -ForegroundColor Blue
            }

            # Create test configuration summary
            $configSummary = @"
Test Configuration Summary
=========================
Test Number: $TestNumber
Test Name: $($TestConfig.Name)
Suite: $SuiteName
Timestamp: $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')

Parameters:
-----------
PID Controller:
  Kp: $($TestConfig.PID.Kp)
  Ki: $($TestConfig.PID.Ki)
  Kd: $($TestConfig.PID.Kd)

System Features:
  FLS Enabled: $($TestConfig.FLS)
  Wind Enabled: $($TestConfig.Wind)
  Feed-Forward: $($TestConfig.FeedForward)

Description:
$($TestConfig.Description)
"@

            $configSummary | Out-File -FilePath (Join-Path $testDir "test_config.txt") -Encoding UTF8

            Write-Host "Results saved to: $testDir" -ForegroundColor Green
            return $true
        }
        else {
            Write-Host "Warning: Simulation output directory not found: $SimOutputDir" -ForegroundColor Yellow
            return $false
        }
    }
    catch {
        Write-Host "Failed to copy results: $($_.Exception.Message)" -ForegroundColor Red
        return $false
    }
}

# Main Logic
Write-Host "Multi-Agent Formation Control - Enhanced Automated Testing" -ForegroundColor Cyan
Write-Host "Suite: $TestSuite | Tests Available: Loading..." -ForegroundColor Yellow

if ($Help) {
    Show-Help
    exit 0
}

# Validate prerequisites
if (-not (Test-Path $ConfigPath)) {
    Write-Host "ERROR: Configuration file not found: $ConfigPath" -ForegroundColor Red
    exit 1
}

if (-not (Test-Path $ExePath)) {
    Write-Host "ERROR: Executable not found: $ExePath" -ForegroundColor Red
    Write-Host "Please build the project first:" -ForegroundColor Yellow
    Write-Host "  cd agent_control_pkg\build"
    Write-Host "  cmake --build . --config Release"
    exit 1
}

# Get test configurations
$TestConfigs = Get-TestConfigs -Suite $TestSuite
if ($null -eq $TestConfigs) {
    Show-Help
    exit 1
}

Write-Host "Suite: $TestSuite | Tests Available: $($TestConfigs.Count)" -ForegroundColor Yellow
Write-Host ""

# Backup original config
$backupFile = $null
if (-not $DryRun) {
    $backupFile = Backup-ConfigFile -ConfigPath $ConfigPath
}

if ($DryRun) {
    Write-Host "DRY RUN - Preview of available test configurations:" -ForegroundColor Magenta
    Write-Host "=================================================" -ForegroundColor Magenta

    $testNumbers = $TestConfigs.Keys | Sort-Object
    foreach ($testNum in $testNumbers) {
        $config = $TestConfigs[$testNum]
        Show-TestPreview -Config $config -TestNum $testNum -SuiteName $TestSuite
    }

    Write-Host ""
    Write-Host "Next Steps:" -ForegroundColor Green
    Write-Host "  - Run specific test: .\run_simulations_enhanced.ps1 -TestNumber X"
    Write-Host "  - Run all tests: .\run_simulations_enhanced.ps1 -RunAll"
    Write-Host "  - Change suite: .\run_simulations_enhanced.ps1 -TestSuite comprehensive -DryRun"

    exit 0
}

if ($TestNumber -gt 0) {
    if ($TestConfigs.ContainsKey($TestNumber)) {
        Write-Host "Executing single test..." -ForegroundColor Green

        # Create output directory
        if (-not (Test-Path $OutputDir)) {
            New-Item -ItemType Directory -Path $OutputDir -Force | Out-Null
            Write-Host "Created output directory: $OutputDir" -ForegroundColor Green
        }

        $success = Run-SingleTest -TestNum $TestNumber -TestConfigs $TestConfigs -SuiteName $TestSuite

        if ($success) {
            Write-Host ""
            Write-Host "Test $TestNumber completed successfully!" -ForegroundColor Green
            Write-Host "Results saved to: $OutputDir" -ForegroundColor Blue
        } else {
            Write-Host "Test $TestNumber failed!" -ForegroundColor Red
        }
    } else {
        Write-Host "ERROR: Test number $TestNumber not found in suite '$TestSuite'" -ForegroundColor Red
        Write-Host "Available tests: $($TestConfigs.Keys -join ', ')" -ForegroundColor Yellow
    }

    # Restore config
    if ($backupFile -and (Test-Path $backupFile)) {
        Copy-Item $backupFile $ConfigPath
        Write-Host "Configuration restored from backup" -ForegroundColor Blue
    }

    exit 0
}

if ($RunAll) {
    Write-Host "Executing full test suite..." -ForegroundColor Green

    # Create output directory
    if (-not (Test-Path $OutputDir)) {
        New-Item -ItemType Directory -Path $OutputDir -Force | Out-Null
        Write-Host "Created output directory: $OutputDir" -ForegroundColor Green
    }

    $results = @{}
    $successCount = 0
    $startTime = Get-Date

    $testNumbers = $TestConfigs.Keys | Sort-Object
    foreach ($testNum in $testNumbers) {
        Write-Host ""
        Write-Host "Progress: $testNum/$($TestConfigs.Count)" -ForegroundColor Blue

        $success = Run-SingleTest -TestNum $testNum -TestConfigs $TestConfigs -SuiteName $TestSuite
        $results[$testNum] = $success

        if ($success) {
            $successCount++
        }

        # Brief pause between tests
        Start-Sleep -Seconds 3
    }

    $endTime = Get-Date
    $totalDuration = ($endTime - $startTime).TotalMinutes

    Write-Host ""
    Write-Host "Test Suite Complete!" -ForegroundColor Green
    Write-Host "Successful: $successCount/$($TestConfigs.Count)" -ForegroundColor Green
    Write-Host "Total Time: $([math]::Round($totalDuration, 1)) minutes" -ForegroundColor Blue
    Write-Host "Results saved in: $OutputDir" -ForegroundColor Yellow

    # Restore config
    if ($backupFile -and (Test-Path $backupFile)) {
        Copy-Item $backupFile $ConfigPath
        Write-Host "Configuration restored from backup" -ForegroundColor Blue
    }

    exit 0
}

# Default: Show available tests
Write-Host "Available tests in suite '$TestSuite':" -ForegroundColor Yellow
$testNumbers = $TestConfigs.Keys | Sort-Object
foreach ($testNum in $testNumbers) {
    $config = $TestConfigs[$testNum]
    Write-Host "  $testNum. $($config.Name) - $($config.Description)" -ForegroundColor Cyan
}

Write-Host ""
Write-Host "Use -DryRun to see detailed configurations, or -Help for usage instructions" -ForegroundColor Green
