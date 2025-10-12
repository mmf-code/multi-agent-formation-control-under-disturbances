# Multi-Agent Formation Control - Comprehensive Automated Testing Suite (FIXED VERSION)
# Fixes: Proper YAML handling, prerequisite checks, improved error handling
# Author: Generated for thesis automation
# Date: $(Get-Date -Format 'yyyy-MM-dd')

param(
    [switch]$RunAll,
    [string]$TestSuite = "comprehensive", # Options: comprehensive, pid_variations, zn_methods, system_comparison
    [int]$TestNumber = 0,
    [switch]$Help,
    [switch]$DryRun,
    [string]$OutputDir = "",
    [int]$MaxParallel = 1
)

# ========================================
# Prerequisites Check
# ========================================
function Test-Prerequisites {
    Write-Host "Checking prerequisites..." -ForegroundColor Yellow

    # Check for powershell-yaml module
    try {
        Import-Module powershell-yaml -ErrorAction Stop
        Write-Host "[OK] PowerShell YAML module found" -ForegroundColor Green
        return $true
    }
    catch {
        Write-Host "[ERROR] PowerShell YAML module not found!" -ForegroundColor Red
        Write-Host "Please install it with:" -ForegroundColor Yellow
        Write-Host "  Install-Module -Name powershell-yaml -Force -Scope CurrentUser" -ForegroundColor White
        Write-Host "Run this command as Administrator if needed." -ForegroundColor Yellow
        return $false
    }
}

# ========================================
# Configuration and Paths
# ========================================

# Get script directory for relative paths
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$ProjectRoot = Split-Path -Parent $ScriptDir

# Dynamic path configuration
$ConfigPath = Join-Path $ProjectRoot "agent_control_pkg\config\simulation_params.yaml"
$ExePath = Join-Path $ProjectRoot "agent_control_pkg\build\Release\multi_drone_pid_tester.exe"
$SimOutputDir = Join-Path $ProjectRoot "agent_control_pkg\simulation_outputs"

if ($OutputDir -eq "") {
    $OutputDir = Join-Path $ProjectRoot "final_project_results\automated_testing_$(Get-Date -Format 'yyyyMMdd_HHmmss')"
}

# ========================================
# Test Configurations
# ========================================

# Comprehensive Test Suite - includes all major parameter combinations
$ComprehensiveTestConfigs = @{
    1 = @{
        Name = "01_Baseline_Conservative_PID"
        Description = "Conservative PID tuning - no disturbances"
        PID = @{ Kp = 2.0; Ki = 0.2; Kd = 1.5 }
        FLS = $false
        Wind = $false
        FeedForward = $false
        ZNMode = "Off"
    }
    2 = @{
        Name = "02_Baseline_Optimal_PID"
        Description = "Optimal PID tuning - no disturbances"
        PID = @{ Kp = 3.1; Ki = 0.4; Kd = 2.2 }
        FLS = $false
        Wind = $false
        FeedForward = $false
        ZNMode = "Off"
    }
    3 = @{
        Name = "03_Baseline_Aggressive_PID"
        Description = "Aggressive PID tuning - no disturbances"
        PID = @{ Kp = 5.0; Ki = 0.8; Kd = 3.5 }
        FLS = $false
        Wind = $false
        FeedForward = $false
        ZNMode = "Off"
    }
    4 = @{
        Name = "04_FLS_Conservative_PID"
        Description = "Conservative PID + FLS"
        PID = @{ Kp = 2.0; Ki = 0.2; Kd = 1.5 }
        FLS = $true
        Wind = $false
        FeedForward = $false
        ZNMode = "Off"
    }
    5 = @{
        Name = "05_FLS_Optimal_PID"
        Description = "Optimal PID + FLS"
        PID = @{ Kp = 3.1; Ki = 0.4; Kd = 2.2 }
        FLS = $true
        Wind = $false
        FeedForward = $false
        ZNMode = "Off"
    }
    6 = @{
        Name = "06_Wind_Conservative_PID"
        Description = "Conservative PID under wind disturbances"
        PID = @{ Kp = 2.0; Ki = 0.2; Kd = 1.5 }
        FLS = $false
        Wind = $true
        FeedForward = $false
        ZNMode = "Off"
    }
    7 = @{
        Name = "07_Wind_Optimal_PID"
        Description = "Optimal PID under wind disturbances"
        PID = @{ Kp = 3.1; Ki = 0.4; Kd = 2.2 }
        FLS = $false
        Wind = $true
        FeedForward = $false
        ZNMode = "Off"
    }
    8 = @{
        Name = "08_Wind_FLS_Optimal"
        Description = "Optimal PID + FLS under wind disturbances"
        PID = @{ Kp = 3.1; Ki = 0.4; Kd = 2.2 }
        FLS = $true
        Wind = $true
        FeedForward = $false
        ZNMode = "Off"
    }
    9 = @{
        Name = "09_FeedForward_Only"
        Description = "Optimal PID + Feed-Forward"
        PID = @{ Kp = 3.1; Ki = 0.4; Kd = 2.2 }
        FLS = $false
        Wind = $false
        FeedForward = $true
        ZNMode = "Off"
    }
    10 = @{
        Name = "10_Full_System_Optimal"
        Description = "All features: PID + FLS + Feed-Forward + Wind"
        PID = @{ Kp = 3.1; Ki = 0.4; Kd = 2.2 }
        FLS = $true
        Wind = $true
        FeedForward = $true
        ZNMode = "Off"
    }
    11 = @{
        Name = "11_ZN_Manual_Test"
        Description = "Ziegler-Nichols manual Kp testing"
        PID = @{ Kp = 2.5; Ki = 0.0; Kd = 0.0 }
        FLS = $false
        Wind = $false
        FeedForward = $false
        ZNMode = "Manual"
        ZNParams = @{ kp_test_value = 2.5; simulation_time = 15.0 }
    }
    12 = @{
        Name = "12_ZN_Classic_Method"
        Description = "Ziegler-Nichols Classic quick method"
        PID = @{ Kp = 3.0; Ki = 0.0; Kd = 0.0 }
        FLS = $false
        Wind = $false
        FeedForward = $false
        ZNMode = "Classic"
        ZNParams = @{ manual_ku = 150.0; manual_pu = 12.0 }
    }
    13 = @{
        Name = "13_ZN_Conservative_Method"
        Description = "Ziegler-Nichols Conservative method"
        PID = @{ Kp = 3.0; Ki = 0.0; Kd = 0.0 }
        FLS = $false
        Wind = $false
        FeedForward = $false
        ZNMode = "Conservative"
        ZNParams = @{ manual_ku = 150.0; manual_pu = 12.0 }
    }
}

# PID Variation Test Suite
$PIDVariationConfigs = @{
    1 = @{ Name = "PID_Low_Kp"; PID = @{ Kp = 1.0; Ki = 0.2; Kd = 1.0 }; FLS = $false; Wind = $false; FeedForward = $false; ZNMode = "Off" }
    2 = @{ Name = "PID_Med_Kp"; PID = @{ Kp = 3.0; Ki = 0.2; Kd = 1.0 }; FLS = $false; Wind = $false; FeedForward = $false; ZNMode = "Off" }
    3 = @{ Name = "PID_High_Kp"; PID = @{ Kp = 6.0; Ki = 0.2; Kd = 1.0 }; FLS = $false; Wind = $false; FeedForward = $false; ZNMode = "Off" }
    4 = @{ Name = "PID_Low_Ki"; PID = @{ Kp = 3.0; Ki = 0.1; Kd = 2.0 }; FLS = $false; Wind = $false; FeedForward = $false; ZNMode = "Off" }
    5 = @{ Name = "PID_Med_Ki"; PID = @{ Kp = 3.0; Ki = 0.5; Kd = 2.0 }; FLS = $false; Wind = $false; FeedForward = $false; ZNMode = "Off" }
    6 = @{ Name = "PID_High_Ki"; PID = @{ Kp = 3.0; Ki = 1.0; Kd = 2.0 }; FLS = $false; Wind = $false; FeedForward = $false; ZNMode = "Off" }
    7 = @{ Name = "PID_Low_Kd"; PID = @{ Kp = 3.0; Ki = 0.4; Kd = 0.5 }; FLS = $false; Wind = $false; FeedForward = $false; ZNMode = "Off" }
    8 = @{ Name = "PID_Med_Kd"; PID = @{ Kp = 3.0; Ki = 0.4; Kd = 2.5 }; FLS = $false; Wind = $false; FeedForward = $false; ZNMode = "Off" }
    9 = @{ Name = "PID_High_Kd"; PID = @{ Kp = 3.0; Ki = 0.4; Kd = 4.0 }; FLS = $false; Wind = $false; FeedForward = $false; ZNMode = "Off" }
}

# System Comparison Test Suite
$SystemComparisonConfigs = @{
    1 = @{ Name = "Pure_PID"; PID = @{ Kp = 3.1; Ki = 0.4; Kd = 2.2 }; FLS = $false; Wind = $false; FeedForward = $false; ZNMode = "Off" }
    2 = @{ Name = "PID_with_FLS"; PID = @{ Kp = 3.1; Ki = 0.4; Kd = 2.2 }; FLS = $true; Wind = $false; FeedForward = $false; ZNMode = "Off" }
    3 = @{ Name = "PID_with_Wind"; PID = @{ Kp = 3.1; Ki = 0.4; Kd = 2.2 }; FLS = $false; Wind = $true; FeedForward = $false; ZNMode = "Off" }
    4 = @{ Name = "FLS_with_Wind"; PID = @{ Kp = 3.1; Ki = 0.4; Kd = 2.2 }; FLS = $true; Wind = $true; FeedForward = $false; ZNMode = "Off" }
    5 = @{ Name = "Full_System"; PID = @{ Kp = 3.1; Ki = 0.4; Kd = 2.2 }; FLS = $true; Wind = $true; FeedForward = $true; ZNMode = "Off" }
}

# ========================================
# Helper Functions
# ========================================

function Show-Help {
    Write-Host "Multi-Agent Formation Control - Comprehensive Automated Testing Suite" -ForegroundColor Cyan
    Write-Host "=" * 80
    Write-Host ""
    Write-Host "Usage:" -ForegroundColor Yellow
    Write-Host "  .\run_comprehensive_simulations_fixed.ps1 -RunAll                    # Run all comprehensive tests"
    Write-Host "  .\run_comprehensive_simulations_fixed.ps1 -TestSuite pid_variations  # Run PID parameter variations"
    Write-Host "  .\run_comprehensive_simulations_fixed.ps1 -TestSuite system_comparison # Run system comparison tests"
    Write-Host "  .\run_comprehensive_simulations_fixed.ps1 -TestNumber 5              # Run specific test from current suite"
    Write-Host "  .\run_comprehensive_simulations_fixed.ps1 -DryRun                    # Preview configurations without running"
    Write-Host "  .\run_comprehensive_simulations_fixed.ps1 -Help                      # Show this help"
    Write-Host ""
    Write-Host "Parameters:" -ForegroundColor Yellow
    Write-Host "  -TestSuite      : comprehensive | pid_variations | system_comparison"
    Write-Host "  -OutputDir      : Custom output directory (default: auto-generated)"
    Write-Host "  -MaxParallel    : Maximum parallel simulations (default: 1)"
    Write-Host "  -DryRun         : Preview test configurations without execution"
    Write-Host ""
    Write-Host "Test Suites:" -ForegroundColor Green
    Write-Host "  comprehensive   : Full test suite with all parameter combinations (13 tests)"
    Write-Host "  pid_variations  : Focus on PID parameter effects (9 tests)"
    Write-Host "  system_comparison : Compare main system configurations (5 tests)"
}

function Get-TestConfigs {
    param([string]$Suite)

    switch ($Suite.ToLower()) {
        "comprehensive" { return $ComprehensiveTestConfigs }
        "pid_variations" { return $PIDVariationConfigs }
        "system_comparison" { return $SystemComparisonConfigs }
        default {
            Write-Host "Unknown test suite: $Suite" -ForegroundColor Red
            return $null
        }
    }
}

function Backup-ConfigFile {
    param([string]$ConfigPath)

    $backupPath = "$ConfigPath.backup_$(Get-Date -Format 'yyyyMMdd_HHmmss')"
    Copy-Item $ConfigPath $backupPath
    Write-Host "Configuration backed up to: $backupPath" -ForegroundColor Green
    return $backupPath
}

function Apply-YamlConfiguration {
    param(
        [hashtable]$Config,
        [string]$ConfigPath
    )

    Write-Host "Applying configuration using proper YAML parsing..." -ForegroundColor Blue

    try {
        # Read YAML file as structured object
        $yamlContent = Get-Content $ConfigPath -Raw
        $yamlObject = ConvertFrom-Yaml $yamlContent

        # Apply PID parameters
        if ($Config.ContainsKey("PID")) {
            $yamlObject.controller_settings.pid.kp = $Config.PID.Kp
            $yamlObject.controller_settings.pid.ki = $Config.PID.Ki
            $yamlObject.controller_settings.pid.kd = $Config.PID.Kd
        }

        # Apply FLS setting
        $yamlObject.controller_settings.fls.enable = $Config.FLS

        # Apply Wind setting
        $yamlObject.scenario_settings.enable_wind = $Config.Wind

        # Apply Feed-Forward setting
        if ($Config.ContainsKey("FeedForward")) {
            $yamlObject.controller_settings.pid.enable_feedforward = $Config.FeedForward
        }

        # Handle Ziegler-Nichols settings
        if ($Config.ContainsKey("ZNMode") -and $Config.ZNMode -ne "Off") {
            $yamlObject.simulation_settings.ziegler_nichols_tuning.enable = $true

            if ($Config.ZNMode -eq "Manual" -and $Config.ContainsKey("ZNParams")) {
                $yamlObject.simulation_settings.ziegler_nichols_tuning.kp_test_value = $Config.ZNParams.kp_test_value
            }
            elseif ($Config.ZNMode -in @("Classic", "Conservative", "LessOvershoot", "NoOvershoot") -and $Config.ContainsKey("ZNParams")) {
                $yamlObject.simulation_settings.ziegler_nichols_tuning.quick_method_test = $Config.ZNMode
                $yamlObject.simulation_settings.ziegler_nichols_tuning.manual_ku = $Config.ZNParams.manual_ku
                $yamlObject.simulation_settings.ziegler_nichols_tuning.manual_pu = $Config.ZNParams.manual_pu
            }
        }
        else {
            $yamlObject.simulation_settings.ziegler_nichols_tuning.enable = $false
            $yamlObject.simulation_settings.ziegler_nichols_tuning.quick_method_test = "Off"
        }

        # Write the modified YAML back to file
        $modifiedYaml = ConvertTo-Yaml $yamlObject -Depth 10
        $modifiedYaml | Set-Content $ConfigPath -Encoding UTF8

        Write-Host "Configuration applied successfully" -ForegroundColor Green
        return $true
    }
    catch {
        Write-Host "ERROR: Failed to apply configuration: $($_.Exception.Message)" -ForegroundColor Red
        return $false
    }
}

function Show-ConfigurationPreview {
    param(
        [hashtable]$Config,
        [string]$TestName
    )

    Write-Host "Configuration Preview: $TestName" -ForegroundColor Cyan
    Write-Host "  PID: Kp=$($Config.PID.Kp), Ki=$($Config.PID.Ki), Kd=$($Config.PID.Kd)"
    Write-Host "  FLS: $($Config.FLS)"
    Write-Host "  Wind: $($Config.Wind)"
    if ($Config.ContainsKey("FeedForward")) {
        Write-Host "  FeedForward: $($Config.FeedForward)"
    }
    if ($Config.ContainsKey("ZNMode") -and $Config.ZNMode -ne "Off") {
        Write-Host "  Ziegler-Nichols: $($Config.ZNMode)"
    }
    Write-Host ""
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

    Show-ConfigurationPreview -Config $config -TestName $config.Name

    if ($DryRun) {
        Write-Host "DRY RUN - Configuration would be applied but simulation not executed" -ForegroundColor Magenta
        return $true
    }

    try {
        # Apply configuration using proper YAML handling
        $configSuccess = Apply-YamlConfiguration -Config $config -ConfigPath $ConfigPath
        if (-not $configSuccess) {
            Write-Host "Failed to apply configuration" -ForegroundColor Red
            return $false
        }

        # Brief pause to ensure file is written
        Start-Sleep -Seconds 1

        # Run the simulation
        Write-Host "Starting simulation..." -ForegroundColor Blue
        $startTime = Get-Date

        # Run simulation and capture both output streams
        $processInfo = New-Object System.Diagnostics.ProcessStartInfo
        $processInfo.FileName = $ExePath
        $processInfo.WorkingDirectory = Split-Path $ExePath
        $processInfo.UseShellExecute = $false
        $processInfo.RedirectStandardOutput = $true
        $processInfo.RedirectStandardError = $true

        $process = New-Object System.Diagnostics.Process
        $process.StartInfo = $processInfo
        $process.Start() | Out-Null

        # Wait for completion with timeout (5 minutes)
        $timeout = 300000  # 5 minutes in milliseconds
        if (-not $process.WaitForExit($timeout)) {
            Write-Host "Simulation timed out after 5 minutes, terminating..." -ForegroundColor Yellow
            $process.Kill()
            return $false
        }

        $endTime = Get-Date
        $duration = ($endTime - $startTime).TotalSeconds

        # Capture both output streams
        $standardOutput = $process.StandardOutput.ReadToEnd()
        $standardError = $process.StandardError.ReadToEnd()

        if ($process.ExitCode -eq 0) {
            Write-Host "Simulation completed successfully in $([math]::Round($duration, 1)) seconds" -ForegroundColor Green

            # Organize and copy results
            $success = Copy-SimulationResults -TestConfig $config -TestNumber $TestNum -SuiteName $SuiteName
            return $success
        }
        else {
            Write-Host "Simulation failed with exit code: $($process.ExitCode)" -ForegroundColor Red
            if ($standardOutput) {
                Write-Host "Standard Output:" -ForegroundColor Yellow
                Write-Host $standardOutput -ForegroundColor Gray
            }
            if ($standardError) {
                Write-Host "Standard Error:" -ForegroundColor Yellow
                Write-Host $standardError -ForegroundColor Red
            }
            return $false
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
            $cutoffTime = (Get-Date).AddMinutes(-5)  # Files created in last 5 minutes

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
"@

            if ($TestConfig.ContainsKey("FeedForward")) {
                $configSummary += "`n  Feed-Forward Enabled: $($TestConfig.FeedForward)"
            }

            if ($TestConfig.ContainsKey("ZNMode") -and $TestConfig.ZNMode -ne "Off") {
                $configSummary += "`n  Ziegler-Nichols Mode: $($TestConfig.ZNMode)"
                if ($TestConfig.ContainsKey("ZNParams")) {
                    $configSummary += "`n  ZN Parameters: $($TestConfig.ZNParams | ConvertTo-Json -Compress)"
                }
            }

            if ($TestConfig.ContainsKey("Description")) {
                $configSummary += "`n`nDescription:`n$($TestConfig.Description)"
            }

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

function Run-TestSuite {
    param(
        [hashtable]$TestConfigs,
        [string]$SuiteName
    )

    Write-Host "Starting Test Suite: $SuiteName" -ForegroundColor Cyan
    Write-Host "Timestamp: $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')" -ForegroundColor Yellow
    Write-Host "Total Tests: $($TestConfigs.Count)" -ForegroundColor Blue
    Write-Host "=" * 70

    $results = @{}
    $successCount = 0
    $startTime = Get-Date

    # Create main output directory
    if (-not (Test-Path $OutputDir)) {
        New-Item -ItemType Directory -Path $OutputDir -Force | Out-Null
        Write-Host "Created output directory: $OutputDir" -ForegroundColor Green
    }

    # Run all tests
    $testNumbers = $TestConfigs.Keys | Sort-Object
    foreach ($testNum in $testNumbers) {
        Write-Host ""
        Write-Host "Progress: $testNum/$($TestConfigs.Count) - $('{0:P0}' -f ($testNum / $TestConfigs.Count))" -ForegroundColor Blue

        $success = Run-SingleTest -TestNum $testNum -TestConfigs $TestConfigs -SuiteName $SuiteName
        $results[$testNum] = $success

        if ($success) {
            $successCount++
        }

        # Brief pause between tests
        if ($testNum -lt $TestConfigs.Count) {
            Start-Sleep -Seconds 2
        }
    }

    $endTime = Get-Date
    $totalDuration = ($endTime - $startTime).TotalMinutes

    # Generate comprehensive summary
    Write-Host ""
    Write-Host "Test Suite Complete!" -ForegroundColor Green
    Write-Host "Successful: $successCount/$($TestConfigs.Count)" -ForegroundColor Green
    Write-Host "Total Time: $([math]::Round($totalDuration, 1)) minutes" -ForegroundColor Blue
    Write-Host "Results saved in: $OutputDir" -ForegroundColor Yellow

    # Generate detailed summary report
    Generate-SummaryReport -Results $results -TestConfigs $TestConfigs -SuiteName $SuiteName -Duration $totalDuration -SuccessCount $successCount
}

function Generate-SummaryReport {
    param(
        [hashtable]$Results,
        [hashtable]$TestConfigs,
        [string]$SuiteName,
        [double]$Duration,
        [int]$SuccessCount
    )

    $reportPath = Join-Path $OutputDir "test_suite_summary_$(Get-Date -Format 'yyyyMMdd_HHmmss').txt"

    $summary = @"
Multi-Agent Formation Control - Automated Testing Summary
========================================================
Suite: $SuiteName
Test Date: $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')
Total Tests: $($TestConfigs.Count)
Successful: $SuccessCount
Failed: $($TestConfigs.Count - $SuccessCount)
Total Duration: $([math]::Round($Duration, 1)) minutes
Output Directory: $OutputDir

Detailed Results:
================
"@

    $testNumbers = $Results.Keys | Sort-Object
    foreach ($testNum in $testNumbers) {
        $config = $TestConfigs[$testNum]
        $status = if ($Results[$testNum]) { "PASS" } else { "FAIL" }
        $summary += "`n[$status] Test $testNum : $($config.Name)"
        if ($config.ContainsKey("Description")) {
            $summary += "`n       $($config.Description)"
        }
        $summary += "`n       PID: Kp=$($config.PID.Kp), Ki=$($config.PID.Ki), Kd=$($config.PID.Kd)"
        $summary += "`n       FLS=$($config.FLS), Wind=$($config.Wind)"
        if ($config.ContainsKey("FeedForward")) {
            $summary += ", FF=$($config.FeedForward)"
        }
        $summary += "`n"
    }

    $summary += @"

Analysis Instructions:
=====================
1. Navigate to the output directory: $OutputDir
2. Each test has its own subdirectory with:
   - CSV data files with descriptive names
   - Metrics files with performance summaries
   - test_config.txt with exact parameter settings
3. Use Python analysis scripts for comparative analysis
4. Generate comparative plots and performance metrics

Next Steps:
===========
1. Review failed tests (if any) and investigate causes
2. Run analysis scripts on the generated data
3. Create performance comparison charts for thesis
4. Document findings and conclusions
"@

    $summary | Out-File -FilePath $reportPath -Encoding UTF8
    Write-Host "Comprehensive summary report generated: $reportPath" -ForegroundColor Blue

    # Also create a quick CSV summary for easy analysis
    $csvPath = Join-Path $OutputDir "test_results_summary.csv"
    $csvContent = "TestNumber,TestName,Status,Kp,Ki,Kd,FLS,Wind,FeedForward,ZN_Mode`n"

    foreach ($testNum in $testNumbers) {
        $config = $TestConfigs[$testNum]
        $status = if ($Results[$testNum]) { "PASS" } else { "FAIL" }
        $ff = if ($config.ContainsKey("FeedForward")) { $config.FeedForward } else { "N/A" }
        $zn = if ($config.ContainsKey("ZNMode")) { $config.ZNMode } else { "Off" }

        $csvContent += "$testNum,$($config.Name),$status,$($config.PID.Kp),$($config.PID.Ki),$($config.PID.Kd),$($config.FLS),$($config.Wind),$ff,$zn`n"
    }

    $csvContent | Out-File -FilePath $csvPath -Encoding UTF8 -NoNewline
    Write-Host "CSV summary created: $csvPath" -ForegroundColor Blue
}

# ========================================
# Main Execution Logic
# ========================================

Write-Host "Multi-Agent Formation Control - Automated Testing (FIXED VERSION)" -ForegroundColor Cyan
Write-Host ""

# Check prerequisites first
if (-not (Test-Prerequisites)) {
    exit 1
}

Write-Host "Prerequisites OK - Script ready to use!" -ForegroundColor Green

# Display help if requested
if ($Help) {
    Show-Help
    exit 0
}

# Validate paths and prerequisites
if (-not (Test-Path $ExePath)) {
    Write-Host "Executable not found: $ExePath" -ForegroundColor Red
    Write-Host "Please build the project first:" -ForegroundColor Yellow
    Write-Host "  cd agent_control_pkg\build"
    Write-Host "  cmake --build . --config Release"
    exit 1
}

if (-not (Test-Path $ConfigPath)) {
    Write-Host "Configuration file not found: $ConfigPath" -ForegroundColor Red
    Write-Host "Expected location: $ConfigPath" -ForegroundColor Yellow
    exit 1
}

# Get the appropriate test configuration set
$TestConfigs = Get-TestConfigs -Suite $TestSuite
if ($null -eq $TestConfigs) {
    Show-Help
    exit 1
}

# Backup the original configuration file
$backupFile = $null
if (-not $DryRun) {
    $backupFile = Backup-ConfigFile -ConfigPath $ConfigPath
    Write-Host "Original configuration backed up" -ForegroundColor Green
}

# Execute based on parameters
Write-Host "Suite: $TestSuite" -ForegroundColor Yellow
Write-Host "Tests Available: $($TestConfigs.Count)" -ForegroundColor Blue
Write-Host ""

if ($RunAll) {
    Run-TestSuite -TestConfigs $TestConfigs -SuiteName $TestSuite
}
elseif ($TestNumber -gt 0) {
    if ($TestConfigs.ContainsKey($TestNumber)) {
        $success = Run-SingleTest -TestNum $TestNumber -TestConfigs $TestConfigs -SuiteName $TestSuite
        if ($success -and -not $DryRun) {
            Write-Host ""
            Write-Host "Test $TestNumber completed successfully!" -ForegroundColor Green
            Write-Host "Results saved to: $OutputDir" -ForegroundColor Blue
            Write-Host "Run analysis scripts on the generated data for insights" -ForegroundColor Cyan
        }
    }
    else {
        Write-Host "Test number $TestNumber not found in suite '$TestSuite'" -ForegroundColor Red
        Write-Host "Available tests: $($TestConfigs.Keys -join ', ')" -ForegroundColor Yellow
    }
}
else {
    # Show available tests in the current suite
    Write-Host "Available tests in suite '$TestSuite':" -ForegroundColor Yellow
    $testNumbers = $TestConfigs.Keys | Sort-Object
    foreach ($testNum in $testNumbers) {
        $config = $TestConfigs[$testNum]
        Write-Host "  $testNum. $($config.Name)" -ForegroundColor Cyan
        if ($config.ContainsKey("Description")) {
            Write-Host "      $($config.Description)" -ForegroundColor Gray
        }
    }
    Write-Host ""
    Write-Host "Use -RunAll to run all tests, or specify -TestNumber X to run a specific test" -ForegroundColor Green
    Show-Help
}

# Final cleanup message
if (-not $DryRun -and $backupFile -and (Test-Path $backupFile)) {
    Write-Host ""
    Write-Host "Original configuration can be restored from: $backupFile" -ForegroundColor Blue
}

Write-Host ""
Write-Host "Script execution completed!" -ForegroundColor Green
