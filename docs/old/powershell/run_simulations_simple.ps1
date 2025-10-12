param(
    [switch]$RunAll,
    [string]$TestSuite = "system_comparison",
    [int]$TestNumber = 0,
    [switch]$Help,
    [switch]$DryRun,
    [string]$OutputDir = ""
)

# Configuration
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$ConfigPath = Join-Path $ScriptDir "agent_control_pkg\config\simulation_params.yaml"
$ExePath = Join-Path $ScriptDir "agent_control_pkg\build\Release\multi_drone_pid_tester.exe"

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
    Write-Host "Multi-Agent Formation Control - Simulation Testing Suite" -ForegroundColor Cyan
    Write-Host "=========================================================" -ForegroundColor Cyan
    Write-Host ""
    Write-Host "Usage:" -ForegroundColor Yellow
    Write-Host "  .\run_simulations_simple.ps1 -DryRun                        # Preview available tests"
    Write-Host "  .\run_simulations_simple.ps1 -TestSuite system_comparison   # Show system comparison tests"
    Write-Host "  .\run_simulations_simple.ps1 -TestNumber 1                  # Run specific test"
    Write-Host "  .\run_simulations_simple.ps1 -RunAll                        # Run all tests in suite"
    Write-Host ""
    Write-Host "Test Suites:" -ForegroundColor Green
    Write-Host "  system_comparison : Compare main system configurations (5 tests)"
    Write-Host "  comprehensive     : Extended parameter exploration (5+ tests)"
    Write-Host ""
    Write-Host "Parameters:" -ForegroundColor Yellow
    Write-Host "  -TestSuite  : system_comparison | comprehensive"
    Write-Host "  -TestNumber : Run specific test number"
    Write-Host "  -DryRun     : Preview configurations without execution"
    Write-Host "  -RunAll     : Execute all tests in the selected suite"
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

# Main Logic
Write-Host "Multi-Agent Formation Control - Automated Testing" -ForegroundColor Cyan
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
    Write-Host "  - Run specific test: .\run_simulations_simple.ps1 -TestNumber X"
    Write-Host "  - Run all tests: .\run_simulations_simple.ps1 -RunAll"
    Write-Host "  - Change suite: .\run_simulations_simple.ps1 -TestSuite comprehensive -DryRun"

    exit 0
}

if ($TestNumber -gt 0) {
    if ($TestConfigs.ContainsKey($TestNumber)) {
        $config = $TestConfigs[$TestNumber]
        Write-Host "Running Test $TestNumber : $($config.Name)" -ForegroundColor Green
        Show-TestPreview -Config $config -TestNum $TestNumber -SuiteName $TestSuite
        Write-Host ""
        Write-Host "NOTE: Actual simulation execution not implemented in this simplified version" -ForegroundColor Yellow
        Write-Host "Use the full run_comprehensive_simulations.ps1 for actual execution" -ForegroundColor Yellow
    } else {
        Write-Host "ERROR: Test number $TestNumber not found in suite '$TestSuite'" -ForegroundColor Red
        Write-Host "Available tests: $($TestConfigs.Keys -join ', ')" -ForegroundColor Yellow
    }
    exit 0
}

if ($RunAll) {
    Write-Host "Would run all $($TestConfigs.Count) tests in suite: $TestSuite" -ForegroundColor Green
    Write-Host "NOTE: Use the full run_comprehensive_simulations.ps1 for actual execution" -ForegroundColor Yellow
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
