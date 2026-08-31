[CmdletBinding()]
param(
    [ValidateSet('Debug', 'Release')]
    [string]$Configuration = 'Debug',
    [string]$SdkRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path,
    [string]$TaskPlan = (Join-Path (Resolve-Path (Join-Path $PSScriptRoot '..')).Path 'task_plan.json')
)

$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$demo = Join-Path $projectRoot "build\\$Configuration\\scheduler_demo.exe"
if (-not (Test-Path -LiteralPath $demo)) { throw "Demo executable not found. Run scripts/build.ps1 first." }
if (-not (Test-Path -LiteralPath (Join-Path $SdkRoot 'zrddslicence.lic'))) {
    throw "ZRDDS license file not found under SDK root: $SdkRoot"
}
if (-not (Test-Path -LiteralPath $TaskPlan)) {
    throw "Task plan not found: $TaskPlan. Run llm_planner.py first."
}

$previousZrddsHome = $env:ZRDDS_HOME
try {
    # The ZRDDS runtime resolves zrddslicence.lic from ZRDDS_HOME at startup.
    $env:ZRDDS_HOME = $SdkRoot
    & $demo --task-plan $TaskPlan --dashboard (Join-Path $projectRoot 'dashboard\\telemetry.json')
    if ($LASTEXITCODE -ne 0) { throw "Demo failed with exit code $LASTEXITCODE" }
}
finally {
    $env:ZRDDS_HOME = $previousZrddsHome
}

Write-Host "Dashboard data: $projectRoot\\dashboard\\telemetry.json"
