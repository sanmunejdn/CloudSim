# Widget 工程依赖门禁：禁止 Widget 源文件直连 Data/引擎头；链接库允许 Robot 传递依赖（待 DocumentPage 机器人逻辑迁入 RobotWidget 后可收紧）
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$widgetVcx = Join-Path $root 'CloudSim\src\UI\Widget\Widget.vcxproj'
$widgetSrc = Join-Path $root 'CloudSim\src\UI\Widget\source'

$fail = $false

if (-not (Test-Path $widgetVcx)) {
    Write-Error "Widget.vcxproj not found: $widgetVcx"
}

$vcxContent = Get-Content $widgetVcx -Raw
$forbiddenVcx = @(
    'BackendVisual\.lib',
    'GeometryAlgorithm\.lib'
)
foreach ($pat in $forbiddenVcx) {
    if ($vcxContent -match $pat) {
        Write-Host "FAIL vcxproj forbidden: $pat"
        $fail = $true
    }
}

# 仅扫描 Widget.vcxproj 实际编译的 source/*.cpp（OsgWidget 等由 Host 编译）
$compiledSources = @()
[regex]::Matches($vcxContent, 'ClCompile Include="source\\([^"]+\.cpp)"') | ForEach-Object {
    $compiledSources += $_.Groups[1].Value
}

$forbiddenInclude = @(
    'BackendDataManager\.h',
    'MeshBackendData\.h',
    'PointCloudBackendData\.h',
    'FollowAttachmentComponent\.h',
    'RobotScene/',
    'RobotInstructionModel\.h',
    'RobotInstructionProgram\.h',
    'RobotInstructionPropertySchema\.h',
    'RobotInstructionPlanningHelpers',
    'OsgWidgetCore/',
    'BackendVisual/',
    'OsgWidget\.h'
)

# 过渡文件：阶段 B/C 迁移完成前仍允许 OsgWidget / RobotInstruction 头（新文件不得加入此表）
$transitionalIncludeAllow = @{
    'WidgetSceneSignalWiring.cpp' = $true
    'MainWindow.cpp' = $true
    'MainWindowRobotStubs.cpp' = $true
    'MainWindowUiSetup.cpp' = $true
    'MainWindowBackendTree.cpp' = $true
    'MainWindowFileImport.cpp' = $true
    'MainWindowPropertyPanel.cpp' = $true
}

foreach ($rel in $compiledSources) {
    $file = Join-Path $widgetSrc $rel
    if (-not (Test-Path $file)) {
        continue
    }
    if ($transitionalIncludeAllow.ContainsKey($rel)) {
        continue
    }
    $lines = Select-String -Path $file -Pattern '#include' -SimpleMatch
    foreach ($m in $lines) {
        foreach ($pat in $forbiddenInclude) {
            if ($m.Line -match $pat) {
                Write-Host "FAIL ${rel}:$($m.LineNumber) $($m.Line.Trim())"
                $fail = $true
            }
        }
    }
}

if ($fail) {
    Write-Host 'check_widget_deps: FAILED'
    exit 1
}
Write-Host 'check_widget_deps: OK'
exit 0
