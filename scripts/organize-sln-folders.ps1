# Organize a CloudSim *.sln into functional solution folders (Solution Explorer only).
# VS requires UTF-8 BOM when folder names contain non-ASCII.
param(
  [Parameter(Mandatory = $false)]
  [string]$SlnPath = ""
)

$ErrorActionPreference = "Stop"

if (-not $SlnPath) {
  $SlnPath = Join-Path $PSScriptRoot "..\CloudSim.sln"
}
$path = (Resolve-Path $SlnPath).Path

$utf8 = New-Object System.Text.UTF8Encoding $true
$lines = [System.Collections.Generic.List[string]]::new()
$lines.AddRange([string[]]([System.IO.File]::ReadAllLines($path, $utf8)))

$folderType = "{2150E333-8FDC-42A3-9474-1A3956D46DE8}"

$F_APP   = "{F0010001-0001-4000-8000-000000000001}"
$F_CTR   = "{F0010002-0002-4000-8000-000000000002}"
$F_HOST  = "{F0010003-0003-4000-8000-000000000003}"
$F_UI    = "{F0010004-0004-4000-8000-000000000004}"
$F_DATA  = "{F0010005-0005-4000-8000-000000000005}"
$F_ROBOT = "{F0010006-0006-4000-8000-000000000006}"
$F_ALG   = "{F0010007-0007-4000-8000-000000000007}"
$F_PLG   = "{F0010008-0008-4000-8000-000000000008}"
$F_SDK   = "{F0010009-0009-4000-8000-000000000009}"
$F_FUNC  = "{F001000A-000A-4000-8000-00000000000A}"
$F_INFRA = "{F001000B-000B-4000-8000-00000000000B}"
$F_WEB   = "{F001000C-000C-4000-8000-00000000000C}"

function U([int[]]$cps) { -join ($cps | ForEach-Object { [char]$_ }) }

$folderMeta = [ordered]@{
  $F_APP   = (U 0x5E94,0x7528)                 # 应用
  $F_CTR   = (U 0x5951,0x7EA6)                 # 契约
  $F_HOST  = (U 0x5BBF,0x4E3B)                 # 宿主
  $F_UI    = (U 0x754C,0x9762)                 # 界面
  $F_DATA  = (U 0x6570,0x636E)                 # 数据
  $F_ROBOT = (U 0x673A,0x5668,0x4EBA)          # 机器人
  $F_ALG   = (U 0x7B97,0x6CD5,0x5E93)          # 算法库
  $F_PLG   = (U 0x63D2,0x4EF6)                 # 插件
  $F_SDK   = "SDK"
  $F_FUNC  = (U 0x529F,0x80FD)                 # 功能
  $F_WEB   = (U 0x7F51,0x9875)                 # 网页
  $F_INFRA = (U 0x57FA,0x7840,0x8BBE,0x65BD)   # 基础设施
}

$projGuid = @{}
$rx = [regex]'Project\("\{8BC9CEB8-8B4A-11D0-8D11-00A0C91BC942\}"\) = "([^"]+)", "[^"]+", "(\{[^}]+\})"'
foreach ($line in $lines) {
  $m = $rx.Match($line)
  if ($m.Success) { $projGuid[$m.Groups[1].Value] = $m.Groups[2].Value }
}

$map = @{
  CloudSim = $F_APP; CloudSimWeb = $F_APP; CloudSimBootstrap = $F_APP; SelfTestRunner = $F_APP
  CloudSimCore = $F_CTR; KinematicCore = $F_CTR
  CloudSimHost = $F_HOST; CloudSimHostCore = $F_HOST
  CloudSimHostDesktopViewport = $F_HOST; CloudSimHostHeadless = $F_HOST
  Widget = $F_UI; OsgWidgetCore = $F_UI; BackendVisual = $F_UI
  RobotWidget = $F_UI; AiWidget = $F_UI; CloudSimUiAssets = $F_UI
  Data = $F_DATA
  RobotKinematics = $F_ROBOT; RobotPathPlanning = $F_ROBOT; RobotUrdf = $F_ROBOT
  RobotScene = $F_ROBOT; TrajectoryAlgorithm = $F_ROBOT; TrajectoryAlgorithmBuiltins = $F_ROBOT
  GeometryEngine = $F_ALG; GeometryAlgorithm = $F_ALG; GeometryServices = $F_ALG
  CollisionAlgorithm = $F_ALG; PointCloudAlgorithm = $F_ALG
  InstantMeshesLib = $F_ALG; InstantMeshesCore = $F_ALG; VcgAlgorithms = $F_ALG
  CloudSimPluginSDK = $F_SDK; CloudSimAiSDK = $F_SDK; CloudSimMeshTrajectorySDK = $F_SDK
  PlcCommSDK = $F_SDK; RobotCommSDK = $F_SDK; IndustrialCameraSDK = $F_SDK
  CloudSimControllerSDK = $F_SDK; PlcCommUI = $F_SDK; CloudSimControllerSample = $F_SDK
  PointCloudPlugin = $F_FUNC; GeometryPlugin = $F_FUNC; PlcCommPlugin = $F_FUNC
  PointNetPlugin = $F_FUNC; LabelingPlugin = $F_FUNC; IndustrialCameraPlugin = $F_FUNC
  ProcessFlowPlugin = $F_FUNC; GeometricModelingPlugin = $F_FUNC; EngineeringDrawingPlugin = $F_FUNC
  CloudSimWebGateway = $F_WEB
  RunLogger = $F_INFRA
  Progress = $F_INFRA
}

$unmapped = @($projGuid.Keys | Where-Object { -not $map.ContainsKey($_) })
if ($unmapped.Count) {
  throw ("Unmapped projects: " + ($unmapped -join ", "))
}

# Strip existing solution folders + NestedProjects
$cleaned = [System.Collections.Generic.List[string]]::new()
$i = 0
while ($i -lt $lines.Count) {
  $line = $lines[$i]
  if ($line -match 'Project\("\{2150E333-8FDC-42A3-9474-1A3956D46DE8\}"\)') {
    while ($i -lt $lines.Count -and $lines[$i] -ne "EndProject") { $i++ }
    $i++
    continue
  }
  if ($line -match 'GlobalSection\(NestedProjects\)') {
    while ($i -lt $lines.Count -and $lines[$i] -ne "`tEndGlobalSection") { $i++ }
    $i++
    continue
  }
  $cleaned.Add($line)
  $i++
}
$lines = $cleaned

# Only emit folders referenced by projects in this sln
$usedFolders = [System.Collections.Generic.HashSet[string]]::new([StringComparer]::OrdinalIgnoreCase)
foreach ($name in $projGuid.Keys) {
  [void]$usedFolders.Add($map[$name])
}
if ($usedFolders.Contains($F_SDK) -or $usedFolders.Contains($F_FUNC)) {
  [void]$usedFolders.Add($F_PLG)
}

$folderBlocks = New-Object System.Collections.Generic.List[string]
foreach ($guid in $folderMeta.Keys) {
  if (-not $usedFolders.Contains($guid)) { continue }
  $name = $folderMeta[$guid]
  $folderBlocks.Add(('Project("{0}") = "{1}", "{1}", "{2}"' -f $folderType, $name, $guid))
  $folderBlocks.Add("EndProject")
}

$globalIdx = $lines.IndexOf("Global")
if ($globalIdx -lt 0) { throw "Global not found" }
$lines.InsertRange($globalIdx, $folderBlocks)

$nested = New-Object System.Collections.Generic.List[string]
$nested.Add("`tGlobalSection(NestedProjects) = preSolution")
if ($usedFolders.Contains($F_SDK)) { $nested.Add("`t`t$F_SDK = $F_PLG") }
if ($usedFolders.Contains($F_FUNC)) { $nested.Add("`t`t$F_FUNC = $F_PLG") }
foreach ($name in ($projGuid.Keys | Sort-Object)) {
  $nested.Add("`t`t$($projGuid[$name]) = $($map[$name])")
}
$nested.Add("`tEndGlobalSection")

$insertAt = -1
for ($j = 0; $j -lt $lines.Count; $j++) {
  if ($lines[$j] -match 'GlobalSection\(SolutionProperties\)') { $insertAt = $j; break }
}
if ($insertAt -lt 0) { throw "SolutionProperties not found" }
$lines.InsertRange($insertAt, $nested)

[System.IO.File]::WriteAllLines($path, $lines.ToArray(), $utf8)

$text = [System.IO.File]::ReadAllText($path, $utf8)
$folderNames = @(
  [regex]::Matches($text, 'Project\("\{2150E333[^"]+"\) = "([^"]+)"') |
    ForEach-Object { $_.Groups[1].Value }
)
$vcx = [regex]::Matches($text, 'Project\("\{8BC9CEB8').Count
$nestBody = [regex]::Match($text, '(?s)GlobalSection\(NestedProjects\).*?EndGlobalSection').Value
$missing = @($projGuid.Keys | Where-Object { $nestBody -notmatch [regex]::Escape($projGuid[$_]) })

Write-Host ("Sln: " + [System.IO.Path]::GetFileName($path))
Write-Host ("Folders: " + ($folderNames -join " | "))
Write-Host ("vcxproj=$vcx folders=$($folderNames.Count)")
if ($missing.Count) {
  Write-Host ("MISSING: " + ($missing -join ", "))
  exit 1
}
Write-Host "All projects nested OK"
