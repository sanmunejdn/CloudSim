#Requires -Version 5.1
<#
.SYNOPSIS
  Regenerate CloudSim *.vcxproj.filters by functional rules (CONSENSUS/DESIGN).
#>
param(
  [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path,
  [string[]]$OnlyProjects = @(),
  [string[]]$ExtraProjects = @(
    'src\UI\CloudSimPluginHost\CloudSimPluginHost.vcxproj',
    'src\Plugins\HelloAiPlugin\HelloAiPlugin.vcxproj'
  )
)

$ErrorActionPreference = 'Stop'

function New-FilterGuid {
  return '{' + [guid]::NewGuid().ToString().ToUpper() + '}'
}

function Get-FileLeafStem([string]$include) {
  $leaf = Split-Path $include -Leaf
  return [System.IO.Path]::GetFileNameWithoutExtension($leaf)
}

function Test-ExternalPath([string]$include) {
  if ($include -match '\.\.[\\/]') { return $true }
  if ($include -match '(?i)(^|[\\/])bin[\\/]SDK') { return $true }
  if ($include -match '(?i)^\$\(') { return $true }
  return $false
}

function Get-ExternalFilter([string]$include) {
  $n = $include -replace '/', '\'
  # $(Var)\path\file → External\Var\path
  if ($n -match '^\$\(([^)]+)\)\\(.*)$') {
    $rest = Split-Path $Matches[2] -Parent
    if ([string]::IsNullOrEmpty($rest)) { return "External\$($Matches[1])" }
    return "External\$($Matches[1])\$rest"
  }
  # strip leading ..\ segments, keep remaining dir under External
  $parts = $n -split '\\'
  $i = 0
  while ($i -lt $parts.Length -and $parts[$i] -eq '..') { $i++ }
  if ($i -ge $parts.Length) { return 'External' }
  $remain = $parts[$i..($parts.Length - 1)] -join '\'
  $dir = Split-Path $remain -Parent
  if ([string]::IsNullOrEmpty($dir)) { return 'External' }
  return "External\$dir"
}

function Get-DiskMirrorFilter([string]$include) {
  $n = $include -replace '/', '\'
  # Builtins ops: top-level ops\Name (headers+sources together)
  if ($n -match '(?i)^ops[\\/]([^\\/]+)[\\/]') {
    return "ops\$($Matches[1])"
  }
  # Keep resource tree as-is
  if ($n -match '(?i)^resource([\\/].*)?$') {
    $dir = Split-Path $n -Parent
    if ([string]::IsNullOrEmpty($dir)) { return 'resource' }
    return $dir
  }
  # discretizers top-level
  if ($n -match '(?i)^discretizers[\\/]') {
    return 'discretizers'
  }
  # inc\adapters, source\adapters
  if ($n -match '(?i)^(inc|source|src)[\\/]adapters[\\/]') {
    $root = $Matches[1]
    if ($root -eq 'source') { $root = 'src' }
    return "$root\adapters"
  }
  # inc\sdf|spare, source\sdf|spare
  if ($n -match '(?i)^(inc|source|src)[\\/](sdf|spare)[\\/]') {
    $root = $Matches[1]
    if ($root -eq 'source') { $root = 'src' }
    return "$root\$($Matches[2])"
  }
  # ProcessFlow sim
  if ($n -match '(?i)^(inc|source|src)[\\/]sim[\\/]') {
    $root = $Matches[1]
    if ($root -eq 'source') { $root = 'src' }
    return "$root\sim"
  }
  # Camera SDK vendors
  if ($n -match '(?i)^(inc|source|src)[\\/](calib|hik|mech|pose)[\\/]') {
    $root = $Matches[1]
    if ($root -eq 'source') { $root = 'src' }
    return "$root\$($Matches[2])"
  }
  # CloudSimPluginHost Ai subtree（磁盘 Ai\ + 按职责再细分）
  if ($n -match '(?i)^(inc|source|src)[\\/]Ai[\\/]([^\\/]+)$') {
    $root = $Matches[1]
    if ($root -eq 'source') { $root = 'src' }
    $stem = [System.IO.Path]::GetFileNameWithoutExtension($Matches[2])
    $aiBucket = 'Core'
    if ($stem -match '(?i)^(AiAgent|AiAssistant|AiActionPlan|CatalogActionPlan)') { $aiBucket = 'Agent' }
    elseif ($stem -match '(?i)^(AiLlm|AiHttps|AiConfig|AiIntent|AiArgs|AiCommand|AiProgress)') { $aiBucket = 'Llm' }
    elseif ($stem -match '(?i)(DomainHandler|AiDomain)') { $aiBucket = 'Domains' }
    elseif ($stem -match '(?i)(Catalog|AiApiCatalog|AiTrajectoryFeatureCatalog)') { $aiBucket = 'Catalog' }
    elseif ($stem -match '(?i)(AiScene|AiMesh|AiProcessFlow|GeometryRecognize|MeshCompose|MeshCreate|TrajectoryFeature)') { $aiBucket = 'Rules' }
    elseif ($stem -match '(?i)_global$') { $aiBucket = 'Global' }
    return "$root\Ai\$aiBucket"
  }
  # GeometryAlgorithm deep trees
  if ($n -match '(?i)^source[\\/]detail[\\/]') { return 'src\detail' }
  if ($n -match '(?i)^source[\\/]MeshSurfaceReconstruction[\\/]') { return 'src\MeshSurfaceReconstruction' }
  if ($n -match '(?i)^source[\\/]TubularGrinding[\\/]') { return 'src\TubularGrinding' }
  # Plugin / Global already nested on some projects
  if ($n -match '(?i)^(inc|source|src)[\\/]Plugin[\\/]') {
    $root = $Matches[1]; if ($root -eq 'source') { $root = 'src' }
    return "$root\Plugin"
  }
  if ($n -match '(?i)^(inc|source|src)[\\/]Global[\\/]') {
    $root = $Matches[1]; if ($root -eq 'source') { $root = 'src' }
    return "$root\Global"
  }
  return $null
}

function Get-KindRoot([string]$include, [string]$itemType) {
  $n = $include -replace '/', '\'
  if ($itemType -in @('ClInclude','QtMoc')) {
    if ($n -match '(?i)^inc[\\/]') { return 'inc' }
    return 'inc'
  }
  if ($itemType -eq 'None' -or $itemType -eq 'ResourceCompile' -or $itemType -eq 'QtRcc') {
    if ($n -match '(?i)^resource') { return $null } # handled by disk mirror
    if ($n -match '(?i)\.(ui|qrc|ts|qml)$') { return 'src' }
    return 'src'
  }
  # ClCompile and others
  if ($n -match '(?i)^source[\\/]') { return 'src' }
  if ($n -match '(?i)^src[\\/]') { return 'src' }
  if ($n -match '(?i)^inc[\\/]') { return 'inc' }
  return 'src'
}

function Match-Any([string]$stem, [string[]]$patterns) {
  foreach ($p in $patterns) {
    if ($stem -like $p) { return $true }
  }
  return $false
}

function Get-FunctionalBucket([string]$projectName, [string]$stem) {
  # Global-ish
  if ($stem -match '(?i)^(pch|CloudSimCoreExport)$') { return 'Global' }
  if ($stem -match '(?i)_global$') { return 'Global' }
  if ($stem -match '(?i)_GLOBAL$') { return 'Global' }

  switch ($projectName) {
    'TrajectoryAlgorithmBuiltins' { return 'Common' }

    'GeometryAlgorithm' {
      if (Match-Any $stem @('SelfTest*')) { return 'SelfTest' }
      if (Match-Any $stem @('Feature*','IFeature*','Discretize*','*Discretize*','*Discretizer*','FaceSection*','ParamSurface*')) { return 'FeatureDiscretize' }
      if (Match-Any $stem @('Mesh*','GeoMesh*')) { return 'Mesh' }
      if (Match-Any $stem @('Trajectory*','Tubular*')) { return 'Trajectory' }
      if (Match-Any $stem @('Shape*','Brep*','Primitive*','Shell*','Wire*','Intersection*','Types','ViewTessellate*','TemplateBrep*')) { return 'ShapeBrep' }
      return 'Other'
    }

    'Widget' {
      if (Match-Any $stem @('MainWindow*','DocumentPage*','RunInfoPage*','ApplicationStyle*','StyledDock*','Viewport*','ViewPreset*')) { return 'MainWindow' }
      if (Match-Any $stem @('OsgWidget*','QWidgetViewer*','WidgetOsg*','WidgetRender*','WidgetDocument*','WidgetScene*')) { return 'OsgWidget' }
      if (Match-Any $stem @('Backend*','IBackend*')) { return 'BackendTree' }
      if (Match-Any $stem @('*Pick*','*Operation*','ObjectTransform*','Selection*','Labeling*','MeshSection*','RobotTcp*')) { return 'PickOperations' }
      if (Match-Any $stem @('JobSystem*','ProgressManager*','GraphicsWindow*','LitMesh*','QtKeyboard*','ProjectPackage*','pch','widget_global')) { return 'Infrastructure' }
      return 'Other'
    }

    'RobotWidget' {
      if (Match-Any $stem @('IRobot*')) { return 'Interfaces' }
      if (Match-Any $stem @('RobotInstruction*','Instruction*','ProgramEdit*','PlanResult*','BrandProgram*')) { return 'Instructions' }
      if (Match-Any $stem @('RobotAxis*','RobotCollision*','RobotFrame*','RobotExternal*','RobotComm*','DevicePage*','RobotUrdf*','RobotProject*','RobotOsgUiTypes*')) { return 'RobotSettings' }
      if (Match-Any $stem @('Trajectory*')) { return 'TrajectoryUi' }
      if (Match-Any $stem @('Feature*','MeshTrajectory*','MeshTriangle*')) { return 'FeatureTrajectory' }
      if (Match-Any $stem @('Simulation*','RobotSimulation*','BackendCollision*','PythonScript*')) { return 'Simulation' }
      return 'Other'
    }

    'Data' {
      if (Match-Any $stem @('BackendHierarchy*','BackendFollow*','FollowAttachment*')) { return 'HierarchyFollow' }
      if (Match-Any $stem @('Mesh*','Brep*','PointCloud*','Frame*','Geometry*','PlyIo*','geometry_base64*')) { return 'GeometryBackends' }
      if (Match-Any $stem @('BackendSpatial*','BackendPrimitive*')) { return 'SpatialPrimitive' }
      if (Match-Any $stem @('BackendData*','BackendRegistry*','BackendComponent*','BackendProperty*','BackendRelations*','BackendObject*','Property*')) { return 'Core' }
      return 'Other'
    }

    'RobotScene' {
      if (Match-Any $stem @('IRobot*')) { return 'Interfaces' }
      if (Match-Any $stem @('RobotInstruction*','InstructionProgram*','ProgramEdit*')) { return 'Instructions' }
      if (Match-Any $stem @('RobotProgram*','RobotCanonical*','Recipe*','ProcessFlowPreset*')) { return 'ProgramExport' }
      if (Match-Any $stem @('RobotSceneKinematics*','RobotCoordinate*','RobotExternal*','RobotTeach*','ExternalAxis*','RobotPerLink*','RobotMatrix*')) { return 'KinematicsFrames' }
      if (Match-Any $stem @('Trajectory*','*Trajectory*','RawTrajectory*','Unified*','*Ingress*','PathPlan*')) { return 'Trajectory' }
      if (Match-Any $stem @('RobotCollision*','RobotSceneGeometry*')) { return 'CollisionGeometry' }
      return 'Other'
    }

    'CloudSimHost' {
      if (Match-Any $stem @('DocumentHost*','CloudSimHost*','CloudSimApplication*')) { return 'DocumentHost' }
      if (Match-Any $stem @('*Import*','ProjectPackage*','Annotation*','HierarchyMesh*','BackendProject*','BackendFile*','BackendFollow*','BackendHierarchy*')) { return 'ImportIo' }
      if (Match-Any $stem @('Robot*','Urdf*','PerLink*','IPerLink*','IRobot*')) { return 'Robot' }
      if (Match-Any $stem @('BackendVisual*','Selection*','HostRender*')) { return 'Visual' }
      if (Match-Any $stem @('*Adapter*')) { return 'adapters' }
      return 'Other'
    }

    'PointCloudAlgorithm' {
      if (Match-Any $stem @('Registration*')) { return 'Registration' }
      if (Match-Any $stem @('Reconstruction*')) { return 'Reconstruction' }
      if (Match-Any $stem @('Crop*','Downsample*','Preprocess*','Measure*','Transform*')) { return 'Preprocess' }
      if (Match-Any $stem @('PointCloud*','PointFeatures*','KdTree*','Parallel*','Performance*','SelfTest*')) { return 'Core' }
      return 'Other'
    }

    'ProcessFlowPlugin' {
      if ($stem -eq 'plugin' -or $stem -eq 'plugin.json') { return 'Plugin' }
      if (Match-Any $stem @('ProcessFlowPlugin*','plugin*')) { return 'Plugin' }
      if (Match-Any $stem @('ProcessFlowSim*','*SimController*')) { return 'SimController' }
      if (Match-Any $stem @('DesEngine*','Dispatch*','IDispatch*','IScheduler*','IStation*','JobSet*','OperationTrace*','PlantGraph*','SimModel*','SimRun*','SimStatistics*')) { return 'sim' }
      if (Match-Any $stem @('ProcessFlow*')) { return 'UI' }
      return 'Other'
    }

    'IndustrialCameraSDK' {
      if (Match-Any $stem @('BoardDetector*','HandEye*','MechOfficial*')) { return 'calib' }
      if (Match-Any $stem @('Hik*')) { return 'hik' }
      if (Match-Any $stem @('MechEye*')) { return 'mech' }
      if (Match-Any $stem @('*PoseSource*','ManualPose*','RealRobotPose*','IRobotPose*')) { return 'pose' }
      if (Match-Any $stem @('SimulatedCamera*','CameraFactory*','CameraTypes*','ICamera*')) { return 'Core' }
      return 'Other'
    }

    'IndustrialCameraPlugin' {
      if (Match-Any $stem @('*Plugin*')) { return 'Plugin' }
      if (Match-Any $stem @('HandEye*')) { return 'HandEye' }
      if (Match-Any $stem @('Camera*','IndustrialCamera*')) { return 'UI' }
      return 'Other'
    }

    'CloudSimPluginHost' {
      if (Match-Any $stem @('IPlugin*')) { return 'Interfaces' }
      if (Match-Any $stem @('Document*')) { return 'Document' }
      if (Match-Any $stem @('Plugin*')) { return 'PluginHost' }
      return 'Other'
    }

    'HelloAiPlugin' {
      if (Match-Any $stem @('HelloAi*','*Plugin*','plugin*')) { return 'Plugin' }
      return 'Other'
    }

    'OsgWidgetCore' {
      if (Match-Any $stem @('OsgScene*','OsgCompass*','OsgSection*')) { return 'Scene' }
      if (Match-Any $stem @('*Pick*','Pick*','BrepPick*','MeshTopology*','BackendPick*','BackendVisualBinding*')) { return 'Pick' }
      if (Match-Any $stem @('*Gizmo*','ObjectGizmo*')) { return 'Gizmo' }
      if (Match-Any $stem @('RobotOsg*','PickTypes*')) { return 'Types' }
      return 'Other'
    }

    'BackendVisual' {
      if (Match-Any $stem @('IBackendVisual*','BackendVisualRegistry*','BackendVisualMath*','BackendGeometry*','BackendId*','BackendPose*','BackendPick*')) { return 'Core' }
      if (Match-Any $stem @('Mesh*','Brep*','PointCloud*','Frame*')) { return 'Backends' }
      return 'Other'
    }

    'AiWidget' {
      if (Match-Any $stem @('*Settings*')) { return 'Settings' }
      if (Match-Any $stem @('Ai*')) { return 'Assistant' }
      return 'Other'
    }

    'TrajectoryAlgorithm' {
      if (Match-Any $stem @('ITrajectory*','IOp*','IExternal*','INonRigid*')) { return 'Interfaces' }
      if (Match-Any $stem @('TrajectoryOp*','TrajectoryParam*','TrajectoryTransform*')) { return 'Registry' }
      return 'Other'
    }

    'RobotCommSDK' {
      if (Match-Any $stem @('IRobot*','RobotComm*','RobotMotion*')) { return 'Client' }
      return 'Other'
    }

    'PlcCommSDK' {
      if (Match-Any $stem @('*')) {
        if (Match-Any $stem @('*global*')) { return 'Global' }
        return 'Core'
      }
    }

    'RobotKinematics' {
      if (Match-Any $stem @('Circular*')) { return 'Geometry' }
      if (Match-Any $stem @('Serial*')) { return 'Kinematics' }
      return 'Other'
    }

    'RobotUrdf' {
      if (Match-Any $stem @('Urdf*','RobotSimulation*')) { return 'Loader' }
      return 'Other'
    }

    'CollisionAlgorithm' {
      return 'Core'
    }

    default {
      # Light touch for remaining projects
      if (Match-Any $stem @('*Plugin*')) { return 'Plugin' }
      if (Match-Any $stem @('*global*','*_global','pch','CloudSimCoreExport')) { return 'Global' }
      return 'Core'
    }
  }
}

function Resolve-Filter([string]$projectName, [string]$include, [string]$itemType) {
  if (Test-ExternalPath $include) { return (Get-ExternalFilter $include) }

  $mirror = Get-DiskMirrorFilter $include
  if ($null -ne $mirror) { return $mirror }

  $kind = Get-KindRoot $include $itemType
  if ($null -eq $kind) {
    # resource leaf at root
    return 'resource'
  }

  $stem = Get-FileLeafStem $include
  $bucket = Get-FunctionalBucket $projectName $stem
  if ([string]::IsNullOrEmpty($bucket)) { $bucket = 'Other' }

  # adapters bucket already mirrored when under adapters\; if name-based adapters:
  if ($bucket -eq 'adapters') {
    return "$kind\adapters"
  }
  if ($bucket -eq 'sim' -and $projectName -eq 'ProcessFlowPlugin') {
    return "$kind\sim"
  }
  if ($bucket -in @('calib','hik','mech','pose') -and $projectName -eq 'IndustrialCameraSDK') {
    return "$kind\$bucket"
  }

  return "$kind\$bucket"
}

function Get-ProjectItems([string]$vcxprojPath) {
  [xml]$xml = Get-Content -LiteralPath $vcxprojPath -Encoding UTF8
  $nsUri = 'http://schemas.microsoft.com/developer/msbuild/2003'
  $nsmgr = New-Object System.Xml.XmlNamespaceManager($xml.NameTable)
  $nsmgr.AddNamespace('msb', $nsUri)

  $itemTypes = @('ClInclude','ClCompile','QtMoc','QtUic','QtRcc','None','ResourceCompile','CustomBuild','Image','Text','Xml','Natvis')
  $results = @()
  foreach ($t in $itemTypes) {
    $nodes = $xml.SelectNodes("//msb:ItemGroup/msb:$t[@Include]", $nsmgr)
    if ($null -eq $nodes) { continue }
    foreach ($n in $nodes) {
      $inc = $n.GetAttribute('Include')
      if ([string]::IsNullOrWhiteSpace($inc)) { continue }
      # skip props/targets noise
      if ($inc -match '\.(props|targets)$') { continue }
      # CustomBuild often wraps Qt; keep only file-like
      if ($t -eq 'CustomBuild' -and $inc -notmatch '\.(h|hpp|cpp|cxx|c|ui|qrc|ts|rc|qml)$') { continue }
      $results += [pscustomobject]@{ ItemType = $t; Include = $inc }
    }
  }
  return $results
}

function Ensure-ParentFilters([System.Collections.Generic.HashSet[string]]$set, [string]$filter) {
  if ([string]::IsNullOrEmpty($filter)) { return }
  [void]$set.Add($filter)
  $parts = $filter -split '\\'
  for ($i = 1; $i -lt $parts.Length; $i++) {
    $parent = ($parts[0..($i-1)] -join '\')
    [void]$set.Add($parent)
  }
}

function Write-FiltersFile([string]$filtersPath, $classified) {
  $filterSet = New-Object 'System.Collections.Generic.HashSet[string]' ([StringComparer]::OrdinalIgnoreCase)
  foreach ($c in $classified) {
    Ensure-ParentFilters $filterSet $c.Filter
  }

  $guidMap = @{}
  # Try preserve existing GUIDs
  if (Test-Path -LiteralPath $filtersPath) {
    $raw = Get-Content -LiteralPath $filtersPath -Raw -Encoding UTF8
    [regex]::Matches($raw, '<Filter Include="([^"]+)">\s*<UniqueIdentifier>(\{[^}]+\})</UniqueIdentifier>') | ForEach-Object {
      $guidMap[$_.Groups[1].Value] = $_.Groups[2].Value
    }
  }
  foreach ($f in @($filterSet)) {
    if (-not $guidMap.ContainsKey($f)) { $guidMap[$f] = New-FilterGuid }
  }

  $sb = New-Object System.Text.StringBuilder
  [void]$sb.AppendLine('<?xml version="1.0" encoding="utf-8"?>')
  [void]$sb.AppendLine('<Project xmlns="http://schemas.microsoft.com/developer/msbuild/2003" ToolsVersion="4.0">')
  [void]$sb.AppendLine('  <ItemGroup>')
  foreach ($f in ($filterSet | Sort-Object)) {
    [void]$sb.AppendLine("    <Filter Include=`"$f`">")
    [void]$sb.AppendLine("      <UniqueIdentifier>$($guidMap[$f])</UniqueIdentifier>")
    [void]$sb.AppendLine('    </Filter>')
  }
  [void]$sb.AppendLine('  </ItemGroup>')

  $byType = $classified | Group-Object ItemType
  foreach ($g in ($byType | Sort-Object Name)) {
    [void]$sb.AppendLine('  <ItemGroup>')
    foreach ($item in ($g.Group | Sort-Object Include)) {
      $esc = $item.Include
      [void]$sb.AppendLine("    <$($item.ItemType) Include=`"$esc`">")
      [void]$sb.AppendLine("      <Filter>$($item.Filter)</Filter>")
      [void]$sb.AppendLine("    </$($item.ItemType)>")
    }
    [void]$sb.AppendLine('  </ItemGroup>')
  }
  [void]$sb.AppendLine('</Project>')

  $utf8Bom = New-Object System.Text.UTF8Encoding $true
  [System.IO.File]::WriteAllText($filtersPath, $sb.ToString().Replace("`n", "`r`n"), $utf8Bom)
}

function Process-Project([string]$vcxprojPath) {
  $projectName = [System.IO.Path]::GetFileNameWithoutExtension($vcxprojPath)
  $filtersPath = $vcxprojPath + '.filters'
  $items = @(Get-ProjectItems $vcxprojPath)
  if ($items.Count -eq 0) {
    Write-Host "SKIP (no items): $projectName"
    return
  }
  $classified = foreach ($it in $items) {
    $f = Resolve-Filter $projectName $it.Include $it.ItemType
    [pscustomobject]@{ ItemType = $it.ItemType; Include = $it.Include; Filter = $f }
  }
  Write-FiltersFile $filtersPath $classified
  $filterCount = ($classified | Select-Object -ExpandProperty Filter -Unique).Count
  Write-Host ("OK {0}: {1} items -> {2} filters" -f $projectName, $classified.Count, $filterCount)
}

# Discover SLN projects
$sln = Join-Path $Root 'CloudSim.sln'
if (-not (Test-Path $sln)) { throw "SLN not found: $sln" }
$projPaths = @()
Get-Content $sln | ForEach-Object {
  if ($_ -match 'Project\("\{8BC9CEB8-8B4A-11D0-8D11-00A0C91BC942\}"\)\s*=\s*"[^"]+",\s*"([^"]+\.vcxproj)"') {
    $rel = $Matches[1] -replace '/', '\'
    $projPaths += (Join-Path $Root $rel)
  }
}

# SLN 外工程（PluginHost / HelloAi 等）
foreach ($extra in $ExtraProjects) {
  $full = if ([System.IO.Path]::IsPathRooted($extra)) { $extra } else { Join-Path $Root ($extra -replace '/', '\') }
  if ((Test-Path -LiteralPath $full) -and ($projPaths -notcontains $full)) {
    $projPaths += $full
  }
}

if ($OnlyProjects.Count -gt 0) {
  # 支持 -OnlyProjects A,B 或 'A','B'
  $only = @($OnlyProjects | ForEach-Object { $_ -split ',' } | ForEach-Object { $_.Trim() } | Where-Object { $_ })
  $projPaths = $projPaths | Where-Object {
    $n = [System.IO.Path]::GetFileNameWithoutExtension($_)
    $only -contains $n
  }
}

foreach ($p in $projPaths) {
  if (-not (Test-Path -LiteralPath $p)) {
    Write-Warning "Missing vcxproj: $p"
    continue
  }
  Process-Project $p
}

Write-Host "Done. Processed $($projPaths.Count) projects."
