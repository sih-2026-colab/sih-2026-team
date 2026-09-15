# Read-only independent checks of JSON evidence and every CSV field.
param([string]$OutputRoot = (Join-Path $PSScriptRoot '../results/animation'),
    [ValidateSet('occluded_pedestrian','cut_in','animal_crossing','dense_market','missing_lane','intersection')]
    [string[]]$Scenarios = @('occluded_pedestrian','cut_in','animal_crossing','dense_market','missing_lane','intersection'))
$ErrorActionPreference = 'Stop'
function Same($Actual, $Expected, [string]$Context) {
    if ($null -eq $Expected -or ($Expected -is [array] -and $Expected.Count -eq 0)) {
        if ($null -ne $Actual -and [string]$Actual -ne '') { throw "Expected empty: $Context" }
    } elseif ($Expected -is [bool]) {
        if ([string]$Actual -ne [string][int]$Expected) { throw "Boolean mismatch: $Context" }
    } elseif ($Expected -is [ValueType]) {
        $number = [double]::Parse([string]$Actual, [Globalization.CultureInfo]::InvariantCulture)
        if ($number -ne [double]$Expected) { throw "Numeric mismatch: $Context" }
    } elseif ([string]$Actual -cne [string]$Expected) { throw "Text mismatch: $Context" }
}
function SameJson($Actual, $Expected, [string]$Context) {
    if ((ConvertTo-Json -InputObject $Actual -Depth 100 -Compress) -cne
        (ConvertTo-Json -InputObject $Expected -Depth 100 -Compress)) { throw "JSON mismatch: $Context" }
}
foreach ($scenario in $Scenarios) {
    $folder = Join-Path $OutputRoot $scenario
    foreach ($file in @('telemetry.mat','telemetry.json','ego.csv','tracks.csv','candidates.csv','events.json')) {
        $item = Get-Item -LiteralPath (Join-Path $folder $file)
        if ($item.Length -eq 0) { throw "Empty export $scenario/$file" }
    }
    $data = Get-Content -LiteralPath (Join-Path $folder 'telemetry.json') -Raw | ConvertFrom-Json
    $ego = @(Import-Csv -LiteralPath (Join-Path $folder 'ego.csv'))
    $tracks = @(Import-Csv -LiteralPath (Join-Path $folder 'tracks.csv'))
    $candidates = @(Import-Csv -LiteralPath (Join-Path $folder 'candidates.csv'))
    if ($ego.Count -ne $data.frames.Count) { throw 'Ego row count mismatch' }
    $ti=0; $ci=0; $last=-1.0
    for ($k=0; $k -lt $data.frames.Count; $k++) {
        $f=$data.frames[$k]; $s=$f.source; $r=$ego[$k]
        if ($f.time -le $last) { throw 'Nonmonotonic time' }; $last=$f.time
        $values=@($f.time,$s.x,$s.y,$s.egoYaw,($s.speed/3.6),$s.ax,$s.ay,$s.steeringAngle,$s.motionMode,$s.guardianMode,$s.collision,$s.boundaryViolation)
        $fields=@('time','x','y','heading_rad','speed_mps','ax','ay','steering_rad','decision','guardian','collision','boundary_violation')
        for ($j=0; $j -lt $fields.Count; $j++) { Same $r.($fields[$j]) $values[$j] "ego $k/$($fields[$j])" }
        SameJson $f.selected $s.selected 'selected path'
        if ($f.predictionAvailability -ne 'UNAVAILABLE_NOT_CAPTURED_BY_CORE' -or
            $f.safetyBubbleAvailability -ne 'UNAVAILABLE_NOT_CAPTURED_BY_CORE' -or
            $f.predictedObjectTrajectories.Count -ne 0 -or $f.safetyBubble.Count -ne 0) { throw 'Unavailable geometry was populated' }
        if (@($f.tracks).Count -ne @($s.tracks).Count) { throw 'Track count mismatch' }
        foreach ($tr in $f.tracks) {
            $raw=@($s.tracks | Where-Object TrackID -eq $tr.id)
            if ($raw.Count -ne 1) { throw 'Track identity mismatch' }
            SameJson $tr.state $raw[0].State 'track state'
            SameJson $tr.covariance $raw[0].StateCovariance 'track covariance'
            $w=@($s.worldModel | Where-Object id -eq $tr.id)
            if ($w.Count -eq 1) {
                foreach ($field in @('class','sensorSources','confidence','uncertainty','coasted')) {
                    SameJson $tr.$field $w[0].$field "track $field"
                }
            }
            $values=@($f.time,$tr.id,$tr.class,$tr.x,$tr.y,$tr.vx,$tr.vy,$tr.rangeM,$tr.relativeVxMps,$tr.relativeVyMps,$tr.confidence,$tr.uncertainty,($tr.sensorSources -join ';'),$tr.coasted,$tr.attributeAvailability)
            $fields=@('time','track_id','class','x','y','vx','vy','range_m','relative_vx_mps','relative_vy_mps','confidence','uncertainty','sensor_sources','coasted','attribute_availability')
            for ($j=0; $j -lt $fields.Count; $j++) { Same $tracks[$ti].($fields[$j]) $values[$j] "track $ti/$($fields[$j])" }
            $ti++
        }
        if (@($f.candidates).Count -ne @($s.candidates).Count) { throw 'Candidate count mismatch' }
        $index=0
        foreach ($c in $f.candidates) {
            SameJson $c.trajectory $s.candidates[$index].trajectory 'candidate path'
            SameJson $c.planner $s.explainability.plannerResults[$index] 'candidate planner result'
            SameJson $c.guardian $s.explainability.guardianResults[$index] 'candidate Guardian result'
            $values=@($f.time,$c.id,$c.name,$c.targetSpeedKmh,$c.targetY,$c.planner.score,$c.safe,$c.selected,$c.planner.minClearance,$c.guardian.minFrontTTC,$c.guardian.minRearTTC,$c.guardian.risk,($c.rejectionReasons -join ';'),("frames{$($k+1)}.candidates($($index+1)).trajectory"))
            $fields=@('time','candidate_id','name','target_speed_kmh','target_y','score','safe','selected','min_center_distance','min_front_ttc','min_rear_ttc','guardian_risk','rejection_reasons','trajectory_reference')
            for ($j=0; $j -lt $fields.Count; $j++) { Same $candidates[$ci].($fields[$j]) $values[$j] "candidate $ci/$($fields[$j])" }
            $ci++; $index++
        }
    }
    if ($ti -ne $tracks.Count -or $ci -ne $candidates.Count) { throw 'CSV extra rows' }
    $last=-1.0
    foreach ($e in $data.events) {
        if ($e.time -lt $last) { throw 'Nonmonotonic event time' }; $last=$e.time
        $f=$data.frames[[int][math]::Round($e.time/$data.options.dt)]
        if ($f.time -ne $e.time) { throw 'Event time mismatch' }
        $c=@($f.candidates | Where-Object id -eq $e.candidateId)
        switch ($e.event) {
            'PEDESTRIAN_DETECTED' { if (-not @($f.source.sensorFrame.camera | Where-Object type -eq 'pedestrian').Count) { throw 'No pedestrian evidence' } }
            'TRACK_CONFIRMED' { if (-not @($f.tracks | Where-Object id -eq $e.trackId).Count) { throw 'No track evidence' } }
            'CANDIDATE_REJECTED' { if ($c.Count -ne 1 -or $c[0].safe) { throw 'No rejection evidence' } }
            'SAFE_PATH_SELECTED' { if ($c.Count -ne 1 -or -not $c[0].safe -or -not $c[0].selected) { throw 'No safe selection evidence' } }
            'TTC_CRITICAL' {
                if ($c.Count -ne 1) { throw 'No TTC candidate' }; $g=$c[0].guardian
                if (-not (($g.minFrontTTC -is [ValueType] -and $g.minFrontTTC -lt 2) -or
                    ($g.minRearTTC -is [ValueType] -and $g.minRearTTC -lt 1.5))) { throw 'No TTC evidence' }
            }
            'GUARDIAN_INTERVENTION' { if ($f.guardian -in @('APPROVED','NO_ACTION')) { throw 'No Guardian evidence' } }
            default { if ($e.event -ne $f.decision) { throw 'Decision event mismatch' } }
        }
    }
    [pscustomobject]@{scenario=$scenario; files=6; frames=$ego.Count; trackRows=$ti; candidateRows=$ci; eventEvidence=$data.events.Count; passed=$true}
}
