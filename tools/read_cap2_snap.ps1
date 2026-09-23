param([string]$SessionId = "fd9fbe2e-4637-4874-960f-ec5a8455a2cb")
# Fetch device snapshot once, decode cap2 block (0x2050..0x213F) with correct float32 (high-word-first).
function Snap([string]$sid){
  $b = '{"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"name":"get_device_snapshot","arguments":{"device_id":3}}}'
  $r = Invoke-WebRequest -Uri "http://127.0.0.1:8081/mcp" -Method Post -Body $b -ContentType "application/json" -Headers @{"Mcp-Session-Id"=$sid; Accept="application/json, text/event-stream"} -UseBasicParsing
  $m = [regex]::Match($r.Content, '"text":"(.*)"\}\]')
  return $m.Groups[1].Value -replace '\\"','"'
}
$txt = Snap $SessionId
# build addr -> raw int16 (value field, signed)
$map = @{}
foreach($o in [regex]::Matches($txt, '\{[^\{\}]+\}')){
  $s = $o.Value
  if($s -match '"address":(\d+)'){
    $a = [int]$matches[1]
    $v = 0
    if($s -match '"value":(-?\d+)'){ $v = [int]$matches[1] }
    $map[$a] = $v
  }
}
function RawWord([int]$addr){ if($map.ContainsKey($addr)){ return $map[$addr] } else { return 0 } }
function U32([int]$a){ $hi = [uint32]((RawWord $a) -band 0xFFFF); $lo = [uint32]((RawWord ($a+1)) -band 0xFFFF); return ([int64]$hi -shl 16) -bor $lo }
function F32([int]$a){
  $hi = [uint32]((RawWord $a) -band 0xFFFF)
  $lo = [uint32]((RawWord ($a+1)) -band 0xFFFF)
  $u = ($hi -shl 16) -bor $lo
  $b = [byte[]]@(($u -band 0xFF),(($u -shr 8)-band 0xFF),(($u -shr 16)-band 0xFF),(($u -shr 24)-band 0xFF))
  return [BitConverter]::ToSingle($b,0)
}
$sc = U32 0x2050
$tk = U32 0x2052
$vn = RawWord 0x2054
$mn = F32 0x2056
$mx = F32 0x2058
Write-Output ("switch_count={0}  survival_ticks={1} ({2} ms)  valid_n={3}" -f $sc,$tk,[math]::Round($tk/16.0,1),$vn)
Write-Output ("min_spd={0:F4} rad/s ({1} rpm)  max_iq={2:F4} A" -f $mn,($mn*60/(2*[math]::PI)),$mx)
Write-Output ""
$offs = @(1,2,4,8,16,32,64,128,256,512,1024,2048,4096,8192)
Write-Output ("{0,-3} {1,6} {2,10} {3,9} {4,9} {5,9} {6,9} {7,11} {8,11} {9,10}" -f "k","off","frame","ia","ib","iha","ihb","e_a","e_b","za")
for($k=0; $k -lt 14; $k++){
  $base = 0x2060 + $k*16
  $frame = F32 $base
  $obs   = F32 ($base+2)
  $spd   = F32 ($base+4)
  $va    = F32 ($base+6)
  $vb    = F32 ($base+8)
  $x1    = F32 ($base+10)
  $x2    = F32 ($base+12)
  $lam   = F32 ($base+14)
  Write-Output ("{0,-3} {1,6} {2,10:F4} {3,10:F4} {4,9:F2} {5,10:F4} {6,10:F4} {7,11:F6} {8,11:F6} {9,10:F6}" -f $k,$offs[$k],$frame,$obs,$spd,$va,$vb,$x1,$x2,$lam)
}
