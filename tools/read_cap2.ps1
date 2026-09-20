param(
  [string]$Port = "COM4",
  [int]$Baud = 9600
)

function Read-RTU([int]$addr, [int]$qty){
  $sp = New-Object System.IO.Ports.SerialPort($Port, $Baud, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
  $sp.ReadTimeout = 1000
  $sp.WriteTimeout = 1000
  $sp.Open()
  $req = [byte[]](0x01,0x03, (($addr -shr 8) -band 0xFF), ($addr -band 0xFF), (($qty -shr 8) -band 0xFF), ($qty -band 0xFF))
  $crc = 0xFFFF
  foreach($bte in $req){
    $crc = $crc -bxor $bte
    for($i=0; $i -lt 8; $i++){
      if($crc -band 1){ $crc = ($crc -shr 1) -bxor 0xA001 } else { $crc = $crc -shr 1 }
    }
  }
  $full = $req + [byte[]](($crc -band 0xFF), (($crc -shr 8) -band 0xFF))
  $sp.Write($full, 0, $full.Length)
  Start-Sleep -Milliseconds 250
  $n = $sp.BytesToRead
  $resp = New-Object byte[] $n
  $null = $sp.Read($resp, 0, $n)
  $sp.Close()
  return $resp
}

# read 0x2050 .. 0x213F in four chunks (60 regs each = 2*60+5 = 125 bytes)
$chunks = @()
foreach($ca in @(0x2050, 0x208C, 0x20C8, 0x2104)){
  $r = Read-RTU $ca 60
  if($r.Length -lt 125){
    "read failed at 0x{0:X4}: got {1} bytes" -f $ca, $r.Length
    exit 1
  }
  $chunks += ,$r[3..122]
}
$all = $chunks[0] + $chunks[1] + $chunks[2] + $chunks[3]

$w = @()
for($i=0; $i -lt $all.Count; $i += 2){ $w += (($all[$i] -shl 8) -bor $all[$i+1]) }

function Get-F32([int]$idx){
  [uint32]$u = (($w[$idx] -shl 16) -bor $w[$idx+1])
  $b = [byte[]]@(($u -band 0xFF), (($u -shr 8) -band 0xFF), (($u -shr 16) -band 0xFF), (($u -shr 24) -band 0xFF))
  return [BitConverter]::ToSingle($b, 0)
}
function Get-U32([int]$idx){
  return [int64](([int64]$w[$idx] -shl 16) -bor $w[$idx+1])
}

$sc = Get-U32 0
$tk = Get-U32 2
$vn = $w[4]
$mn = Get-F32 6
$mx = Get-F32 8

Write-Output ("switch_count = {0}" -f $sc)
Write-Output ("survival_ticks = {0}  ({1} ms)" -f $tk, [math]::Round($tk*0.0625,1))
Write-Output ("valid_n = {0}" -f $vn)
Write-Output ("min_spd = {0} rad/s  ({1} rpm)" -f $mn, [math]::Round($mn*60/(2*[math]::PI),1))
Write-Output ("max_iq  = {0} A" -f $mx)
Write-Output ""
Write-Output ("{0,-3} {1,7} {2,10} {3,10} {4,9} {5,10} {6,10} {7,11} {8,11} {9,11}" -f "k","off","frame","obs","spd","va","vb","x1","x2","lam")
$offs = @(1,2,4,8,16,32,64,128,256,512,1024,2048,4096,8192)
for($k=0; $k -lt 14; $k++){
  $base = 16 + $k*16
  $frame = Get-F32 $base
  $obs   = Get-F32 ($base+2)
  $spd   = Get-F32 ($base+4)
  $va    = Get-F32 ($base+6)
  $vb    = Get-F32 ($base+8)
  $x1    = Get-F32 ($base+10)
  $x2    = Get-F32 ($base+12)
  $lam   = Get-F32 ($base+14)
  Write-Output ("{0,-3} {1,7} {2,10:F4} {3,10:F4} {4,9:F2} {5,10:F4} {6,10:F4} {7,11:F6} {8,11:F6} {9,11:F6}" -f $k,$offs[$k],$frame,$obs,$spd,$va,$vb,$x1,$x2,$lam)
}
