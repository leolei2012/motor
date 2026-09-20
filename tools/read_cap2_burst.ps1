param([string]$Port = "COM4", [int]$Baud = 9600)
# Read cap2 block in ONE Modbus burst (0x2050..0x213F) but split into 4 chunks of 60 regs
# and decode. Do it fast (minimal sleep) to minimize races, repeat a few times and keep the
# snapshot that is internally consistent (same switch_count across chunks).
function Read-RTU([int]$addr, [int]$qty){
  $sp = New-Object System.IO.Ports.SerialPort($Port, $Baud, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
  $sp.ReadTimeout=900; $sp.WriteTimeout=900; $sp.Open()
  $req=[byte[]](0x01,0x03,(($addr-shr 8)-band 0xFF),($addr-band 0xFF),(($qty-shr 8)-band 0xFF),($qty-band 0xFF))
  $crc=0xFFFF
  foreach($bte in $req){$crc=$crc-bxor$bte;for($i=0;$i-lt 8;$i++){if($crc-band 1){$crc=($crc-shr 1)-bxor 0xA001}else{$crc=$crc-shr 1}}}
  $full=$req+[byte[]](($crc-band 0xFF),(($crc-shr 8)-band 0xFF))
  $sp.Write($full,0,$full.Length); Start-Sleep -Milliseconds 80
  $n=$sp.BytesToRead; $resp=New-Object byte[] $n; $null=$sp.Read($resp,0,$n); $sp.Close(); return $resp
}
function F([byte[]]$bytes, [int]$i){
  $hi = ($bytes[3+2*$i] -shl 8) -bor $bytes[4+2*$i]
  $lo = ($bytes[5+2*$i] -shl 8) -bor $bytes[6+2*$i]
  [uint32]$u = ([uint32]$hi -shl 16) -bor $lo
  $b = [byte[]]@(($u -band 0xFF),(($u -shr 8)-band 0xFF),(($u -shr 16)-band 0xFF),(($u -shr 24)-band 0xFF))
  return [BitConverter]::ToSingle($b,0)
}

for($attempt=0; $attempt -lt 8; $attempt++){
  # chunk 1: header 0x2050..0x205F (16 regs)
  $c0 = Read-RTU 0x2050 16
  if($c0.Length -lt 37){ continue }
  $sc_h = ($c0[3]-shl 8)-bor$c0[4]; $sc_l = ($c0[5]-shl 8)-bor$c0[6]
  $sc = [int64]([int64]$sc_h -shl 16) -bor $sc_l
  $tk = [int64]((($c0[7]-shl 8)-bor$c0[8]) -shl 16) -bor (($c0[9]-shl 8)-bor$c0[10])
  # read points 0..6 (0x2060..0x20CF, 7*16=112 regs) in 2 chunks of 56
  $c1 = Read-RTU 0x2060 56
  $c2 = Read-RTU 0x20D0 56
  if($c1.Length -lt 115 -or $c2.Length -lt 115){ continue }
  # verify switch_count unchanged (consistency)
  $c0b = Read-RTU 0x2050 2
  $sc2 = [int64]((($c0b[3]-shl 8)-bor$c0b[4]) -shl 16) -bor (($c0b[5]-shl 8)-bor$c0b[6])
  if($sc -ne $sc2){ continue }  # raced, skip
  "=== consistent snapshot (switch_count=$sc, survival=$tk ticks = $([math]::Round($tk/16000.0,3))s) ==="
  $offs = @(1,2,4,8,16,32,64,128,256,512,1024,2048,4096,8192)
  "{0,-3} {1,6} {2,10} {3,10} {4,9} {5,10} {6,10} {7,11} {8,11} {9,10}" -f "k","off","frame","obs","spd","va","vb","x1","x2","lam"
  for($k=0; $k -lt 14; $k++){
    $chunk = if($k -lt 7){ $c1 } else { $c2 }
    $j = if($k -lt 7){ $k } else { $k - 7 }
    $frame = F $chunk ($j*8+0)
    $obs   = F $chunk ($j*8+1)
    $spd   = F $chunk ($j*8+2)
    $va    = F $chunk ($j*8+3)
    $vb    = F $chunk ($j*8+4)
    $x1    = F $chunk ($j*8+5)
    $x2    = F $chunk ($j*8+6)
    $lam   = F $chunk ($j*8+7)
    "{0,-3} {1,6} {2,10:F4} {3,10:F4} {4,9:F2} {5,10:F4} {6,10:F4} {7,11:F6} {8,11:F6} {9,10:F6}" -f $k,$offs[$k],$frame,$obs,$spd,$va,$vb,$x1,$x2,$lam
  }
  break
}
