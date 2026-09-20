param([string]$Port = "COM4", [int]$Baud = 9600)
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

# Wait until ol_stage==2 (openloop drag), then read shadow (frozen during openloop)
$snapshot = $null
for($t=0; $t -lt 40; $t++){
  $st = Read-RTU 0x2004 1
  if($st.Length -ge 6){
    $ol = ($st[3] -shl 8) -bor $st[4]
    if($ol -eq 2){
      # openloop drag: shadow frozen. read the whole cap2 header + table
      $hdr = Read-RTU 0x2050 12
      $sc_h = ($hdr[3]-shl 8)-bor$hdr[4]; $sc_l = ($hdr[5]-shl 8)-bor$hdr[6]
      $sc = [int64]([int64]$sc_h -shl 16) -bor $sc_l
      $tk = [int64]((($hdr[7]-shl 8)-bor$hdr[8]) -shl 16) -bor (($hdr[9]-shl 8)-bor$hdr[10])
      $vn = ($hdr[11]-shl 8)-bor$hdr[12]
      $mn = F $hdr 6
      $mx = F $hdr 8
      $c1 = Read-RTU 0x2060 56
      $c2 = Read-RTU 0x20D0 56
      $snapshot = @{sc=$sc; tk=$tk; vn=$vn; mn=$mn; mx=$mx; c1=$c1; c2=$c2}
      break
    }
  }
  Start-Sleep -Milliseconds 100
}
if($null -eq $snapshot){ "never saw openloop"; return }
"=== switch_count=$($snapshot.sc)  survival=$($snapshot.tk) ticks ($([math]::Round($snapshot.tk/16000.0,3))s)  valid_n=$($snapshot.vn) ==="
"min_spd=$($snapshot.mn)  max_iq=$($snapshot.mx)"
$offs = @(1,2,4,8,16,32,64,128,256,512,1024,2048,4096,8192)
"{0,-3} {1,6} {2,10} {3,10} {4,9} {5,10} {6,10} {7,11} {8,11} {9,10}" -f "k","off","frame","obs","spd","va","vb","x1","x2","lam"
for($k=0; $k -lt 14; $k++){
  $chunk = if($k -lt 7){ $snapshot.c1 } else { $snapshot.c2 }
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