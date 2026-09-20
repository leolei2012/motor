param([string]$Port = "COM4", [int]$Baud = 9600)

function Read-RTU([int]$addr, [int]$qty){
  $sp = New-Object System.IO.Ports.SerialPort($Port, $Baud, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
  $sp.ReadTimeout = 800; $sp.WriteTimeout = 800
  $sp.Open()
  $req = [byte[]](0x01,0x03, (($addr -shr 8) -band 0xFF), ($addr -band 0xFF), (($qty -shr 8) -band 0xFF), ($qty -band 0xFF))
  $crc = 0xFFFF
  foreach($bte in $req){ $crc = $crc -bxor $bte; for($i=0;$i -lt 8;$i++){ if($crc -band 1){ $crc = ($crc -shr 1) -bxor 0xA001 } else { $crc = $crc -shr 1 } } }
  $full = $req + [byte[]](($crc -band 0xFF), (($crc -shr 8) -band 0xFF))
  $sp.Write($full, 0, $full.Length)
  Start-Sleep -Milliseconds 120
  $n = $sp.BytesToRead
  $resp = New-Object byte[] $n
  $null = $sp.Read($resp, 0, $n)
  $sp.Close()
  return $resp
}

# Sample status words 0x2000..0x2006 and float speed/est_speed/cap_post_spd/cap_post_lam over time.
# Read 0x2000..0x2033 = 52 regs = 2*52+5=109 bytes
$N = 60
for($i=0; $i -lt $N; $i++){
  $r = Read-RTU 0x2000 52
  if($r.Length -lt 109){ continue }
  $w = @()
  for($j=3; $j -lt 109; $j+=2){ $w += (($r[$j] -shl 8) -bor $r[$j+1]) }
  function FF([int]$idx){
    [uint32]$u = ([uint32]$w[$idx] -shl 16) -bor $w[$idx+1]
    $b = [byte[]]@(($u -band 0xFF), (($u -shr 8)-band 0xFF), (($u -shr 16)-band 0xFF), (($u -shr 24)-band 0xFF))
    return [BitConverter]::ToSingle($b,0)
  }
  # ints: w[0..6] = 0x2000..0x2006
  $state=$w[0]; $cm=$w[2]; $ol=$w[4]
  # floats begin at w[7]? 0x2007 is start_step byte (int), so floats start at 0x2008 = w[8]
  # speed 0x2008=w[8], phase 0x200A=w[10], iq 0x200C=w[12], id 0x200E=w[14], vbus 0x2010=w[16]
  # cap_post_lam 0x2012=w[18], duty 0x2014=w[20]
  # est_speed 0x201C=w[28]
  # speed_ref 0x2020=w[32]
  # v_alpha 0x2028=w[40], v_beta 0x202A=w[42]
  # cap_post_spd 0x2032=w[50]
  $speed = FF 8
  $iq = FF 12
  $id = FF 14
  $vbus = FF 16
  $postlam = FF 18
  $estspeed = FF 28
  $spdref = FF 32
  $valpha = FF 40
  $vbeta = FF 42
  $postspd = FF 50
  "{0,3} st={1} ol={2} spd={3,6:F1} iq={4,5:F2} id={5,5:F2} vbus={6,4:F1} est={7,6:F1} ref={8,5:F1} va={9,6:F3} vb={10,6:F3} postLam={11,6:F3} postSpd={12,6:F1}" -f $i,$state,$ol,$speed,$iq,$id,$vbus,$estspeed,$spdref,$valpha,$vbeta,$postlam,$postspd
  Start-Sleep -Milliseconds 30
}
