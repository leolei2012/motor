param([string]$Port = "COM4", [int]$Baud = 9600)
function Read-RTU([int]$addr, [int]$qty){
  $sp = New-Object System.IO.Ports.SerialPort($Port, $Baud, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
  $sp.ReadTimeout=900; $sp.WriteTimeout=900; $sp.Open()
  $req=[byte[]](0x01,0x03,(($addr-shr 8)-band 0xFF),($addr-band 0xFF),(($qty-shr 8)-band 0xFF),($qty-band 0xFF))
  $crc=0xFFFF
  foreach($bte in $req){$crc=$crc-bxor$bte;for($i=0;$i-lt 8;$i++){if($crc-band 1){$crc=($crc-shr 1)-bxor 0xA001}else{$crc=$crc-shr 1}}}
  $full=$req+[byte[]](($crc-band 0xFF),(($crc-shr 8)-band 0xFF))
  $sp.Write($full,0,$full.Length); Start-Sleep -Milliseconds 150
  $n=$sp.BytesToRead; $resp=New-Object byte[] $n; $null=$sp.Read($resp,0,$n); $sp.Close(); return $resp
}
# Read each point's 8 floats individually (2 regs each = clean)
function F([int]$reg){
  $r = Read-RTU $reg 2
  if($r.Length -lt 7){ return $null }
  $hi = ($r[3] -shl 8) -bor $r[4]
  $lo = ($r[5] -shl 8) -bor $r[6]
  [uint32]$u = ([uint32]$hi -shl 16) -bor $lo
  $b = [byte[]]@(($u -band 0xFF),(($u -shr 8)-band 0xFF),(($u -shr 16)-band 0xFF),(($u -shr 24)-band 0xFF))
  return [BitConverter]::ToSingle($b,0)
}
$offs = @(1,2,4,8,16,32,64,128,256,512,1024,2048,4096,8192)
"{0,-5} {1,7} {2,10} {3,10} {4,9} {5,10} {6,10} {7,11} {8,11} {9,10}" -f "k","off","frame","obs","spd","va","vb","x1","x2","lam"
for($k=0; $k -lt 14; $k++){
  $b = 0x2060 + $k*16
  $frame = F $b
  $obs   = F ($b+2)
  $spd   = F ($b+4)
  $va    = F ($b+6)
  $vb    = F ($b+8)
  $x1    = F ($b+10)
  $x2    = F ($b+12)
  $lam   = F ($b+14)
  "{0,-5} {1,7} {2,10:F4} {3,10:F4} {4,9:F2} {5,10:F4} {6,10:F4} {7,11:F6} {8,11:F6} {9,10:F6}" -f $k,$offs[$k],$frame,$obs,$spd,$va,$vb,$x1,$x2,$lam
}