param([string]$Port = "COM4", [int]$Baud = 9600, [int]$Samples = 400)
# Continuous sampler: log ol_stage, est_speed, PLL phase, observer angle, speed, across switchover.
# Reads 0x2004 (ol_stage), 0x201C (est_speed), and a few others each loop.
function Read-RTU([int]$addr, [int]$qty){
  $sp = New-Object System.IO.Ports.SerialPort($Port, $Baud, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
  $sp.ReadTimeout=700; $sp.WriteTimeout=700; $sp.Open()
  $req=[byte[]](0x01,0x03,(($addr-shr 8)-band 0xFF),($addr-band 0xFF),(($qty-shr 8)-band 0xFF),($qty-band 0xFF))
  $crc=0xFFFF
  foreach($bte in $req){$crc=$crc-bxor$bte;for($i=0;$i-lt 8;$i++){if($crc-band 1){$crc=($crc-shr 1)-bxor 0xA001}else{$crc=$crc-shr 1}}}
  $full=$req+[byte[]](($crc-band 0xFF),(($crc-shr 8)-band 0xFF))
  $sp.Write($full,0,$full.Length); Start-Sleep -Milliseconds 50
  $n=$sp.BytesToRead; $resp=New-Object byte[] $n; $null=$sp.Read($resp,0,$n); $sp.Close(); return $resp
}
function F([byte[]]$bytes, [int]$i){
  $hi = ($bytes[3+2*$i] -shl 8) -bor $bytes[4+2*$i]
  $lo = ($bytes[5+2*$i] -shl 8) -bor $bytes[6+2*$i]
  [uint32]$u = ([uint32]$hi -shl 16) -bor $lo
  $b = [byte[]]@(($u -band 0xFF),(($u -shr 8)-band 0xFF),(($u -shr 16)-band 0xFF),(($u -shr 24)-band 0xFF))
  return [BitConverter]::ToSingle($b,0)
}
$rows = New-Object System.Collections.Generic.List[string]
# header
$rows.Add("t_ms,ol_stage,est_speed,speed_rpm,iq,id,vbus,lambda_est,x1,x2")
$sw = [System.Diagnostics.Stopwatch]::StartNew()
for($i=0; $i -lt $Samples; $i++){
  # read ol_stage + tick (0x2004, 1 reg)
  $ol_r = Read-RTU 0x2004 1
  $ol = if($ol_r.Length -ge 6){ ($ol_r[3]-shl 8)-bor$ol_r[4] } else { -1 }
  # read est_speed (0x201C), speed (0x2008), iq (0x200C), id(0x200E), vbus(0x2010) => 0x2008..0x2011 = 10 regs
  $f = Read-RTU 0x2008 10
  $est = F $f 14   # 0x201C is not in this range; read separately below
  $spd = F $f 0
  $iq  = F $f 4
  $id  = F $f 6
  $vbus= F $f 8
  # read est_speed 0x201C
  $ef = Read-RTU 0x201C 2
  $est = F $ef 0
  # read lambda_est 0x203A, x1 0x2036, x2 0x2038 => 0x2036..0x203B = 6 regs
  $orb = Read-RTU 0x2036 6
  $x1 = F $orb 0
  $x2 = F $orb 2
  $lam= F $orb 4
  $t = $sw.ElapsedMilliseconds
  $rows.Add(("{0},{1},{2:F3},{3:F3},{4:F3},{5:F3},{6:F2},{7:F6},{8:F6},{9:F6}" -f $t,$ol,$est,$spd,$iq,$id,$vbus,$lam,$x1,$x2))
}
$rows | Set-Content -Path "C:\Users\vmware\Desktop\workspace_github\motor\tools\switchover_log.csv" -Encoding UTF8
"Logged $($rows.Count-1) samples to tools/switchover_log.csv in $($sw.ElapsedMilliseconds) ms"
