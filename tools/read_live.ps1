param([string]$Port = "COM4", [int]$Baud = 9600)

function Read-RTU([int]$addr, [int]$qty){
  $sp = New-Object System.IO.Ports.SerialPort($Port, $Baud, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
  $sp.ReadTimeout = 1000; $sp.WriteTimeout = 1000
  $sp.Open()
  $req = [byte[]](0x01,0x03, (($addr -shr 8) -band 0xFF), ($addr -band 0xFF), (($qty -shr 8) -band 0xFF), ($qty -band 0xFF))
  $crc = 0xFFFF
  foreach($bte in $req){ $crc = $crc -bxor $bte; for($i=0;$i -lt 8;$i++){ if($crc -band 1){ $crc = ($crc -shr 1) -bxor 0xA001 } else { $crc = $crc -shr 1 } } }
  $full = $req + [byte[]](($crc -band 0xFF), (($crc -shr 8) -band 0xFF))
  $sp.Write($full, 0, $full.Length)
  Start-Sleep -Milliseconds 250
  $n = $sp.BytesToRead
  $resp = New-Object byte[] $n
  $null = $sp.Read($resp, 0, $n)
  $sp.Close()
  return $resp
}

# Read several spans and decode live values.
# Integer status 0x2000..0x2006
$r = Read-RTU 0x2000 7
$w = @()
for($i=3; $i -lt $r.Length-2; $i+=2){ $w += (($r[$i] -shl 8) -bor $r[$i+1]) }
Write-Output ("state={0} fault={1} ctrl_mode={2} mode={3} ol_stage={4}" -f $w[0],$w[1],$w[2],$w[3],$w[4])
Write-Output ("tick_count={0}{1}" -f $w[5],$w[6])

# speed_rpm 0x2008, iq 0x200C, id 0x200E, vbus 0x2010 (floats, 2 regs each)
function Get-F([byte[]]$resp, [int]$regIdxFrom3){
  $hi = ($resp[3+2*$regIdxFrom3] -shl 8) -bor $resp[4+2*$regIdxFrom3]
  $lo = ($resp[5+2*$regIdxFrom3] -shl 8) -bor $resp[6+2*$regIdxFrom3]
  [uint32]$u = ([uint32]$hi -shl 16) -bor $lo
  $b = [byte[]]@(($u -band 0xFF), (($u -shr 8)-band 0xFF), (($u -shr 16)-band 0xFF), (($u -shr 24)-band 0xFF))
  return [BitConverter]::ToSingle($b,0)
}

# floats block: 0x2008..0x2043 (60 regs)
$rf = Read-RTU 0x2008 60
$fw = @()
for($i=3; $i -lt $rf.Length-2; $i+=2){ $fw += (($rf[$i] -shl 8) -bor $rf[$i+1]) }
function FF([int]$idx){
  [uint32]$u = ([uint32]$fw[$idx] -shl 16) -bor $fw[$idx+1]
  $b = [byte[]]@(($u -band 0xFF), (($u -shr 8)-band 0xFF), (($u -shr 16)-band 0xFF), (($u -shr 24)-band 0xFF))
  return [BitConverter]::ToSingle($b,0)
}
# index mapping: reg 0x2008 = fw[0], so offset = (reg - 0x2008) but each value is 2 regs.
# speed_rpm 0x2008->fw[0]; phase 0x200A->fw[2]; iq 0x200C->fw[4]; id 0x200E->fw[6]; vbus 0x2010->fw[8]
Write-Output ("speed_rpm={0:F2}" -f (FF 0))
Write-Output ("phase_rad={0:F4}" -f (FF 2))
Write-Output ("iq={0:F3} A  id={1:F3} A" -f (FF 4),(FF 6))
Write-Output ("vbus={0:F2} V" -f (FF 8))
# cap_post_lam 0x2012 -> fw[10], duty 0x2014 -> fw[12]
Write-Output ("cap_post_lam={0:F4}" -f (FF 10))
Write-Output ("duty={0:F4}" -f (FF 12))
# est_speed 0x201C -> fw[20]
Write-Output ("est_speed={0:F2} rad/s" -f (FF 20))
# speed_ref 0x2020 -> fw[24]
Write-Output ("speed_ref_rpm={0:F2}" -f (FF 24))
# v_alpha 0x2028 -> fw[32], v_beta 0x202A -> fw[34]
Write-Output ("v_alpha={0:F4} v_beta={1:F4}" -f (FF 32),(FF 34))
# ia/ib/ic 0x202C/2E/30 -> fw[36],[38],[40]
Write-Output ("ia={0:F3} ib={1:F3} ic={2:F3} A" -f (FF 36),(FF 38),(FF 40))
# cap_post_spd 0x2032 -> fw[42]
Write-Output ("cap_post_spd={0:F2}" -f (FF 42))
# ortega x1 0x2036 -> fw[46], x2 0x2038 -> fw[48], lambda_est 0x203A -> fw[50]
Write-Output ("ortega x1={0:F6} x2={1:F6}" -f (FF 46),(FF 48))
Write-Output ("ortega lambda_est={0:F6}" -f (FF 50))
# i_alpha_last 0x203C -> fw[52], i_beta_last 0x203E -> fw[54]
Write-Output ("i_alpha_last={0:F4} i_beta_last={1:F4}" -f (FF 52),(FF 54))
# r_meas 0x2040 -> fw[56], l_meas 0x2042 -> fw[58]
Write-Output ("r_meas={0:F4} l_meas={1:F6}" -f (FF 56),(FF 58))
