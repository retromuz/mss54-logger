<#
.SYNOPSIS
    Simple MAP (manifold absolute pressure) data logger for a BMW MSS54HP DME running CSL firmware.

.DESCRIPTION
    Talks BMW DS2 directly over a K+DCAN / INPA cable (K-line, 9600 8E1, DME address 0x12).
    No EDIABAS needed. Read-only: only identification (0x00) and status-block (0x0B) requests are sent.

    The MAP value is taken from one DS2 status block at a configurable byte position, so it can be
    adjusted if your firmware lays the block out differently. Use -Discover to dump the blocks and
    find/confirm the position.

.EXAMPLE
    .\MapLogger.ps1                         # log MAP on COM1 to the console and a CSV
    .\MapLogger.ps1 -IntervalMs 100 -Csv run1.csv
    .\MapLogger.ps1 -Discover               # dump raw blocks to locate/confirm the MAP field
#>
[CmdletBinding()]
param(
    [string]$Port = 'COM1',
    [int]$IntervalMs = 0,
    [string]$Csv = ("map_log_{0:yyyyMMdd_HHmmss}.csv" -f (Get-Date)),

    # Where the MAP value lives: DS2 status block (0x0B selection), byte position in the full
    # response frame (EDIABAS convention: 0 = address byte, 3 = first data byte), and scaling.
    # Defaults verified on a car running CSL program 0401: block 0x23, u16 at pos 25, 1/32 mbar
    # (engine off ~= ambient, warm idle ~300 mbar).
    [int]$MapBlock = 0x23,
    [int]$MapPos = 25,
    [ValidateSet('u8', 'u16', 's16')][string]$MapType = 'u16',
    [double]$MapFactor = 0.03125,
    [double]$MapOffset = 0.0,

    [switch]$Discover
)

$ErrorActionPreference = 'Stop'
$DmeAddress = 0x12

function Open-KLine([string]$name) {
    $sp = New-Object System.IO.Ports.SerialPort $name, 9600, ([System.IO.Ports.Parity]::Even), 8, ([System.IO.Ports.StopBits]::One)
    $sp.ReadTimeout = 2000
    $sp.WriteTimeout = 1000
    $sp.DtrEnable = $false   # K+DCAN cables: DTR low = K-line mode
    $sp.RtsEnable = $false
    $sp.Open()
    $sp.DiscardInBuffer()
    return $sp
}

function Read-Exact($sp, [int]$count) {
    $buf = New-Object byte[] $count
    $got = 0
    while ($got -lt $count) {
        $n = $sp.Read($buf, $got, $count - $got)   # throws TimeoutException on timeout
        $got += $n
    }
    return , $buf
}

function Invoke-Ds2($sp, [byte[]]$data) {
    # Frame: [addr][len][data...][xor]; len counts the whole frame.
    $frame = New-Object byte[] ($data.Length + 3)
    $frame[0] = $DmeAddress
    $frame[1] = $frame.Length
    [Array]::Copy($data, 0, $frame, 2, $data.Length)
    $cs = 0
    for ($i = 0; $i -lt $frame.Length - 1; $i++) { $cs = $cs -bxor $frame[$i] }
    $frame[$frame.Length - 1] = $cs

    $sp.DiscardInBuffer()
    $sp.Write($frame, 0, $frame.Length)

    # Half-duplex K-line: our own bytes come back first.
    $echo = Read-Exact $sp $frame.Length
    for ($i = 0; $i -lt $frame.Length; $i++) {
        if ($echo[$i] -ne $frame[$i]) { throw "Echo mismatch (check cable/ignition)" }
    }

    $hdr = Read-Exact $sp 2
    $len = $hdr[1]
    if ($len -lt 4) { throw ("Bad response length {0}" -f $len) }
    $rest = Read-Exact $sp ($len - 2)
    $resp = [byte[]]($hdr + $rest)

    $cs = 0
    for ($i = 0; $i -lt $resp.Length - 1; $i++) { $cs = $cs -bxor $resp[$i] }
    if ($cs -ne $resp[$resp.Length - 1]) { throw "Response checksum error" }
    if ($resp[2] -ne 0xA0) { throw ("DME rejected request, status 0x{0:X2}" -f $resp[2]) }
    return , $resp
}

function Get-Value([byte[]]$resp, [int]$pos, [string]$type, [double]$factor, [double]$offset) {
    switch ($type) {
        'u8'  { $raw = [int]$resp[$pos] }
        'u16' { $raw = ([int]$resp[$pos] -shl 8) -bor $resp[$pos + 1] }
        's16' { $raw = ([int]$resp[$pos] -shl 8) -bor $resp[$pos + 1]; if ($raw -ge 0x8000) { $raw -= 0x10000 } }
    }
    return $raw * $factor + $offset
}

function Format-Hex([byte[]]$b) { ($b | ForEach-Object { '{0:X2}' -f $_ }) -join ' ' }

$sp = Open-KLine $Port
try {
    $id = Invoke-Ds2 $sp ([byte[]](0x00))
    $ascii = -join ($id[3..($id.Length - 2)] | ForEach-Object { if ($_ -ge 0x20 -and $_ -lt 0x7F) { [char]$_ } else { '.' } })
    Write-Host ("DME ident: {0}" -f $ascii)

    if ($Discover) {
        # Block 3 has a known ambient pressure (byte 22, mbar = raw*3+500). With key on / engine off,
        # MAP should read about the same; at idle it drops to roughly 300-450 mbar.
        Write-Host ("Discover mode - block 0x{0:X2}, Ctrl+C to stop. Rev the engine and watch which word follows." -f $MapBlock)
        $first = $true
        while ($true) {
            $b3 = Invoke-Ds2 $sp ([byte[]](0x0B, 0x03))
            $bm = Invoke-Ds2 $sp ([byte[]](0x0B, [byte]$MapBlock))
            if ($first) {
                Write-Host ("{0,-22}{1}" -f 'pos ->', ((3..($bm.Length - 2) | ForEach-Object { '{0,2}' -f ($_ % 100) }) -join ' '))
                $first = $false
            }
            $rpm = Get-Value $b3 3 'u16' 1 0
            $pumg = Get-Value $b3 22 'u8' 3 500
            $hex = Format-Hex $bm[3..($bm.Length - 2)]
            Write-Host ("rpm {0,5} amb {1,4}mb  {2}" -f $rpm, $pumg, $hex)
            Start-Sleep -Milliseconds 250
        }
    }

    "timestamp,elapsed_s,rpm,map_mbar,ambient_mbar,map_raw" | Set-Content -Path $Csv -Encoding ASCII
    Write-Host ("Logging MAP from block 0x{0:X2} pos {1} to {2} - Ctrl+C to stop" -f $MapBlock, $MapPos, $Csv)
    $sw = [Diagnostics.Stopwatch]::StartNew()
    while ($true) {
        try {
            $b3 = Invoke-Ds2 $sp ([byte[]](0x0B, 0x03))
            $bm = if ($MapBlock -eq 0x03) { $b3 } else { Invoke-Ds2 $sp ([byte[]](0x0B, [byte]$MapBlock)) }
            $rpm = Get-Value $b3 3 'u16' 1 0
            $amb = Get-Value $b3 22 'u8' 3 500
            $raw = Get-Value $bm $MapPos $MapType 1 0
            $map = $raw * $MapFactor + $MapOffset
            $t = $sw.Elapsed.TotalSeconds
            Write-Host ("{0,8:F2}s  rpm {1,5}  MAP {2,7:F1} mbar  (ambient {3} mbar)" -f $t, $rpm, $map, $amb)
            ("{0:o},{1:F3},{2},{3:F1},{4},{5}" -f (Get-Date), $t, $rpm, $map, $amb, $raw) | Add-Content -Path $Csv -Encoding ASCII
        }
        catch [TimeoutException] { Write-Warning "No response from DME (ignition on?)" }
        catch { Write-Warning $_.Exception.Message }
        if ($IntervalMs -gt 0) { Start-Sleep -Milliseconds $IntervalMs }
    }
}
finally {
    $sp.Close()
}
