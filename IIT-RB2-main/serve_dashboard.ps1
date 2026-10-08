# Lightweight, zero-dependency HTTP & Serial Bridge Server for IIT-RB2 Robot Dashboard
param (
    [int]$Port = 8080,
    [string]$RootPath = $PSScriptRoot
)

if (-not $RootPath) {
    $RootPath = (Get-Location).Path
}

# Auto-detect local IPv4
$localIP = "localhost"
try {
    $ips = Get-NetIPAddress -AddressFamily IPv4 | Where-Object { $_.InterfaceAlias -notmatch 'Loopback' -and $_.IPAddress -notmatch '^169\.254\.' }
    if ($ips) {
        $localIP = ($ips | Select-Object -First 1).IPAddress
    }
} catch {
    $ipConfig = ipconfig
    if ($ipConfig -match 'IPv4 Address[^\d]+([\d\.]+)') {
        $localIP = $matches[1]
    }
}

$listener = New-Object System.Net.Sockets.TcpListener([System.Net.IPAddress]::Any, $Port)
try {
    $listener.Start()
} catch {
    Write-Host "Failed to bind to port $Port. It might be in use." -ForegroundColor Red
    exit 1
}

# Global Serial Port Bridge Variables
$global:serialPort = $null
$global:activePort = ""
$global:activeBaud = 115200
$global:telemetryBuffer = [System.Collections.Generic.List[string]]::new()
$global:maxBufferLines = 40

function Open-SerialPort([string]$portName, [int]$baud) {
    try {
        if ($global:serialPort -ne $null -and $global:serialPort.IsOpen) {
            $global:serialPort.Close()
            $global:serialPort.Dispose()
        }
        $global:serialPort = New-Object System.IO.Ports.SerialPort($portName, $baud, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
        $global:serialPort.ReadTimeout = 100
        $global:serialPort.WriteTimeout = 500
        $global:serialPort.DtrEnable = $true
        $global:serialPort.RtsEnable = $true
        $global:serialPort.Open()
        $global:activePort = $portName
        $global:activeBaud = $baud
        Write-Host "[BRIDGE] Connected to $portName at $baud Baud" -ForegroundColor Green
        return $true
    } catch {
        Write-Host "[BRIDGE ERR] Failed to open $portName : $($_.Exception.Message)" -ForegroundColor Red
        return $false
    }
}

function Close-SerialPort() {
    try {
        if ($global:serialPort -ne $null -and $global:serialPort.IsOpen) {
            $global:serialPort.Close()
            $global:serialPort.Dispose()
            $global:serialPort = $null
            Write-Host "[BRIDGE] Serial port closed" -ForegroundColor Yellow
        }
    } catch { }
}

function Send-SerialCmd([string]$cmd) {
    if ($global:serialPort -ne $null -and $global:serialPort.IsOpen) {
        try {
            $global:serialPort.WriteLine($cmd)
            return $true
        } catch {
            return $false
        }
    }
    return $false
}

function Read-SerialTelemetry() {
    if ($global:serialPort -ne $null -and $global:serialPort.IsOpen) {
        try {
            $data = $global:serialPort.ReadExisting()
            if ($data) {
                $lines = $data.Split("`n")
                foreach ($l in $lines) {
                    $trimmed = $l.Trim()
                    if ($trimmed.Length -gt 0) {
                        $global:telemetryBuffer.Add($trimmed)
                        while ($global:telemetryBuffer.Count -gt $global:maxBufferLines) {
                            $global:telemetryBuffer.RemoveAt(0)
                        }
                    }
                }
            }
        } catch { }
    }
}

Write-Host "============================================================" -ForegroundColor Cyan
Write-Host "       IIT-RB2: ROBOT 2 DASHBOARD NETWORK HTTP SERVER       " -ForegroundColor Yellow
Write-Host "============================================================" -ForegroundColor Cyan
Write-Host ""
Write-Host "  [LOCAL PC]     http://localhost:$Port/" -ForegroundColor Green
Write-Host "  [NETWORK/LAN]  http://${localIP}:$Port/" -ForegroundColor Cyan
Write-Host ""
Write-Host "  FEATURES ACTIVE:" -ForegroundColor White
Write-Host "  * Web Serial Direct USB on Localhost" -ForegroundColor Gray
Write-Host "  * Wi-Fi Server Bridge: Control Arduino wirelessly from any device!" -ForegroundColor Gray
Write-Host "  * Auto Hardware Diagnostic Test Suite" -ForegroundColor Gray
Write-Host ""
Write-Host "  Press Ctrl+C in this window to stop the server." -ForegroundColor Yellow
Write-Host "============================================================" -ForegroundColor Cyan

$mimeTypes = @{
    ".html" = "text/html; charset=utf-8"
    ".htm"  = "text/html; charset=utf-8"
    ".css"  = "text/css; charset=utf-8"
    ".js"   = "application/javascript; charset=utf-8"
    ".json" = "application/json; charset=utf-8"
    ".png"  = "image/png"
    ".jpg"  = "image/jpeg"
    ".jpeg" = "image/jpeg"
    ".svg"  = "image/svg+xml"
    ".ico"  = "image/x-icon"
}

while ($true) {
    try {
        $client = $listener.AcceptTcpClient()
        $stream = $client.GetStream()
        $reader = New-Object System.IO.StreamReader($stream)
        $requestLine = $reader.ReadLine()
        
        if ($requestLine) {
            $parts = $requestLine.Split(" ")
            $method = $parts[0]
            if ($parts.Length -ge 2) {
                $rawUrl = [System.Uri]::UnescapeDataString($parts[1])
                $path = $rawUrl.Split("?")[0]
                $query = ""
                if ($rawUrl.Contains("?")) {
                    $query = $rawUrl.Substring($rawUrl.IndexOf("?") + 1)
                }

                # Read telemetry whenever loop runs
                Read-SerialTelemetry

                # Read remaining request headers
                $contentLength = 0
                while (($h = $reader.ReadLine()) -and $h.Trim().Length -gt 0) {
                    if ($h.ToLower().StartsWith("content-length:")) {
                        $contentLength = [int]($h.Split(":")[1].Trim())
                    }
                }
                $postBody = ""
                if ($contentLength -gt 0) {
                    $buffer = New-Object char[] $contentLength
                    $bytesRead = $reader.Read($buffer, 0, $contentLength)
                    $postBody = New-Object string ($buffer, 0, $bytesRead)
                }

                # Helper function for JSON responses
                $sendJsonResponse = {
                    param ($jsonString, [int]$statusCode = 200)
                    $jsonBytes = [System.Text.Encoding]::UTF8.GetBytes($jsonString)
                    $crlf = "`r`n"
                    $respHeader = "HTTP/1.1 $statusCode OK" + $crlf +
                                  "Content-Type: application/json; charset=utf-8" + $crlf +
                                  "Content-Length: " + $jsonBytes.Length + $crlf +
                                  "Access-Control-Allow-Origin: *" + $crlf +
                                  "Access-Control-Allow-Methods: GET, POST, OPTIONS" + $crlf +
                                  "Access-Control-Allow-Headers: Content-Type" + $crlf +
                                  "Connection: close" + $crlf + $crlf
                    $respBytes = [System.Text.Encoding]::ASCII.GetBytes($respHeader)
                    $stream.Write($respBytes, 0, $respBytes.Length)
                    $stream.Write($jsonBytes, 0, $jsonBytes.Length)
                }

                # Handle CORS Preflight
                if ($method -eq "OPTIONS") {
                    $crlf = "`r`n"
                    $optHeader = "HTTP/1.1 204 No Content" + $crlf +
                                 "Access-Control-Allow-Origin: *" + $crlf +
                                 "Access-Control-Allow-Methods: GET, POST, OPTIONS" + $crlf +
                                 "Access-Control-Allow-Headers: Content-Type" + $crlf +
                                 "Connection: close" + $crlf + $crlf
                    $optBytes = [System.Text.Encoding]::ASCII.GetBytes($optHeader)
                    $stream.Write($optBytes, 0, $optBytes.Length)
                    $stream.Close(); $client.Close(); continue
                }

                # ================= API ENDPOINTS =================
                if ($path -eq "/api/ports") {
                    $ports = [System.IO.Ports.SerialPort]::GetPortNames()
                    if (-not $ports) { $ports = @() }
                    $json = @{ ports = $ports } | ConvertTo-Json -Compress
                    & $sendJsonResponse $json
                }
                elseif ($path -eq "/api/status") {
                    $isConnected = ($global:serialPort -ne $null -and $global:serialPort.IsOpen)
                    $ports = [System.IO.Ports.SerialPort]::GetPortNames()
                    $json = @{
                        connected = $isConnected
                        port = $global:activePort
                        baud = $global:activeBaud
                        localIP = $localIP
                        serverPort = $Port
                        availablePorts = $ports
                    } | ConvertTo-Json -Compress
                    & $sendJsonResponse $json
                }
                elseif ($path -eq "/api/connect") {
                    $reqPort = "COM7"
                    $reqBaud = 115200

                    if ($query) {
                        foreach ($part in $query.Split('&')) {
                            if ($part.StartsWith("port=")) { $reqPort = $part.Substring(5) }
                            if ($part.StartsWith("baud=")) { $reqBaud = [int]$part.Substring(5) }
                        }
                    }
                    if ($postBody -and $postBody.Trim().StartsWith("{")) {
                        try {
                            $parsed = $postBody | ConvertFrom-Json
                            if ($parsed.port) { $reqPort = $parsed.port }
                            if ($parsed.baud) { $reqBaud = [int]$parsed.baud }
                        } catch {}
                    }

                    $ok = Open-SerialPort $reqPort $reqBaud
                    $json = @{
                        success = $ok
                        port = $global:activePort
                        baud = $global:activeBaud
                    } | ConvertTo-Json -Compress
                    & $sendJsonResponse $json
                }
                elseif ($path -eq "/api/disconnect") {
                    Close-SerialPort
                    $json = @{ success = $true } | ConvertTo-Json -Compress
                    & $sendJsonResponse $json
                }
                elseif ($path -eq "/api/cmd") {
                    $cmd = ""
                    if ($query) {
                        foreach ($part in $query.Split('&')) {
                            if ($part.StartsWith("c=")) { $cmd = [System.Uri]::UnescapeDataString($part.Substring(2)) }
                        }
                    }
                    if ($postBody) {
                        $pTrim = $postBody.Trim()
                        if ($pTrim.StartsWith("{")) {
                            try {
                                $parsed = $pTrim | ConvertFrom-Json
                                if ($parsed.cmd) { $cmd = $parsed.cmd }
                                elseif ($parsed.c) { $cmd = $parsed.c }
                            } catch {}
                        } else {
                            $cmd = $pTrim
                        }
                    }

                    $sent = Send-SerialCmd $cmd
                    $json = @{
                        success = $sent
                        cmd = $cmd
                        connected = ($global:serialPort -ne $null -and $global:serialPort.IsOpen)
                    } | ConvertTo-Json -Compress
                    & $sendJsonResponse $json
                }
                elseif ($path -eq "/api/telemetry") {
                    Read-SerialTelemetry
                    $telemetryLines = [string[]]$global:telemetryBuffer.ToArray()
                    $global:telemetryBuffer.Clear()
                    $json = @{
                        connected = ($global:serialPort -ne $null -and $global:serialPort.IsOpen)
                        lines = $telemetryLines
                    } | ConvertTo-Json -Compress
                    & $sendJsonResponse $json
                }
                # ================= STATIC FILE SERVING =================
                else {
                    if ($path -eq "/" -or $path -eq "/index.html") {
                        $path = "/Robot_2_Dashboard.html"
                    }

                    $filePath = [System.IO.Path]::Combine($RootPath, $path.TrimStart("/").Replace("/", [System.IO.Path]::DirectorySeparatorChar))
                    
                    if ([System.IO.File]::Exists($filePath)) {
                        $ext = [System.IO.Path]::GetExtension($filePath).ToLower()
                        $contentType = if ($mimeTypes.ContainsKey($ext)) { $mimeTypes[$ext] } else { "application/octet-stream" }
                        $bytes = [System.IO.File]::ReadAllBytes($filePath)
                        
                        $crlf = "`r`n"
                        $header = "HTTP/1.1 200 OK" + $crlf +
                                  "Content-Type: " + $contentType + $crlf +
                                  "Content-Length: " + $bytes.Length + $crlf +
                                  "Access-Control-Allow-Origin: *" + $crlf +
                                  "Connection: close" + $crlf + $crlf
                        $headerBytes = [System.Text.Encoding]::ASCII.GetBytes($header)
                        $stream.Write($headerBytes, 0, $headerBytes.Length)
                        $stream.Write($bytes, 0, $bytes.Length)
                    } else {
                        $msg = "404 Not Found: $path"
                        $msgBytes = [System.Text.Encoding]::UTF8.GetBytes($msg)
                        $crlf = "`r`n"
                        $header = "HTTP/1.1 404 Not Found" + $crlf +
                                  "Content-Type: text/plain" + $crlf +
                                  "Content-Length: " + $msgBytes.Length + $crlf +
                                  "Connection: close" + $crlf + $crlf
                        $headerBytes = [System.Text.Encoding]::ASCII.GetBytes($header)
                        $stream.Write($headerBytes, 0, $headerBytes.Length)
                        $stream.Write($msgBytes, 0, $msgBytes.Length)
                    }
                }
            }
        }
        $stream.Close()
        $client.Close()
    } catch {
        # Continue loop on connection drop
    }
}
