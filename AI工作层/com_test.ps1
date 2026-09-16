<#
  调试串口（USART1）命令行测试脚本

  用法:
    powershell -ExecutionPolicy Bypass -File com_test.ps1
    powershell -ExecutionPolicy Bypass -File com_test.ps1 -Port COM10 -Commands "?", "r 1"

  注意：
    - 运行前必须关闭串口助手，否则 COM10 会被占用
    - 故意不设置 DTR/RTS，避免某些 USB-TTL 上的自动复位电路被触发
#>
param(
  [string]   $Port     = "COM10",
  [int]      $Baud     = 115200,
  [string[]] $Commands = @("?"),
  [int]      $WaitMs   = 1500
)

$sp = New-Object System.IO.Ports.SerialPort $Port, $Baud, "None", 8, "One"
$sp.ReadTimeout  = 500
$sp.WriteTimeout = 1000

try {
  $sp.Open()
}
catch {
  Write-Host ("无法打开 " + $Port + ": " + $_.Exception.Message) -ForegroundColor Red
  Write-Host "（串口助手是不是还开着？）"
  exit 1
}

Start-Sleep -Milliseconds 300
$sp.DiscardInBuffer()
Write-Host ("已打开 " + $Port + " @ " + $Baud) -ForegroundColor Green

foreach ($cmd in $Commands) {
  Write-Host ""
  Write-Host ("--> 发送: [" + $cmd + "]") -ForegroundColor Cyan
  $sp.Write($cmd + "`n")

  Start-Sleep -Milliseconds $WaitMs

  $data = $sp.ReadExisting()
  if ([string]::IsNullOrWhiteSpace($data)) {
    Write-Host "<-- 没有任何回复" -ForegroundColor Yellow
  }
  else {
    Write-Host "<-- 收到:"
    foreach ($line in ($data -split "`r?`n")) {
      if ($line.Trim().Length -gt 0) {
        Write-Host ("    " + $line.TrimEnd())
      }
    }
  }
}

$sp.Close()
$sp.Dispose()
Write-Host ""
Write-Host "测试结束" -ForegroundColor Green
