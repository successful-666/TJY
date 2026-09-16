<#
  主控板以太网 bring-up 自检脚本

  用法（普通权限即可）：
    powershell -ExecutionPolicy Bypass -File "D:\Code\STM32\TJY\Car_Robot\AI工作层\eth_test.ps1"

  可选参数：
    -BoardIp 192.168.1.2   板子 IP
    -Port    5000          TCP 端口
    -PingCount 4           ping 次数

  脚本只做「检查 + 测试」，不会修改你电脑的网络配置。
  如果第 2 步提示缺少 192.168.1.x 地址，请按提示用管理员身份执行给出的命令。
#>
param(
  [string]$BoardIp   = "192.168.1.2",
  [int]$Port         = 5000,
  [int]$PingCount    = 4
)

function Write-Step([string]$text) {
  Write-Host ""
  Write-Host "=== $text ===" -ForegroundColor Cyan
}

$failCount = 0

# ---------------------------------------------------------------------------
Write-Step "1. 本机 IPv4 地址"
Get-NetIPAddress -AddressFamily IPv4 |
  Where-Object { $_.IPAddress -ne '127.0.0.1' } |
  Select-Object InterfaceAlias, IPAddress, PrefixLength |
  Format-Table -AutoSize | Out-String | Write-Host

# ---------------------------------------------------------------------------
Write-Step "2. 检查本机是否处于 192.168.1.0/24 网段"
$local = Get-NetIPAddress -AddressFamily IPv4 -ErrorAction SilentlyContinue |
         Where-Object { $_.IPAddress -like "192.168.1.*" }

if ($local) {
  Write-Host "OK - 本机地址 $($local.IPAddress -join ', ')" -ForegroundColor Green
} else {
  $failCount++
  Write-Host "缺少 192.168.1.x 地址，板子与你不在同一网段。" -ForegroundColor Red
  Write-Host "请用「管理员身份」打开 PowerShell，执行下面这条命令给网卡加一个副 IP：" -ForegroundColor Yellow
  Write-Host ""
  Write-Host '    New-NetIPAddress -InterfaceAlias "以太网" -IPAddress 192.168.1.100 -PrefixLength 24' -ForegroundColor White
  Write-Host ""
  Write-Host "（如果网卡名字不是「以太网」，把名字换成上面第 1 步里列出的那一个）"
  Write-Host "不再需要时用下面这条命令移除："
  Write-Host '    Remove-NetIPAddress -InterfaceAlias "以太网" -IPAddress 192.168.1.100 -Confirm:$false' -ForegroundColor DarkGray
}

# ---------------------------------------------------------------------------
Write-Step "3. ping $BoardIp （$PingCount 次）"
$ping = Test-Connection -ComputerName $BoardIp -Count $PingCount -ErrorAction SilentlyContinue
if ($ping) {
  $avg = ($ping | Measure-Object -Property ResponseTime -Average).Average
  Write-Host "OK - 收到 $($ping.Count) 个回应，平均 $([math]::Round($avg,1)) ms" -ForegroundColor Green
} else {
  $failCount++
  Write-Host "FAIL - 没有回应" -ForegroundColor Red
  Write-Host "按这个顺序查：" -ForegroundColor Yellow
  Write-Host "  1) RJ45 链路灯亮不亮？不亮 -> 网线 / PHY 供电 / PB14 复位 / 25MHz 晶振"
  Write-Host "  2) 板子真的烧进去了吗？重新烧一次 Car_Robot.hex"
  Write-Host "  3) PC 防火墙是否拦截 ICMP"
}

# ---------------------------------------------------------------------------
Write-Step "4. TCP 连接 $BoardIp`:$Port 并验证回显"
$payload = "HELLO-CAR-ROBOT $(Get-Date -Format 'HH:mm:ss')"
try {
  $client = New-Object System.Net.Sockets.TcpClient
  $iar = $client.BeginConnect($BoardIp, $Port, $null, $null)
  if (-not $iar.AsyncWaitHandle.WaitOne(3000)) {
    $client.Close()
    throw "连接超时（3 秒）"
  }
  $client.EndConnect($iar)
  Write-Host "已建立 TCP 连接" -ForegroundColor Green

  $stream = $client.GetStream()
  $bytes  = [System.Text.Encoding]::ASCII.GetBytes($payload)
  $stream.Write($bytes, 0, $bytes.Length)
  $stream.Flush()
  Write-Host "发送: $payload"

  $stream.ReadTimeout = 3000
  $buf = New-Object byte[] 256
  $n = $stream.Read($buf, 0, $buf.Length)
  $echo = [System.Text.Encoding]::ASCII.GetString($buf, 0, $n)
  Write-Host "收到: $echo"

  if ($echo.Trim() -eq $payload.Trim()) {
    Write-Host "OK - 回显一致，TCP 收发链路正常" -ForegroundColor Green
  } else {
    $failCount++
    Write-Host "FAIL - 回显内容不一致" -ForegroundColor Red
  }
  $client.Close()
}
catch {
  $failCount++
  Write-Host "FAIL - $($_.Exception.Message)" -ForegroundColor Red
  Write-Host "可能原因：端口号不对、服务线程没起来、或者 ping 就没通" -ForegroundColor Yellow
}

# ---------------------------------------------------------------------------
Write-Step "结果"
if ($failCount -eq 0) {
  Write-Host "全部通过，以太网已经跑通，可以开始做 CAN / 485 协议了。" -ForegroundColor Green
} else {
  Write-Host "有 $failCount 项没通过，把上面的完整输出发给我。" -ForegroundColor Yellow
}
