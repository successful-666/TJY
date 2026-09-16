# Add a UTF-8 BOM to hand-written source files.
#
# Why: ARM Compiler 5 assumes the local code page (GBK on Chinese Windows)
# when a source file has no BOM. UTF-8 encoded Chinese comments/strings then
# get mis-decoded, which can swallow a quote or a "*/" and break the build.
# With a BOM present armcc parses the file as UTF-8 correctly.
#
# Run this after editing files with tools that rewrite them without a BOM.

$root = "D:\Code\STM32\TJY\Car_Robot\code"
$dirs = @("App", "Service", "Device", "BSP")
$fixed = 0

foreach ($d in $dirs) {
    $path = Join-Path $root $d
    if (-not (Test-Path $path)) { continue }

    Get-ChildItem $path -File -Include *.c, *.h -Recurse | ForEach-Object {
        $bytes = [System.IO.File]::ReadAllBytes($_.FullName)
        if ($bytes.Length -ge 3 -and $bytes[0] -eq 0xEF -and
            $bytes[1] -eq 0xBB -and $bytes[2] -eq 0xBF) {
            return
        }

        $text = [System.IO.File]::ReadAllText($_.FullName,
                    [System.Text.UTF8Encoding]::new($false))
        [System.IO.File]::WriteAllText($_.FullName, $text,
                    [System.Text.UTF8Encoding]::new($true))
        Write-Output ("BOM added: " + $_.Name)
        $fixed++
    }
}

Write-Output ("Done. $fixed file(s) updated.")
