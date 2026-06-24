# run_all_eval.ps1
# Chạy lần lượt eval.py trên nhiều map, lưu log từng map, không dừng khi 1 map lỗi.
# Cách dùng: mở PowerShell tại thư mục aco_ga_astar_mtipp rồi chạy:
#     .\run_all_eval.ps1

# ----- Cau hinh -----
$N = 10                 # so lan chay moi map (--n)
$Independent = $true    # $true => them --independent, $false => bo

# Danh sach config muon chay (them/bot tuy y)
$configs = @(
    "configs/scenario_square_400_lowrisk.yaml",
    "configs/scenario_triangle_300_lowrisk.yaml",
    "configs/scenario_custom_200x200.yaml",
    "configs/factory400_30.yaml",
    "configs/mixed500.yaml"
)
# --------------------

$logDir = "eval_logs"
New-Item -ItemType Directory -Force -Path $logDir | Out-Null

$overall = Get-Date
Write-Host "Bat dau chay $($configs.Count) map luc $overall" -ForegroundColor Cyan

foreach ($cfg in $configs) {
    $name = [System.IO.Path]::GetFileNameWithoutExtension($cfg)
    $log  = Join-Path $logDir "$name.log"

    Write-Host ""
    Write-Host ("=" * 70) -ForegroundColor Yellow
    Write-Host "MAP: $cfg" -ForegroundColor Yellow
    Write-Host ("=" * 70) -ForegroundColor Yellow

    $argsList = @("eval.py", "--config", $cfg, "--n", $N)
    if ($Independent) { $argsList += "--independent" }

    $start = Get-Date
    # Ghi log ra file dong thoi van hien tren man hinh
    python @argsList 2>&1 | Tee-Object -FilePath $log
    $elapsed = (Get-Date) - $start

    if ($LASTEXITCODE -eq 0) {
        Write-Host ("OK  $name  (mat {0:hh\:mm\:ss})" -f $elapsed) -ForegroundColor Green
    } else {
        Write-Host ("LOI $name  (exit=$LASTEXITCODE) - xem $log" -f $elapsed) -ForegroundColor Red
    }
}

$total = (Get-Date) - $overall
Write-Host ""
Write-Host ("HOAN TAT. Tong thoi gian: {0:hh\:mm\:ss}. Log nam trong: $logDir" -f $total) -ForegroundColor Cyan
