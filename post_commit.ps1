$repoPath = "C:\Users\kccistc\Documents\GitHub\VGA_code"  # 로컬 레포지토리 경로
$gitExe = "C:\Program Files\Git\bin\git.exe"   # Git 실행 파일 경로
$vivadoPath = "C:\Xilinx\Vivado\2020.2\bin\vivado.bat"  # Vivado 실행 경로
$vivadoWorkspace = "C:\VGA"
$gitWorkspace = "$repoPath\workspace"
# 오늘 날짜 가져오기 (yyyy-MM-dd 형식)
$today = Get-Date -Format "yyyy-MM-dd"

# Vivado 실행
Write-Host "Vivado 2020.2 Start!!"
Start-Process -FilePath $vivadoPath -NoNewWindow -Wait

Write-Host "Vivado Closed. Checking for modified projects..."

# 수정된 날짜가 오늘인 폴더 찾기
$modifiedProjects = Get-ChildItem -Path $vivadoWorkspace -Directory #| Where-Object {
#     $_.LastWriteTime.Date -eq (Get-Date $today).Date
# }

foreach ($project in $modifiedProjects) {
    $projectName = $project.Name
    $srcPath = "$vivadoWorkspace\$project\$project.srcs"
    $dstPath = "$gitWorkspace\$projectName\$projectName.srcs"

    if (Test-Path $srcPath) {
        Write-Host "Detected Changed Project: $projectName"

        # 기존에 대상 폴더가 있다면 삭제 (완전한 덮어쓰기를 위해)
        if (Test-Path $dstPath) {
            Write-Host "Deleting old version of: $dstPath"
            Remove-Item -Path $dstPath -Recurse -Force
        }

        # 새로 복사
        Copy-Item -Path $srcPath -Destination $dstPath -Recurse -Force
    }
}

# 만약 수정된 프로젝트가 없으면 메시지 출력
if ($modifiedProjects.Count -eq 0) {
    Write-Host "Nothing has changed"
}

Write-Host "Git Auto Commit & Push Start!!"

# Git 커밋 및 푸시 수행
Set-Location -Path $repoPath

# 현재 시간 포맷 (예: 2025-03-06 15:30)
$currentTime = Get-Date -Format "yyyy-MM-dd HH:mm"
$commitMessage = "Auto commit for modified projects on $currentTime"

& $gitExe add .
& $gitExe commit -m $commitMessage
& $gitExe push origin main

Write-Host "Git Auto Commit & Push done."

# 3초 대기 후 PowerShell 창 종료
Start-Sleep -Seconds 3
Exit
