$ErrorActionPreference = 'Stop'

Write-Host "=========================================="
Write-Host " ARCHER VOICE SETUP: Downloading Piper TTS"
Write-Host "=========================================="

$piperDir = Join-Path $HOME "piper"
$modelsDir = Join-Path $HOME "piper-models"

# 1. Create directories
if (-not (Test-Path $piperDir)) { New-Item -ItemType Directory -Path $piperDir | Out-Null }
if (-not (Test-Path $modelsDir)) { New-Item -ItemType Directory -Path $modelsDir | Out-Null }

# 2. Download Piper binary for Windows
$piperUrl = "https://github.com/rhasspy/piper/releases/download/2023.11.14-2/piper_windows_amd64.zip"
$piperZip = Join-Path $piperDir "piper.zip"

if (-not (Test-Path (Join-Path $piperDir "piper.exe"))) {
    Write-Host "Downloading Piper..."
    Invoke-WebRequest -Uri $piperUrl -OutFile $piperZip
    Write-Host "Extracting Piper..."
    Expand-Archive -Path $piperZip -DestinationPath $piperDir -Force
    # The zip usually contains a folder piper/, move contents up
    $extractedFolder = Join-Path $piperDir "piper"
    if (Test-Path $extractedFolder) {
        Move-Item -Path "$extractedFolder\*" -Destination $piperDir -Force
        Remove-Item -Path $extractedFolder -Recurse -Force
    }
    Remove-Item -Path $piperZip -Force
} else {
    Write-Host "Piper binary already exists."
}

# 3. Download Voice Model (en_US-lessac-medium)
$modelUrl = "https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/lessac/medium/en_US-lessac-medium.onnx"
$jsonUrl = "https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/lessac/medium/en_US-lessac-medium.onnx.json"
$modelFile = Join-Path $modelsDir "en_US-lessac-medium.onnx"
$jsonFile = Join-Path $modelsDir "en_US-lessac-medium.onnx.json"

if (-not (Test-Path $modelFile)) {
    Write-Host "Downloading Voice Model..."
    Invoke-WebRequest -Uri $modelUrl -OutFile $modelFile
    Invoke-WebRequest -Uri $jsonUrl -OutFile $jsonFile
} else {
    Write-Host "Voice model already exists."
}

# 4. Try updating User PATH
$userPath = [Environment]::GetEnvironmentVariable("PATH", "User")
if ($userPath -notmatch [regex]::Escape($piperDir)) {
    Write-Host "Adding Piper to PATH..."
    [Environment]::SetEnvironmentVariable("PATH", $userPath + ";" + $piperDir, "User")
    Write-Host "Piper added to user PATH. You may need to restart your terminal for python to see it!"
}

Write-Host "=========================================="
Write-Host " SETUP COMPLETE! "
Write-Host "=========================================="
Write-Host "Please restart this terminal window if the pipeline still cannot find Piper."
