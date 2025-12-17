# Quick Start Setup Script for AI Robotics Textbook
# Run this script to set up everything automatically

Write-Host "================================" -ForegroundColor Cyan
Write-Host "AI Robotics Textbook - Quick Setup" -ForegroundColor Cyan
Write-Host "================================" -ForegroundColor Cyan
Write-Host ""

# Check for Node.js
Write-Host "Checking for Node.js..." -ForegroundColor Yellow
if (-not (Get-Command npm -ErrorAction SilentlyContinue)) {
    Write-Host "ERROR: Node.js and npm are required!" -ForegroundColor Red
    Write-Host "Download from: https://nodejs.org/" -ForegroundColor Yellow
    exit 1
}
Write-Host "✓ Node.js found" -ForegroundColor Green

# Check for Python
Write-Host "Checking for Python..." -ForegroundColor Yellow
if (-not (Get-Command python -ErrorAction SilentlyContinue)) {
    Write-Host "ERROR: Python 3.9+ is required!" -ForegroundColor Red
    Write-Host "Download from: https://www.python.org/" -ForegroundColor Yellow
    exit 1
}
Write-Host "✓ Python found" -ForegroundColor Green
Write-Host ""

# Setup Docusaurus
Write-Host "Setting up Docusaurus textbook..." -ForegroundColor Yellow
cd docs
npm install
if ($LASTEXITCODE -ne 0) {
    Write-Host "ERROR: Failed to install npm dependencies" -ForegroundColor Red
    exit 1
}
Write-Host "✓ Docusaurus setup complete" -ForegroundColor Green
Write-Host ""

# Setup RAG Backend (optional)
Write-Host "Setting up RAG Backend (optional)..." -ForegroundColor Yellow
$setupRag = Read-Host "Set up RAG backend for AI-powered search? (y/n)"

if ($setupRag -eq "y" -or $setupRag -eq "Y") {
    cd ..\rag_backend
    
    # Create virtual environment
    if (-not (Test-Path ".venv")) {
        Write-Host "Creating Python virtual environment..." -ForegroundColor Yellow
        python -m venv .venv
    }
    
    # Activate virtual environment
    Write-Host "Activating virtual environment..." -ForegroundColor Yellow
    & ".\.venv\Scripts\Activate.ps1"
    
    # Install dependencies
    Write-Host "Installing Python dependencies..." -ForegroundColor Yellow
    pip install -r requirements.txt
    if ($LASTEXITCODE -ne 0) {
        Write-Host "ERROR: Failed to install Python dependencies" -ForegroundColor Red
        exit 1
    }
    
    # Check for .env file
    if (-not (Test-Path ".env")) {
        Write-Host "Creating .env file from template..." -ForegroundColor Yellow
        Copy-Item ".env.example" ".env"
        Write-Host ""
        Write-Host "⚠️  IMPORTANT: Edit .env with your credentials:" -ForegroundColor Yellow
        Write-Host "   - OPENAI_API_KEY: Get from https://platform.openai.com/account/api-keys" -ForegroundColor White
        Write-Host "   - QDRANT_URL & QDRANT_API_KEY: Get from https://qdrant.tech/" -ForegroundColor White
        Write-Host ""
        
        $editEnv = Read-Host "Open .env in editor now? (y/n)"
        if ($editEnv -eq "y" -or $editEnv -eq "Y") {
            notepad ".env"
        }
    }
    
    # Ask to ingest documents
    $ingestDocs = Read-Host "Ingest documentation into vector database? (This requires configured .env) (y/n)"
    if ($ingestDocs -eq "y" -or $ingestDocs -eq "Y") {
        Write-Host "Ingesting documents..." -ForegroundColor Yellow
        python ingest.py
        if ($LASTEXITCODE -eq 0) {
            Write-Host "✓ Documents ingested successfully" -ForegroundColor Green
        } else {
            Write-Host "⚠️  Document ingestion had issues (check .env configuration)" -ForegroundColor Yellow
        }
    }
    
    cd ..\docs
    Write-Host "✓ RAG Backend setup complete" -ForegroundColor Green
} else {
    Write-Host "Skipping RAG Backend setup" -ForegroundColor Yellow
}

Write-Host ""
Write-Host "================================" -ForegroundColor Cyan
Write-Host "Setup Complete!" -ForegroundColor Green
Write-Host "================================" -ForegroundColor Cyan
Write-Host ""

Write-Host "Next steps:" -ForegroundColor Yellow
Write-Host "1. Start the textbook:" -ForegroundColor White
Write-Host "   cd docs" -ForegroundColor Cyan
Write-Host "   npm start" -ForegroundColor Cyan
Write-Host ""

if ($setupRag -eq "y" -or $setupRag -eq "Y") {
    Write-Host "2. In a new terminal, start the RAG backend:" -ForegroundColor White
    Write-Host "   cd rag_backend" -ForegroundColor Cyan
    Write-Host "   .\.venv\Scripts\Activate.ps1" -ForegroundColor Cyan
    Write-Host "   python server.py" -ForegroundColor Cyan
    Write-Host ""
    Write-Host "3. Open your browser to http://localhost:3000" -ForegroundColor White
    Write-Host "   The search bar will automatically connect to the RAG backend" -ForegroundColor White
} else {
    Write-Host "2. Open your browser to http://localhost:3000" -ForegroundColor White
}

Write-Host ""
Write-Host "For more information, see COMPREHENSIVE_README.md" -ForegroundColor Yellow
Write-Host ""
