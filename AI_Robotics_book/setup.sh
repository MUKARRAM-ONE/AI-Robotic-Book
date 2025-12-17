#!/bin/bash

# Quick Start Setup Script for AI Robotics Textbook
# Run this script to set up everything automatically

set -e  # Exit on any error

echo "================================"
echo "AI Robotics Textbook - Quick Setup"
echo "================================"
echo ""

# Check for Node.js
echo "Checking for Node.js..."
if ! command -v npm &> /dev/null; then
    echo "ERROR: Node.js and npm are required!"
    echo "Download from: https://nodejs.org/"
    exit 1
fi
echo "✓ Node.js found"

# Check for Python
echo "Checking for Python..."
if ! command -v python3 &> /dev/null; then
    echo "ERROR: Python 3.9+ is required!"
    echo "Download from: https://www.python.org/"
    exit 1
fi
echo "✓ Python found"
echo ""

# Setup Docusaurus
echo "Setting up Docusaurus textbook..."
cd docs
npm install
echo "✓ Docusaurus setup complete"
echo ""

# Setup RAG Backend (optional)
echo "Setting up RAG Backend (optional)..."
read -p "Set up RAG backend for AI-powered search? (y/n) " -n 1 -r
echo ""

if [[ $REPLY =~ ^[Yy]$ ]]; then
    cd ../rag_backend
    
    # Create virtual environment
    if [ ! -d ".venv" ]; then
        echo "Creating Python virtual environment..."
        python3 -m venv .venv
    fi
    
    # Activate virtual environment
    echo "Activating virtual environment..."
    source .venv/bin/activate
    
    # Install dependencies
    echo "Installing Python dependencies..."
    pip install -r requirements.txt
    
    # Check for .env file
    if [ ! -f ".env" ]; then
        echo "Creating .env file from template..."
        cp .env.example .env
        echo ""
        echo "⚠️  IMPORTANT: Edit .env with your credentials:"
        echo "   - OPENAI_API_KEY: Get from https://platform.openai.com/account/api-keys"
        echo "   - QDRANT_URL & QDRANT_API_KEY: Get from https://qdrant.tech/"
        echo ""
        
        read -p "Open .env in editor now? (y/n) " -n 1 -r
        echo ""
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            ${EDITOR:-nano} ".env"
        fi
    fi
    
    # Ask to ingest documents
    read -p "Ingest documentation into vector database? (This requires configured .env) (y/n) " -n 1 -r
    echo ""
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "Ingesting documents..."
        python ingest.py
        echo "✓ Documents ingested successfully"
    fi
    
    cd ../docs
    echo "✓ RAG Backend setup complete"
else
    echo "Skipping RAG Backend setup"
fi

echo ""
echo "================================"
echo "Setup Complete!"
echo "================================"
echo ""

echo "Next steps:"
echo "1. Start the textbook:"
echo "   cd docs"
echo "   npm start"
echo ""

if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "2. In a new terminal, start the RAG backend:"
    echo "   cd rag_backend"
    echo "   source .venv/bin/activate"
    echo "   python server.py"
    echo ""
    echo "3. Open your browser to http://localhost:3000"
    echo "   The search bar will automatically connect to the RAG backend"
else
    echo "2. Open your browser to http://localhost:3000"
fi

echo ""
echo "For more information, see COMPREHENSIVE_README.md"
echo ""
