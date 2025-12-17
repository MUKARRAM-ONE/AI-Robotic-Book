#!/usr/bin/env python3
"""
Quick setup script for RAG backend
"""
import os
import sys

def check_env_file():
    """Check if .env file exists"""
    if not os.path.exists('.env'):
        print("❌ .env file not found!")
        print("\n📝 Creating .env from .env.example...")
        if os.path.exists('.env.example'):
            with open('.env.example', 'r') as src:
                content = src.read()
            with open('.env', 'w') as dst:
                dst.write(content)
            print("✅ .env file created!")
            print("\n⚠️  IMPORTANT: Edit .env and add your API keys:")
            print("   - QDRANT_URL")
            print("   - QDRANT_API_KEY (if using cloud)")
            print("   - OPENAI_API_KEY")
            return False
        else:
            print("❌ .env.example not found!")
            return False
    return True

def check_dependencies():
    """Check if required packages are installed"""
    required = [
        'qdrant_client',
        'fastapi',
        'uvicorn',
        'dotenv',
        'fastembed',
        'openai'
    ]
    
    missing = []
    for package in required:
        try:
            __import__(package.replace('-', '_'))
        except ImportError:
            missing.append(package)
    
    if missing:
        print(f"❌ Missing packages: {', '.join(missing)}")
        print("\n📦 Install with: pip install -r requirements.txt")
        return False
    return True

def test_qdrant_connection():
    """Test connection to Qdrant"""
    try:
        from dotenv import load_dotenv
        from qdrant_client import QdrantClient
        
        load_dotenv()
        url = os.getenv('QDRANT_URL')
        api_key = os.getenv('QDRANT_API_KEY')
        
        if not url:
            print("❌ QDRANT_URL not set in .env")
            return False
        
        client = QdrantClient(url=url, api_key=api_key)
        collections = client.get_collections()
        print(f"✅ Connected to Qdrant at {url}")
        return True
    except Exception as e:
        print(f"❌ Could not connect to Qdrant: {e}")
        print("\n💡 Make sure Qdrant is running:")
        print("   docker run -p 6333:6333 qdrant/qdrant")
        return False

def main():
    print("🚀 RAG Backend Setup Check\n")
    
    # Check .env
    env_ok = check_env_file()
    if not env_ok:
        sys.exit(1)
    
    print("✅ .env file exists\n")
    
    # Check dependencies
    deps_ok = check_dependencies()
    if not deps_ok:
        sys.exit(1)
    
    print("✅ All dependencies installed\n")
    
    # Test Qdrant
    qdrant_ok = test_qdrant_connection()
    if not qdrant_ok:
        sys.exit(1)
    
    print("\n🎉 Setup complete!\n")
    print("Next steps:")
    print("1. Run: python ingest.py    (to load docs into Qdrant)")
    print("2. Run: python server.py    (to start the RAG server)")
    print("\n📚 See QUICKSTART.md for detailed instructions")

if __name__ == '__main__':
    main()
