# RAG Backend Setup & Integration Guide

This guide helps you set up and integrate the RAG (Retrieval-Augmented Generation) backend with the AI Robotics Textbook.

## Overview

The RAG backend provides intelligent search capabilities for the textbook by:
1. Ingesting documentation content into a vector database (Qdrant)
2. Embedding text using OpenAI's embedding model
3. Retrieving relevant context for user queries
4. Generating answers using OpenAI's LLM

## Prerequisites

- Python 3.9+
- OpenAI API key (for embeddings and LLM)
- Qdrant instance (Cloud or local)
- FastAPI for running the backend server

## Installation

### 1. Set Up Python Environment

```bash
cd AI_Robotics_book/rag_backend
python -m venv .venv

# On Windows
.venv\Scripts\activate

# On macOS/Linux
source .venv/bin/activate
```

### 2. Install Dependencies

```bash
pip install -r requirements.txt
```

### 3. Configure Environment Variables

Create a `.env` file in the `rag_backend` directory:

```env
# OpenAI Configuration
OPENAI_API_KEY=your_openai_api_key_here
OPENAI_EMBEDDING_MODEL=text-embedding-3-small
OPENAI_LLM_MODEL=gpt-4-turbo-preview

# Qdrant Configuration (Cloud)
QDRANT_URL=https://your-instance.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key_here

# OR for Local Qdrant
# QDRANT_PATH=./qdrant_storage

# Authentication Configuration
# Generate a secure random SECRET key (min 32 characters)
SECRET=your_random_secret_key_here_min_32_chars

# Optional: OAuth Configuration
# Get these from Google Cloud Console: https://console.cloud.google.com/
GOOGLE_OAUTH_CLIENT_ID=your_google_client_id_here
GOOGLE_OAUTH_CLIENT_SECRET=your_google_client_secret_here

# Get these from GitHub Developer Settings: https://github.com/settings/developers
GITHUB_OAUTH_CLIENT_ID=your_github_client_id_here
GITHUB_OAUTH_CLIENT_SECRET=your_github_client_secret_here

# Local Server Configuration
RAG_SERVER_HOST=0.0.0.0
RAG_SERVER_PORT=8001

# Database Configuration (optional)
DATABASE_URL=sqlite:///./rag_backend.db
```

**Generating a Secure SECRET:**
```bash
# On Windows PowerShell
-join ([char[]]'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789' | Get-Random -Count 32)

# On macOS/Linux
openssl rand -base64 32
```

## Running the RAG Backend

### Step 1: Ingest Documentation

```bash
python ingest.py
```

This script will:
- Read all markdown files from `../docs/docs/`
- Split content into chunks
- Generate embeddings
- Upload to Qdrant

**Output**: You should see progress messages indicating successful ingestion of all chapters.

### Step 2: Initialize Authentication Database (Optional)

If you want to enable user authentication:

```bash
python -c "from database import create_db_and_tables; import asyncio; asyncio.run(create_db_and_tables())"
```

This creates the user database tables for authentication.

### Step 3: Start the Server

```bash
python server.py
```

The API will be available at `http://localhost:8001`

## Authentication System

The RAG backend includes built-in user authentication with JWT and OAuth support.

### Authentication Endpoints

```
POST   /auth/jwt/login              - Login with email/password
POST   /auth/jwt/logout             - Logout (invalidates token)
POST   /auth/register                - Create new user account
GET    /auth/verify                  - Email verification
POST   /auth/forgot-password         - Request password reset
GET    /users/me                     - Get current user info
GET    /auth/google/authorize        - Google OAuth login
GET    /auth/github/authorize        - GitHub OAuth login
```

### Using Authentication in Frontend

The Docusaurus frontend includes:
- **Paywall System**: Shows login prompt after 40% scroll (like Medium)
- **Auth Context**: Global user state management
- **Protected Endpoints**: Automatically include credentials

Example Protected Query:
```javascript
fetch('http://localhost:8001/users/me', {
  credentials: 'include',  // Sends auth cookie
});
```

### OAuth Setup

#### Google OAuth

1. Go to [Google Cloud Console](https://console.cloud.google.com/)
2. Create a new project
3. Enable Google+ API
4. Create OAuth 2.0 Credentials (Consent Screen → Credentials)
5. Add authorized redirect URIs:
   - `http://localhost:8001/auth/google/callback`
   - `https://yourdomain.com/auth/google/callback`
6. Copy Client ID and Client Secret to `.env`

#### GitHub OAuth

1. Go to [GitHub Developer Settings](https://github.com/settings/developers)
2. Click "New OAuth App"
3. Set Authorization callback URL:
   - `http://localhost:8001/auth/github/callback`
4. Copy Client ID and Client Secret to `.env`

## API Endpoints

### POST `/query`

Send a natural language query and receive an intelligent answer with sources.

**Request:**
```json
{
  "query": "How do I create a ROS 2 node?"
}
```

**Response:**
```json
{
  "answer": "To create a ROS 2 node in Python...",
  "sources": [
    {
      "source": "03-python-rclpy-integration.md",
      "text": "Relevant excerpt from the document...",
      "score": 0.95
    }
  ]
}
```

## Integration with Docusaurus

The SearchBar component automatically connects to the RAG backend when configured.

### Local Development

1. Ensure the RAG backend is running on `http://localhost:8001`
2. Start Docusaurus:
   ```bash
   cd docs
   npm start
   ```
3. The search component should be available on the homepage

### Production Deployment

Set the RAG API URL in `docusaurus.config.js`:

```javascript
customFields: {
  ragApiUrl: process.env.RAG_API_URL || 'http://localhost:8001',
}
```

Then deploy with environment variables:

```bash
RAG_API_URL=https://your-rag-backend.com npm run build
```

## Troubleshooting

### API Connection Issues

**Problem**: SearchBar shows "Error: API Error: 503"

**Solution**: 
- Ensure RAG backend is running
- Check that `ragApiUrl` in docusaurus.config.js is correct
- Verify CORS is enabled (it is by default in server.py)

### Ingestion Errors

**Problem**: "ModuleNotFoundError: No module named 'qdrant_client'"

**Solution**:
```bash
pip install --upgrade qdrant-client
```

### Empty Search Results

**Problem**: Query returns no sources

**Solution**:
1. Verify documents were ingested: Check Qdrant dashboard
2. Try a more specific query
3. Re-run `ingest.py` to update the database

## Advanced Configuration

### Custom Text Chunking

Edit `ingest.py` to change chunk size and overlap:

```python
from langchain.text_splitter import RecursiveCharacterTextSplitter

splitter = RecursiveCharacterTextSplitter(
    chunk_size=1000,      # Adjust chunk size
    chunk_overlap=100,    # Adjust overlap
    separators=["\n\n", "\n", " ", ""]
)
```

### Using Different Embedding Models

```env
# In .env
OPENAI_EMBEDDING_MODEL=text-embedding-3-large
```

### Rate Limiting & Authentication

For production, add authentication to `server.py`:

```python
from fastapi.security import HTTPBearer

security = HTTPBearer()

@app.post("/query")
async def query_endpoint(req: QueryRequest, credentials = Depends(security)):
    # Verify credentials before processing
    return rag_backend.search_and_generate(req.query)
```

## Performance Tips

1. **Use Smaller Embedding Model**: `text-embedding-3-small` is faster than `3-large`
2. **Enable Caching**: Cache frequent queries in Docusaurus
3. **Batch Processing**: For large documents, use batch embedding APIs
4. **Local Qdrant**: Use local Qdrant instance for faster retrieval in development

## Next Steps

1. ✅ Set up and test the RAG backend locally
2. ✅ Verify search works in the textbook
3. 📦 Consider deploying to a cloud service (Heroku, AWS, GCP)
4. 📊 Monitor query performance and relevance
5. 🔄 Periodically update embeddings as content changes

## Support

For issues or questions:
- Check Qdrant documentation: https://qdrant.tech/documentation/
- OpenAI API docs: https://platform.openai.com/docs/
- FastAPI docs: https://fastapi.tiangolo.com/

## Resources

- [Qdrant Cloud](https://qdrant.to/cloud)
- [OpenAI API Keys](https://platform.openai.com/account/api-keys)
- [RAG Best Practices](https://docs.langchain.com/docs/guides/docs_loaders/)
