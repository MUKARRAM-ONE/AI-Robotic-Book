# 🚀 Quick Start - RAG Search Setup

Your documentation now has **AI-powered search** with RAG (Retrieval Augmented Generation)!

## What You Get

✅ **Floating chat removed** from home page  
✅ **Smart search bar** in top-right navbar (press `Ctrl+K`)  
✅ **RAG-powered answers** from your documentation  
✅ **FictionLab-inspired dark theme** with orange accents  
✅ **Tokenized document ingestion** to Qdrant vector database

---

## 🎯 3-Step Setup

### 1️⃣ Install & Configure

```bash
cd rag_backend

# Install dependencies
pip install -r requirements.txt

# Create .env file
cp .env.example .env
```

**Edit `.env` with your keys:**

```env
QDRANT_URL=http://localhost:6333
QDRANT_API_KEY=
OPENAI_API_KEY=sk-your-key-here
QDRANT_COLLECTION=ai_robotics_docs
```

### 2️⃣ Start Qdrant (Vector Database)

**Option A - Docker (Recommended):**
```bash
docker run -p 6333:6333 qdrant/qdrant
```

**Option B - Qdrant Cloud:**
1. Sign up at https://cloud.qdrant.io
2. Create a cluster
3. Use cluster URL in `.env`

### 3️⃣ Ingest & Run

```bash
# Tokenize and embed your documentation
python ingest.py

# Start RAG server
python server.py
```

In another terminal:
```bash
# Start docs site
cd ../docs
npm start
```

---

## ✨ Using the Search

1. **Open**: http://localhost:3000/AI-driven-development/
2. **Search**: Click search bar (top-right) or press `Ctrl+K`
3. **Ask**: Type any question about your robotics documentation
4. **Get Answer**: AI generates intelligent responses with sources!

---

## 🔧 What Happens Behind the Scenes

### During Ingestion (`python ingest.py`)

1. **Reads** all `.md` files from `../docs/docs/`
2. **Tokenizes** text into 512-token chunks (50 overlap)
3. **Embeds** using FastEmbed (BAAI/bge-small-en model)
4. **Stores** vectors in Qdrant with metadata

```
📄 01-intro-to-ros2.md
   ↓ Split into chunks
   ↓ Create embeddings
   ↓ Store in Qdrant
✅ 156 chunks indexed
```

### During Search

1. **User query** → Embedded with same model
2. **Vector search** → Find top-K similar chunks
3. **LLM generation** → OpenAI creates answer from context
4. **Return** → Answer + source documents

---

## 📁 File Structure

```
rag_backend/
├── .env              # Your configuration (create this!)
├── .env.example      # Template
├── requirements.txt  # Python dependencies
├── ingest.py         # Document tokenization & embedding
├── rag_backend.py    # RAG logic with OpenAI
├── server.py         # FastAPI server
├── setup_check.py    # Setup validation script
├── QUICKSTART.md     # This file
└── README.md         # Detailed documentation
```

---

## 🧪 Test Your Setup

### Check Setup
```bash
python setup_check.py
```

### Test RAG Endpoint
```bash
curl -X POST http://localhost:8001/query \
  -H "Content-Type: application/json" \
  -d '{"query":"What is ROS 2?"}'
```

### Health Check
```bash
curl http://localhost:8001/health
```

---

## ⚙️ Configuration Options

### Adjust Chunk Size

Edit `ingest.py`:
```python
def chunk_text(text, chunk_size=512, chunk_overlap=50):
    # Larger chunks = more context, fewer chunks
    # Smaller chunks = more precise, more chunks
```

### Change Number of Results

Edit `.env`:
```env
TOP_K=4  # Number of document chunks to retrieve
```

### Use Different LLM

Edit `.env`:
```env
CHAT_MODEL=gpt-4  # or gpt-3.5-turbo, gpt-4-turbo, etc.
```

---

## 🐛 Troubleshooting

| Problem | Solution |
|---------|----------|
| **"Could not connect to Qdrant"** | Start Qdrant: `docker run -p 6333:6333 qdrant/qdrant` |
| **"OpenAI API error"** | Check OPENAI_API_KEY in `.env` |
| **"No results found"** | Run `python ingest.py` first |
| **"Module not found"** | Run `pip install -r requirements.txt` |
| **Search bar not visible** | Rebuild docs: `cd ../docs && npm run build` |

---

## 🎨 Frontend Features

✅ **Dark theme** - FictionLab-inspired design  
✅ **Search bar** - Top-right navbar position  
✅ **Keyboard shortcut** - Press `Ctrl+K` / `Cmd+K`  
✅ **Dropdown results** - Appears below search  
✅ **Loading states** - Visual feedback during search  
✅ **Error handling** - Clear messages if backend is down  

---

## 🚀 Production Deployment

### Deploy RAG Backend

**Render / Railway / Fly.io:**
```bash
# Set environment variables
QDRANT_URL=https://your-qdrant-cluster.com
QDRANT_API_KEY=your-key
OPENAI_API_KEY=your-key
SERVER_PORT=8001
```

### Update Frontend

Edit `docs/docusaurus.config.js`:
```javascript
customFields: {
  ragApiUrl: 'https://your-rag-backend.com',
}
```

---

## 📚 Additional Resources

- **Qdrant Docs**: https://qdrant.tech/documentation/
- **OpenAI API**: https://platform.openai.com/docs
- **FastEmbed**: https://github.com/qdrant/fastembed
- **Docusaurus**: https://docusaurus.io

---

## 💡 Next Steps

1. ✅ **Test search** - Try different queries
2. 📝 **Add more docs** - Run `ingest.py` again after adding files
3. 🎨 **Customize theme** - Edit `docs/src/css/custom.css`
4. 🚀 **Deploy** - Put your docs and RAG backend online

---

## ❓ Need Help?

- Check [SETUP_GUIDE.md](SETUP_GUIDE.md) for detailed setup
- Review [README.md](README.md) for architecture details
- Open an issue on GitHub
- Check console logs for error messages

---

**Enjoy your AI-powered documentation! 🎉**
