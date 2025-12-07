import os
from dotenv import load_dotenv
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
import openai
from qdrant_client import QdrantClient

load_dotenv()

OPENAI_API_KEY = os.getenv('OPENAI_API_KEY')
QDRANT_URL = os.getenv('QDRANT_URL')
QDRANT_API_KEY = os.getenv('QDRANT_API_KEY')
COLLECTION = os.getenv('QDRANT_COLLECTION', 'ai_robotics_docs')
CHAT_MODEL = os.getenv('CHAT_MODEL', 'gpt-3.5-turbo')
TOP_K = int(os.getenv('TOP_K', '4'))

openai.api_key = OPENAI_API_KEY
client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY, prefer_grpc=False)

app = FastAPI(title='AI Robotics RAG Backend')

class QueryRequest(BaseModel):
    query: str

@app.post('/search')
def search(req: QueryRequest):
    q = req.query
    if not q:
        raise HTTPException(status_code=400, detail='Query required')

    # 1) create embedding for query
    emb = openai.Embedding.create(model=os.getenv('EMBEDDING_MODEL', 'text-embedding-3-small'), input=[q])['data'][0]['embedding']

    # 2) perform vector search
    try:
        hits = client.search(collection_name=COLLECTION, query_vector=emb, limit=TOP_K)
    except Exception as e:
        raise HTTPException(status_code=500, detail=f'Error querying Qdrant: {e}')

    context_parts = []
    for h in hits:
        payload = h.payload or {}
        text = payload.get('text') or payload.get('content') or str(payload)
        src = payload.get('source', '')
        context_parts.append(f"Source: {src}\n{text}\n---\n")

    context = '\n'.join(context_parts)

    # 3) call OpenAI chat completion with context
    messages = [
        {"role": "system", "content": "You are an assistant that answers questions based on provided documentation context."},
        {"role": "user", "content": f"Context:\n{context}\nQuestion: {q}"}
    ]

    try:
        resp = openai.ChatCompletion.create(model=CHAT_MODEL, messages=messages, max_tokens=512, temperature=0.1)
        answer = resp['choices'][0]['message']['content'].strip()
    except Exception as e:
        raise HTTPException(status_code=500, detail=f'LLM error: {e}')

    return {"query": q, "answer": answer, "sources": [h.payload for h in hits]}

if __name__ == '__main__':
    import uvicorn
    uvicorn.run('server:app', host='0.0.0.0', port=8000, reload=True)
