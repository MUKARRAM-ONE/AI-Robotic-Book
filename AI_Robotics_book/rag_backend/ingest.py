import os
import glob
import uuid
from dotenv import load_dotenv
from tqdm import tqdm
import openai
from qdrant_client import QdrantClient
from qdrant_client.http import models

load_dotenv()

OPENAI_API_KEY = os.getenv('OPENAI_API_KEY')
QDRANT_URL = os.getenv('QDRANT_URL')
QDRANT_API_KEY = os.getenv('QDRANT_API_KEY')
COLLECTION = os.getenv('QDRANT_COLLECTION', 'ai_robotics_docs')
EMBED_MODEL = os.getenv('EMBEDDING_MODEL', 'text-embedding-3-small')

openai.api_key = OPENAI_API_KEY

def read_markdown_files(docs_path):
    files = glob.glob(os.path.join(docs_path, '*.md'))
    docs = []
    for path in files:
        with open(path, 'r', encoding='utf-8') as f:
            text = f.read()
        docs.append({'id': os.path.basename(path), 'content': text})
    return docs

def chunk_text(text, chunk_size=1000, overlap=200):
    # simple character-based chunking
    chunks = []
    start = 0
    while start < len(text):
        end = min(start + chunk_size, len(text))
        chunks.append(text[start:end])
        start += chunk_size - overlap
    return chunks

def create_collection_if_not_exists(client: QdrantClient, collection: str, vector_size: int):
    try:
        if collection not in [c.name for c in client.get_collections().collections]:
            client.recreate_collection(collection_name=collection, vectors_config=models.VectorParams(size=vector_size, distance=models.Distance.COSINE))
    except Exception:
        # fallback recreate
        client.recreate_collection(collection_name=collection, vectors_config=models.VectorParams(size=vector_size, distance=models.Distance.COSINE))

def embed_texts(texts):
    # use OpenAI embeddings
    resp = openai.Embedding.create(model=EMBED_MODEL, input=texts)
    return [e['embedding'] for e in resp['data']]

def ingest(docs_path: str = '../docs/docs'):
    print('Connecting to Qdrant...')
    client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY, prefer_grpc=False)

    docs = read_markdown_files(docs_path)
    items = []
    for doc in docs:
        chunks = chunk_text(doc['content'])
        for i, chunk in enumerate(chunks):
            items.append({'id': f"{doc['id']}_{i}", 'text': chunk, 'meta': {'source': doc['id'], 'chunk': i}})

    # compute embedding in batches
    texts = [it['text'] for it in items]
    print(f'Creating embeddings for {len(texts)} chunks...')
    batch_size = 32
    vectors = []
    for i in tqdm(range(0, len(texts), batch_size)):
        batch = texts[i:i+batch_size]
        vectors.extend(embed_texts(batch))

    vector_size = len(vectors[0])
    create_collection_if_not_exists(client, COLLECTION, vector_size)

    points = []
    for it, vec in zip(items, vectors):
        points.append(models.PointStruct(id=str(uuid.uuid4()), vector=vec, payload={'text': it['text'], **it['meta']}))

    print(f'Upserting {len(points)} points to Qdrant collection "{COLLECTION}"...')
    client.upsert(collection_name=COLLECTION, points=points)
    print('Ingestion complete.')

if __name__ == '__main__':
    ingest()
