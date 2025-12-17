import os
import glob
import uuid
from dotenv import load_dotenv, find_dotenv
from tqdm import tqdm
from qdrant_client import QdrantClient
from qdrant_client.http import models
from langchain_text_splitters import RecursiveCharacterTextSplitter
from fastembed import TextEmbedding

# Load .env from the current folder or parent repository root if present
load_dotenv()  # load local .env if exists
env_path = find_dotenv(filename='.env', usecwd=True)
if env_path:
    load_dotenv(env_path, override=False)

QDRANT_URL = os.getenv('QDRANT_URL')
QDRANT_API_KEY = os.getenv('QDRANT_API_KEY')
COLLECTION = os.getenv('QDRANT_COLLECTION', 'ai_robotics_docs')
EMBED_MODEL = 'BAAI/bge-small-en'
VECTOR_SIZE = 384  # for BAAI/bge-small-en

def read_markdown_files(docs_path):
    files = glob.glob(os.path.join(docs_path, '*.md'))
    docs = []
    for path in files:
        with open(path, 'r', encoding='utf-8') as f:
            text = f.read()
        docs.append({'id': os.path.basename(path), 'content': text})
    return docs

def chunk_text(text, chunk_size=512, chunk_overlap=50):
    """Chunks text by characters."""
    text_splitter = RecursiveCharacterTextSplitter(
        chunk_size=chunk_size,
        chunk_overlap=chunk_overlap,
        length_function=len
    )
    chunks = text_splitter.split_text(text)
    return chunks

def create_collection_if_not_exists(client: QdrantClient, collection: str, vector_size: int):
    # Recreate collection every time to ensure consistency
    client.recreate_collection(
        collection_name=collection, 
        vectors_config=models.VectorParams(size=vector_size, distance=models.Distance.COSINE)
    )

def ingest(docs_path: str = '../docs/docs'):
    print('Loading embedding model... This may take a moment.')
    embedding_model = TextEmbedding(model_name=EMBED_MODEL)

    print('Connecting to Qdrant...')
    client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY, prefer_grpc=False)

    create_collection_if_not_exists(client, COLLECTION, VECTOR_SIZE)

    docs = read_markdown_files(docs_path)
    items = []
    for doc in docs:
        chunks = chunk_text(doc['content'])
        for i, chunk in enumerate(chunks):
            items.append({'id': f"{doc['id']}_{i}", 'text': chunk, 'meta': {'source': doc['id'], 'chunk': i}})

    # compute embedding in batches
    texts = [it['text'] for it in items]
    print(f'Creating embeddings for {len(texts)} chunks...')
    
    # Fastembed's embed method handles batching automatically
    vectors = embedding_model.embed(texts, batch_size=32)

    points = []
    for it, vec in zip(items, vectors):
        points.append(models.PointStruct(id=str(uuid.uuid4()), vector=vec, payload={'text': it['text'], **it['meta']}))

    print(f'Upserting {len(points)} points to Qdrant collection "{COLLECTION}"...')
    client.upsert(collection_name=COLLECTION, points=points)
    print('Ingestion complete.')

if __name__ == '__main__':
    ingest()
