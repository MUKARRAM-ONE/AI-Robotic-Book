import os
import sys
import json
from dotenv import load_dotenv, find_dotenv
import requests

load_dotenv()
env_path = find_dotenv(filename='.env', usecwd=True)
if env_path:
    load_dotenv(env_path, override=False)

QDRANT_URL = os.getenv('QDRANT_URL')
QDRANT_API_KEY = os.getenv('QDRANT_API_KEY')
COLLECTION = os.getenv('QDRANT_COLLECTION', 'ai_robotics_docs')

if not QDRANT_URL or not QDRANT_API_KEY:
    print('QDRANT_URL or QDRANT_API_KEY not set in environment. Please set them in .env')
    sys.exit(2)

def try_request(path):
    url = QDRANT_URL.rstrip('/') + path
    headers_list = [
        {'api-key': QDRANT_API_KEY},
        {'x-api-key': QDRANT_API_KEY},
        {'Authorization': f'ApiKey {QDRANT_API_KEY}'},
        {'Authorization': f'Bearer {QDRANT_API_KEY}'},
    ]
    for headers in headers_list:
        try:
            r = requests.get(url, headers=headers, timeout=10)
            print(f'Attempt {headers}: status={r.status_code}')
            try:
                print(json.dumps(r.json(), indent=2)[:1000])
            except Exception:
                print('Non-JSON response or empty')
            if r.status_code == 200:
                return r.json()
        except Exception as e:
            print('Request error with headers', headers, e)
    return None

print('Checking Qdrant collections...')
res = try_request('/collections')
if not res:
    print('Could not list collections. The API key or URL may be incorrect, or the cluster is unreachable.')
    sys.exit(1)

collections = res.get('collections') if isinstance(res, dict) else None
if not collections:
    print('No collections found in response. Raw response:', res)
    sys.exit(1)

print('Collections found:')
for c in collections:
    print(' -', c.get('name'))

if COLLECTION in [c.get('name') for c in collections]:
    print(f'Collection "{COLLECTION}" exists. Fetching info...')
    info = try_request(f'/collections/{COLLECTION}/info')
    if info:
        print('Collection info:')
        print(json.dumps(info, indent=2))
    else:
        print('Could not fetch collection info.')
else:
    print(f'Collection "{COLLECTION}" not found on the cluster.')
