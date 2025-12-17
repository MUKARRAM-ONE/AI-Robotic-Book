from fastapi import FastAPI, HTTPException, Depends
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import os
from dotenv import load_dotenv
from rag_backend import RAGBackend
from auth import auth_backend, fastapi_users, google_oauth_client_id, google_oauth_client_secret, github_oauth_client_id, github_oauth_client_secret, SECRET
from database import create_db_and_tables, User
from httpx_oauth.clients.google import GoogleOAuth2
from httpx_oauth.clients.github import GitHubOAuth2
from fastapi_users import schemas as fas


class UserRead(fas.BaseUser[int]):
    pass


class UserCreate(fas.BaseUserCreate):
    pass


class UserUpdate(fas.BaseUserUpdate):
    pass

load_dotenv()

# Define the request model
class QueryRequest(BaseModel):
    query: str

# Initialize FastAPI app
app = FastAPI(title='AI Robotics RAG Backend')

# Add CORS middleware to allow requests from the frontend
# CORS: use explicit origins when sending credentials
frontend_origins = os.getenv(
    "FRONTEND_ORIGINS",
    "http://localhost:3000,http://127.0.0.1:3000",
)
allowed = [o.strip() for o in frontend_origins.split(",") if o.strip()]

app.add_middleware(
    CORSMiddleware,
    allow_origins=allowed,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize RAG backend
try:
    rag = RAGBackend()
    print("✅ RAG Backend initialized successfully")
except Exception as e:
    print(f"⚠️  Warning: Could not initialize RAG backend: {e}")
    rag = None

google_oauth_client = GoogleOAuth2(google_oauth_client_id, google_oauth_client_secret)
github_oauth_client = GitHubOAuth2(github_oauth_client_id, github_oauth_client_secret)


app.include_router(
    fastapi_users.get_auth_router(auth_backend), prefix="/auth/jwt", tags=["auth"]
)
app.include_router(
    fastapi_users.get_register_router(UserRead, UserCreate),
    prefix="/auth",
    tags=["auth"],
)
app.include_router(
    fastapi_users.get_reset_password_router(),
    prefix="/auth",
    tags=["auth"],
)
app.include_router(
    fastapi_users.get_verify_router(UserRead),
    prefix="/auth",
    tags=["auth"],
)
app.include_router(
    fastapi_users.get_users_router(UserRead, UserUpdate),
    prefix="/users",
    tags=["users"],
)
app.include_router(
    fastapi_users.get_oauth_router(google_oauth_client, auth_backend, SECRET, is_verified_by_default=True),
    prefix="/auth/google",
    tags=["auth"],
)
app.include_router(
    fastapi_users.get_oauth_router(github_oauth_client, auth_backend, SECRET, is_verified_by_default=True),
    prefix="/auth/github",
    tags=["auth"],
)


@app.on_event("startup")
async def on_startup():
    # Not needed if you setup a migration system like Alembic
    await create_db_and_tables()


@app.post("/query")
async def query_endpoint(req: QueryRequest, user: User = Depends(fastapi_users.current_user(active=True, optional=False))):
    """
    Receives a query and returns AI-generated answer with sources.
    """
    print(f"Query endpoint called by user: {user.email if user else 'Anonymous'}")
    
    if not rag:
        raise HTTPException(
            status_code=503,
            detail="RAG backend not initialized. Please check your .env configuration."
        )

    try:
        result = rag.search_and_generate(req.query)
        print(f"Query result: {result}")
        return result
    except Exception as e:
        print(f"Error processing query: {e}")
        import traceback
        traceback.print_exc()
        raise HTTPException(
            status_code=500,
            detail=f"Error processing query: {str(e)}"
        )


@app.get("/health")
async def health_check():
    """Health check endpoint"""
    status = "healthy" if rag else "degraded"
    return {
        "status": status,
        "message": "RAG Backend is running" if rag else "RAG Backend not initialized"
    }


if __name__ == "__main__":
    import uvicorn
    port = int(os.getenv("SERVER_PORT", "8001"))
    host = os.getenv("SERVER_HOST", "0.0.0.0")
    print(f"🚀 Starting server on {host}:{port}")
    uvicorn.run(app, host=host, port=port, log_level="info")