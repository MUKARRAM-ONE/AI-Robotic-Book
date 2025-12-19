import os
from typing import Optional, AsyncGenerator

from fastapi import Depends, Request
from fastapi_users import FastAPIUsers
from fastapi_users.authentication import (
    AuthenticationBackend,
    BearerTransport,
    JWTStrategy,
)
from fastapi_users.manager import BaseUserManager, IntegerIDMixin
from fastapi_users.exceptions import InvalidPasswordException
from fastapi_users.password import PasswordHelper
from passlib.context import CryptContext

from database import User, get_user_db

SECRET = os.getenv("SECRET", "SECRET")


def get_jwt_strategy() -> JWTStrategy:
    return JWTStrategy(secret=SECRET, lifetime_seconds=3600)


# Use Bearer token transport so frontend can send Authorization header
# and receive {"access_token": "..."} from /auth/jwt/login
bearer_transport = BearerTransport(tokenUrl="auth/jwt/login")

auth_backend = AuthenticationBackend(
    name="jwt",
    transport=bearer_transport,
    get_strategy=get_jwt_strategy,
)

fastapi_users = FastAPIUsers[User, int](get_user_db, [auth_backend])

google_oauth_client_id = os.getenv("GOOGLE_OAUTH_CLIENT_ID")
google_oauth_client_secret = os.getenv("GOOGLE_OAUTH_CLIENT_SECRET")
github_oauth_client_id = os.getenv("GITHUB_OAUTH_CLIENT_ID")
github_oauth_client_secret = os.getenv("GITHUB_OAUTH_CLIENT_SECRET")

# Proper UserManager for FastAPI Users v12.x
class UserManager(IntegerIDMixin, BaseUserManager[User, int]):
    reset_password_token_secret = SECRET
    verification_token_secret = SECRET

    def __init__(self, user_db):
        super().__init__(user_db)
        # Allow passwords longer than bcrypt's 72 byte limit by hashing with
        # bcrypt_sha256 first, while still accepting existing bcrypt hashes.
        self.password_helper = PasswordHelper(
            CryptContext(
                schemes=["bcrypt_sha256", "bcrypt"],
                deprecated="auto",
                bcrypt__rounds=12,
                bcrypt_sha256__default_rounds=12,
            )
        )

    async def validate_password(self, password: str, user: User) -> None:
        # Enforce a hard limit to avoid passlib bcrypt overflow errors.
        if len(password.encode("utf-8")) > 256:
            raise InvalidPasswordException("Password must be at most 256 characters.")

    async def authenticate(self, credentials):
        try:
            return await super().authenticate(credentials)
        except ValueError as exc:
            # Treat passlib length errors as invalid credentials instead of 500.
            if "72 bytes" in str(exc):
                return None
            raise

    async def on_after_register(
        self, user: User, request: Optional[Request] = None
    ) -> None:
        # Optional: add logging/analytics here
        pass


async def get_user_manager(
    user_db=Depends(get_user_db),
) -> AsyncGenerator["UserManager", None]:
    yield UserManager(user_db)


# Recreate FastAPIUsers with the correct manager dependency
fastapi_users = FastAPIUsers[User, int](get_user_manager, [auth_backend])
