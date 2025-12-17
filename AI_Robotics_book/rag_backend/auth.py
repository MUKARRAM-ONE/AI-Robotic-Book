import os
from typing import Optional, AsyncGenerator

from fastapi import Depends, Request
from fastapi_users import FastAPIUsers
from fastapi_users.authentication import (
    AuthenticationBackend,
    CookieTransport,
    JWTStrategy,
)
from fastapi_users.manager import BaseUserManager, IntegerIDMixin

from database import User, get_user_db

SECRET = os.getenv("SECRET", "SECRET")


def get_jwt_strategy() -> JWTStrategy:
    return JWTStrategy(secret=SECRET, lifetime_seconds=3600)


cookie_transport = CookieTransport(cookie_name="bonds", cookie_max_age=3600)

auth_backend = AuthenticationBackend(
    name="jwt",
    transport=cookie_transport,
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
