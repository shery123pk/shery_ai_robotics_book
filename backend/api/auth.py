"""
Authentication API endpoints - User signup, login, logout
"""

from datetime import datetime, timedelta
from typing import Optional
import asyncpg
import json
from fastapi import APIRouter, HTTPException, status, Depends
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from pydantic import BaseModel, EmailStr, Field
from jose import jwt
from passlib.context import CryptContext

from config import settings
from database.postgres import get_db_pool
from models.auth import UserCreate, UserLogin, UserResponse, TokenResponse

router = APIRouter(prefix="/api/auth")
security = HTTPBearer()
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")


# Helper functions
def verify_password(plain_password: str, hashed_password: str) -> bool:
    """Verify password against hash."""
    return pwd_context.verify(plain_password, hashed_password)


def get_password_hash(password: str) -> str:
    """Hash password."""
    return pwd_context.hash(password)


def create_access_token(data: dict, expires_delta: Optional[timedelta] = None) -> str:
    """Create JWT access token."""
    to_encode = data.copy()
    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(minutes=settings.jwt_access_token_expire_minutes)

    to_encode.update({"exp": expire})
    encoded_jwt = jwt.encode(to_encode, settings.jwt_secret_key, algorithm="HS256")
    return encoded_jwt


async def get_current_user(credentials: HTTPAuthorizationCredentials = Depends(security)) -> dict:
    """Get current authenticated user from JWT token."""
    token = credentials.credentials

    try:
        payload = jwt.decode(token, settings.jwt_secret_key, algorithms=["HS256"])
        user_id: str = payload.get("sub")
        if user_id is None:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Could not validate credentials",
            )
    except jwt.ExpiredSignatureError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Token has expired",
        )
    except jwt.JWTError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate credentials",
        )

    # Get user from database
    pool = await get_db_pool()
    async with pool.acquire() as conn:
        user = await conn.fetchrow(
            "SELECT id, email, full_name, role, background, created_at FROM users WHERE id = $1",
            user_id
        )

    if user is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="User not found",
        )

    return dict(user)


# API Endpoints

@router.post("/signup", response_model=TokenResponse, status_code=status.HTTP_201_CREATED)
async def signup(user_data: UserCreate):
    """
    Register a new user.

    Returns JWT access token on success.
    """
    pool = await get_db_pool()

    async with pool.acquire() as conn:
        # Check if user already exists
        existing_user = await conn.fetchrow(
            "SELECT id FROM users WHERE email = $1",
            user_data.email
        )

        if existing_user:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Email already registered",
            )

        # Hash password
        hashed_password = get_password_hash(user_data.password)

        # Create user
        user = await conn.fetchrow(
            """
            INSERT INTO users (email, password_hash, full_name, role, background, preferences)
            VALUES ($1, $2, $3, $4, $5, $6::jsonb)
            RETURNING id, email, full_name, role, background, created_at
            """,
            user_data.email,
            hashed_password,
            user_data.full_name,
            user_data.role or "student",
            user_data.background,
            json.dumps({})  # Empty JSON object for preferences
        )

    # Create access token
    access_token = create_access_token(data={"sub": str(user["id"])})

    return TokenResponse(
        access_token=access_token,
        token_type="bearer",
        user=UserResponse(
            id=str(user["id"]),
            email=user["email"],
            full_name=user["full_name"],
            role=user["role"],
            background=user["background"],
            created_at=user["created_at"]
        )
    )


@router.post("/login", response_model=TokenResponse)
async def login(credentials: UserLogin):
    """
    Login with email and password.

    Returns JWT access token on success.
    """
    pool = await get_db_pool()

    async with pool.acquire() as conn:
        # Get user
        user = await conn.fetchrow(
            "SELECT id, email, password_hash, full_name, role, background, created_at FROM users WHERE email = $1",
            credentials.email
        )

        if not user:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Incorrect email or password",
            )

        # Verify password
        if not verify_password(credentials.password, user["password_hash"]):
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Incorrect email or password",
            )

        # Update last login
        await conn.execute(
            "UPDATE users SET last_login = NOW() WHERE id = $1",
            user["id"]
        )

    # Create access token
    access_token = create_access_token(data={"sub": str(user["id"])})

    return TokenResponse(
        access_token=access_token,
        token_type="bearer",
        user=UserResponse(
            id=str(user["id"]),
            email=user["email"],
            full_name=user["full_name"],
            role=user["role"],
            background=user["background"],
            created_at=user["created_at"]
        )
    )


@router.get("/me", response_model=UserResponse)
async def get_current_user_profile(current_user: dict = Depends(get_current_user)):
    """
    Get current user profile.

    Requires authentication.
    """
    return UserResponse(
        id=str(current_user["id"]),
        email=current_user["email"],
        full_name=current_user["full_name"],
        role=current_user["role"],
        background=current_user["background"],
        created_at=current_user["created_at"]
    )


@router.post("/logout")
async def logout():
    """
    Logout (client-side token removal).

    Note: JWT tokens can't be invalidated server-side without a token blacklist.
    For now, client should just delete the token.
    """
    return {"message": "Logged out successfully. Please delete your token."}


@router.put("/profile", response_model=UserResponse)
async def update_profile(
    full_name: Optional[str] = None,
    background: Optional[str] = None,
    current_user: dict = Depends(get_current_user)
):
    """
    Update user profile.

    Requires authentication.
    """
    pool = await get_db_pool()

    updates = []
    params = []
    param_count = 1

    if full_name is not None:
        updates.append(f"full_name = ${param_count}")
        params.append(full_name)
        param_count += 1

    if background is not None:
        updates.append(f"background = ${param_count}")
        params.append(background)
        param_count += 1

    if not updates:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="No fields to update",
        )

    params.append(current_user["id"])

    query = f"""
        UPDATE users
        SET {", ".join(updates)}
        WHERE id = ${param_count}
        RETURNING id, email, full_name, role, background, created_at
    """

    async with pool.acquire() as conn:
        user = await conn.fetchrow(query, *params)

    return UserResponse(
        id=str(user["id"]),
        email=user["email"],
        full_name=user["full_name"],
        role=user["role"],
        background=user["background"],
        created_at=user["created_at"]
    )
