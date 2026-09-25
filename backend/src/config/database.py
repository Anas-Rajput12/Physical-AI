from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker, declarative_base
from ..config.settings import settings


# ============================================================
# DATABASE URL
# ============================================================

SQLALCHEMY_DATABASE_URL = settings.neon_database_url


if not SQLALCHEMY_DATABASE_URL:
    raise ValueError(
        "NEON_DATABASE_URL is not configured."
    )


# ============================================================
# DATABASE ENGINE
# ============================================================

engine = create_engine(
    SQLALCHEMY_DATABASE_URL,
    pool_pre_ping=True,
    pool_recycle=300,
    pool_size=1,
    max_overflow=2,
)


# ============================================================
# SESSION
# ============================================================

SessionLocal = sessionmaker(
    autocommit=False,
    autoflush=False,
    bind=engine,
)


# ============================================================
# BASE
# ============================================================

Base = declarative_base()


# ============================================================
# DATABASE DEPENDENCY
# ============================================================

def get_db():
    db = SessionLocal()

    try:
        yield db
    except Exception:
        db.rollback()
        raise
    finally:
        db.close()
