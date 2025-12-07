"""
Database migration runner for Neon Serverless Postgres.
Executes SQL migration files in order.
"""

import asyncio
import asyncpg
from pathlib import Path
from config import settings


async def run_migrations():
    """Run all SQL migrations in the migrations/ directory."""
    print("🔄 Starting database migrations...")

    # Connect to database
    conn = await asyncpg.connect(dsn=settings.database_url)

    try:
        # Get migration files sorted by name (001_, 002_, 003_)
        migrations_dir = Path(__file__).parent / "migrations"
        migration_files = sorted(migrations_dir.glob("*.sql"))

        if not migration_files:
            print("⚠️  No migration files found")
            return

        # Execute each migration
        for migration_file in migration_files:
            print(f"\n📝 Running migration: {migration_file.name}")

            # Read SQL file
            sql = migration_file.read_text(encoding="utf-8")

            # Execute migration
            await conn.execute(sql)

            print(f"✅ Completed: {migration_file.name}")

        print("\n🎉 All migrations completed successfully!")

    except Exception as e:
        print(f"\n❌ Migration failed: {str(e)}")
        raise

    finally:
        await conn.close()


if __name__ == "__main__":
    asyncio.run(run_migrations())
