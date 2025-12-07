"""
Qdrant Cloud collection setup script.
Creates the book_embeddings collection with proper schema.
"""

import asyncio
from database.qdrant import create_collection_if_not_exists, get_qdrant_client
from config import settings


async def setup_qdrant_collection():
    """Set up Qdrant collection for book embeddings."""
    print("🔄 Setting up Qdrant collection...")

    try:
        # Create collection if it doesn't exist
        await create_collection_if_not_exists()

        # Verify collection exists
        client = get_qdrant_client()
        collection_info = client.get_collection(settings.qdrant_collection_name)

        print(f"\n✅ Qdrant collection ready: {settings.qdrant_collection_name}")
        print(f"   - Vectors size: {collection_info.config.params.vectors.size}")
        print(f"   - Distance: {collection_info.config.params.vectors.distance}")
        print(f"   - Points count: {collection_info.points_count}")

        print("\n🎉 Qdrant setup completed successfully!")

    except Exception as e:
        print(f"\n❌ Qdrant setup failed: {str(e)}")
        raise


if __name__ == "__main__":
    asyncio.run(setup_qdrant_collection())
