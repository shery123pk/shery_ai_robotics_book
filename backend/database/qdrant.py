"""
Qdrant Cloud vector database client for storing and searching book embeddings.
Used for RAG (Retrieval-Augmented Generation) chatbot functionality.
"""

from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
from typing import Optional, List, Dict, Any
from config import settings


# Global Qdrant client
_client: Optional[QdrantClient] = None


def get_qdrant_client() -> QdrantClient:
    """
    Get or create the Qdrant client.

    Returns:
        QdrantClient: Qdrant Cloud client instance
    """
    global _client

    if _client is None:
        _client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
        )

    return _client


async def create_collection_if_not_exists():
    """
    Create the book embeddings collection if it doesn't exist.
    Collection schema:
    - Vectors: 1536 dimensions (OpenAI text-embedding-ada-002)
    - Distance: Cosine similarity
    - Payload: module, chapter_id, section_title, chunk_text, chunk_index
    """
    client = get_qdrant_client()
    collection_name = settings.qdrant_collection_name

    # Check if collection exists
    collections = client.get_collections().collections
    collection_exists = any(c.name == collection_name for c in collections)

    if not collection_exists:
        client.create_collection(
            collection_name=collection_name,
            vectors_config=VectorParams(
                size=1536,  # OpenAI text-embedding-ada-002 dimension
                distance=Distance.COSINE,
            ),
        )
        print(f"Created Qdrant collection: {collection_name}")
    else:
        print(f"Qdrant collection already exists: {collection_name}")


async def search_similar_chunks(
    query_embedding: List[float],
    limit: int = 5,
    score_threshold: float = 0.7,
) -> List[Dict[str, Any]]:
    """
    Search for similar text chunks using vector similarity.

    Args:
        query_embedding: Query vector (1536 dimensions)
        limit: Maximum number of results to return
        score_threshold: Minimum similarity score (0-1)

    Returns:
        List of matching chunks with metadata and scores
    """
    client = get_qdrant_client()

    results = client.search(
        collection_name=settings.qdrant_collection_name,
        query_vector=query_embedding,
        limit=limit,
        score_threshold=score_threshold,
    )

    return [
        {
            "id": result.id,
            "score": result.score,
            "module": result.payload.get("module"),
            "chapter_id": result.payload.get("chapter_id"),
            "section_title": result.payload.get("section_title"),
            "chunk_text": result.payload.get("chunk_text"),
            "chunk_index": result.payload.get("chunk_index"),
        }
        for result in results
    ]


async def upsert_embeddings(points: List[PointStruct]):
    """
    Insert or update embeddings in the collection.

    Args:
        points: List of PointStruct objects with vectors and payloads
    """
    client = get_qdrant_client()

    client.upsert(
        collection_name=settings.qdrant_collection_name,
        points=points,
    )
