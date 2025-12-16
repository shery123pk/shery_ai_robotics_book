#!/usr/bin/env python3
"""
Ingest textbook content into Qdrant vector database
"""

import os
import sys
from pathlib import Path
from typing import List, Dict
import re
from openai import OpenAI
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
import hashlib
import uuid

# Add parent directory to path
sys.path.append(str(Path(__file__).parent.parent))

from config import settings

# Simple print-based logging
class SimpleLogger:
    @staticmethod
    def info(msg): print(f"[INFO] {msg}")
    @staticmethod
    def error(msg): print(f"[ERROR] {msg}")
    @staticmethod
    def warning(msg): print(f"[WARNING] {msg}")

logger = SimpleLogger()


def parse_markdown_file(file_path: Path) -> Dict:
    """Parse markdown file and extract metadata."""
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Extract frontmatter
    frontmatter = {}
    if content.startswith('---'):
        parts = content.split('---', 2)
        if len(parts) >= 3:
            frontmatter_text = parts[1]
            content = parts[2]

            # Parse frontmatter
            for line in frontmatter_text.strip().split('\n'):
                if ':' in line:
                    key, value = line.split(':', 1)
                    frontmatter[key.strip()] = value.strip()

    # Determine module and chapter from path
    path_parts = file_path.parts
    module = ""
    chapter = file_path.stem

    for part in path_parts:
        if part.startswith('module-'):
            module = part.replace('module-', 'Module ')
            module = module.replace('-', ' ').title()

    return {
        'content': content.strip(),
        'module': module if module else "Introduction",
        'chapter': chapter.replace('-', ' ').title(),
        'file_path': str(file_path)
    }


def chunk_content(content: str, chunk_size: int = 1000) -> List[Dict]:
    """Split content into chunks by sections."""
    chunks = []

    # Split by headers
    sections = re.split(r'\n#{1,3}\s+', content)

    current_chunk = ""
    current_section = "Introduction"

    for i, section in enumerate(sections):
        if i == 0 and not section.startswith('#'):
            # First section might not have header
            lines = section.split('\n', 1)
            if len(lines) > 1:
                current_chunk = lines[1]
            continue

        # Extract section title
        lines = section.split('\n', 1)
        section_title = lines[0].strip()
        section_content = lines[1] if len(lines) > 1 else ""

        # If adding this would exceed chunk size, save current chunk
        if len(current_chunk) + len(section_content) > chunk_size and current_chunk:
            chunks.append({
                'text': current_chunk.strip(),
                'section': current_section
            })
            current_chunk = ""

        current_section = section_title
        current_chunk += f"\n\n{section_content}"

    # Add final chunk
    if current_chunk.strip():
        chunks.append({
            'text': current_chunk.strip(),
            'section': current_section
        })

    return chunks


def generate_embedding(text: str, client: OpenAI) -> List[float]:
    """Generate embedding using OpenAI."""
    response = client.embeddings.create(
        model="text-embedding-3-small",
        input=text
    )
    return response.data[0].embedding


def ingest_textbook_content(docs_dir: str):
    """Main ingestion function."""

    logger.info("Starting content ingestion...")

    # Initialize clients
    openai_client = OpenAI(api_key=settings.openai_api_key)
    qdrant_client = QdrantClient(
        url=settings.qdrant_url,
        api_key=settings.qdrant_api_key
    )

    # Create or recreate collection
    collection_name = settings.qdrant_collection_name

    try:
        qdrant_client.delete_collection(collection_name)
        logger.info(f"Deleted existing collection: {collection_name}")
    except Exception:
        pass

    qdrant_client.create_collection(
        collection_name=collection_name,
        vectors_config=VectorParams(size=1536, distance=Distance.COSINE)
    )
    logger.info(f"Created collection: {collection_name}")

    # Find all markdown files
    docs_path = Path(docs_dir)
    md_files = list(docs_path.glob('**/*.md'))

    logger.info(f"Found {len(md_files)} markdown files")

    points = []
    point_id = 0

    for md_file in md_files:
        logger.info(f"Processing: {md_file}")

        # Parse file
        doc_data = parse_markdown_file(md_file)

        # Chunk content
        chunks = chunk_content(doc_data['content'], chunk_size=800)

        logger.info(f"  Created {len(chunks)} chunks")

        # Create embeddings and points
        for chunk in chunks:
            # Generate embedding
            embedding = generate_embedding(chunk['text'], openai_client)

            # Create point
            point = PointStruct(
                id=point_id,
                vector=embedding,
                payload={
                    'text': chunk['text'],
                    'section': chunk['section'],
                    'module': doc_data['module'],
                    'chapter': doc_data['chapter'],
                    'file_path': doc_data['file_path']
                }
            )
            points.append(point)
            point_id += 1

    # Upload to Qdrant in batches
    batch_size = 100
    for i in range(0, len(points), batch_size):
        batch = points[i:i + batch_size]
        qdrant_client.upsert(
            collection_name=collection_name,
            points=batch
        )
        logger.info(f"Uploaded batch {i // batch_size + 1}/{(len(points) + batch_size - 1) // batch_size}")

    logger.info(f"✓ Ingestion complete! Total points: {len(points)}")

    # Verify
    collection_info = qdrant_client.get_collection(collection_name)
    logger.info(f"Collection stats: {collection_info.points_count} points")


if __name__ == "__main__":
    # Get docs directory
    docs_dir = Path(__file__).parent.parent / "docs"

    if not docs_dir.exists():
        logger.error(f"Docs directory not found: {docs_dir}")
        sys.exit(1)

    ingest_textbook_content(str(docs_dir))
