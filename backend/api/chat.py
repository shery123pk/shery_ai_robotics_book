"""
Chat API endpoint for RAG chatbot
"""

from fastapi import APIRouter, HTTPException, Depends
from models.chat import ChatRequest, ChatResponse, Citation
from database.qdrant import get_qdrant_client
from database.postgres import get_db_pool
from utils.rate_limit import is_rate_limited
from utils.logger import get_logger
from config import settings
from openai import OpenAI
from qdrant_client.models import Filter, FieldCondition, MatchValue
import asyncpg
from typing import List
import uuid

router = APIRouter(prefix="/api/chat", tags=["chat"])
logger = get_logger(__name__)
openai_client = OpenAI(api_key=settings.openai_api_key)


async def get_relevant_content(query: str, limit: int = 5) -> List[dict]:
    """Retrieve relevant content from Qdrant vector database."""
    qdrant = get_qdrant_client()

    # Generate embedding for query
    response = openai_client.embeddings.create(
        model="text-embedding-3-small",
        input=query
    )
    query_embedding = response.data[0].embedding

    # Search in Qdrant
    search_results = qdrant.query_points(
        collection_name=settings.qdrant_collection_name,
        query=query_embedding,
        limit=limit,
        with_payload=True
    ).points

    # Extract relevant chunks
    contexts = []
    for result in search_results:
        contexts.append({
            "text": result.payload.get("text", ""),
            "module": result.payload.get("module", ""),
            "chapter": result.payload.get("chapter", ""),
            "section": result.payload.get("section", ""),
            "score": result.score
        })

    return contexts


async def generate_response(query: str, contexts: List[dict]) -> tuple[str, List[Citation]]:
    """Generate response using GPT-4 with retrieved context."""

    # Build context string
    context_text = "\n\n".join([
        f"[{ctx['module']} - {ctx['chapter']}]\n{ctx['text']}"
        for ctx in contexts
    ])

    # System prompt
    system_prompt = """You are a friendly and knowledgeable AI tutor for Physical AI and Humanoid Robotics.
    You're here to help students learn about ROS 2, simulation, NVIDIA Isaac, and VLA models.

    Personality:
    - Be warm, encouraging, and conversational
    - Use a friendly, human tone (like talking to a friend)
    - Respond to greetings naturally ("Hi!", "Hello!", "Hey there!")
    - Show enthusiasm for the subject
    - Be patient and understanding

    Teaching Guidelines:
    1. When students ask questions, use the provided textbook context to give accurate answers
    2. Explain concepts clearly and break down complex topics
    3. Reference specific modules/chapters when it helps (e.g., "In Module 2, we learn that...")
    4. Include code examples from context when available
    5. If the context doesn't fully answer their question, acknowledge what you know and suggest related topics
    6. For greetings or casual conversation, respond warmly without needing textbook context

    Format: Use clear, friendly language. Use Markdown for code and formatting when helpful.
    """

    # User prompt
    user_prompt = f"""Context from textbook:
{context_text}

Student question: {query}

Please answer the question based on the context above."""

    # Call GPT-4
    completion = openai_client.chat.completions.create(
        model="gpt-4-turbo-preview",
        messages=[
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_prompt}
        ],
        temperature=0.7,
        max_tokens=1000
    )

    answer = completion.choices[0].message.content

    # Create citations
    citations = [
        Citation(
            module=ctx["module"],
            chapter=ctx["chapter"],
            section=ctx.get("section")
        )
        for ctx in contexts[:3]  # Top 3 citations
    ]

    return answer, citations


@router.post("/message", response_model=ChatResponse)
async def chat_message(request: ChatRequest):
    """
    Handle chat message and return AI response with citations
    """

    # Rate limiting
    rate_key = f"chat:{request.session_id}"
    if is_rate_limited(rate_key, limit=10, window_seconds=60):
        raise HTTPException(status_code=429, detail="Rate limit exceeded. Please wait a moment.")

    try:
        logger.info(f"Chat query: {request.message[:100]}")

        # Check if it's a greeting or casual conversation
        message_lower = request.message.lower().strip()
        logger.info(f"Message lower: '{message_lower}'")

        greetings = ['hi', 'hello', 'hey', 'hi there', 'hello there', 'hey there', 'good morning', 'good afternoon', 'good evening']
        casual_phrases = ['how are you', 'whats up', 'what\'s up', 'sup', 'yo', 'hiya']

        is_greeting = message_lower in greetings or any(phrase in message_lower for phrase in casual_phrases)
        logger.info(f"Is greeting: {is_greeting}")

        if is_greeting:
            # Respond to greeting warmly without needing textbook context
            logger.info("Responding with friendly greeting")
            friendly_responses = [
                "Hey there! 👋 I'm so glad you're here! I'm ready to help you learn about Physical AI and Humanoid Robotics. What topic would you like to explore? ROS 2, simulation, NVIDIA Isaac, or maybe VLA models?",
                "Hi! 😊 Great to see you! I'm your AI tutor for robotics and I'm excited to help you learn. What are you curious about today?",
                "Hello! 🤖 Welcome! I'm here to make learning Physical AI and Humanoid Robotics fun and easy. What would you like to know?",
            ]
            import random
            response_text = random.choice(friendly_responses)
            logger.info(f"Greeting response: {response_text[:50]}")
            return ChatResponse(
                response=response_text,
                citations=[],
                session_id=request.session_id
            )

        # Retrieve relevant content
        contexts = await get_relevant_content(request.message, limit=5)

        if not contexts:
            return ChatResponse(
                response="I couldn't find relevant information in the textbook. Could you rephrase your question?",
                citations=[],
                session_id=request.session_id
            )

        # Generate response
        answer, citations = await generate_response(request.message, contexts)

        # Store in database (async)
        if request.user_id:
            try:
                pool = await get_db_pool()
                async with pool.acquire() as conn:
                    await conn.execute(
                        """
                        INSERT INTO chat_messages (user_id, session_id, message_text, response_text, citations)
                        VALUES ($1, $2, $3, $4, $5)
                        """,
                        uuid.UUID(request.user_id) if request.user_id else None,
                        request.session_id,
                        request.message,
                        answer,
                        [{"module": c.module, "chapter": c.chapter, "section": c.section} for c in citations]
                    )
            except Exception as e:
                logger.error(f"Failed to store chat message: {e}")
                # Don't fail the request if storage fails

        logger.info(f"Response generated with {len(citations)} citations")

        return ChatResponse(
            response=answer,
            citations=citations,
            session_id=request.session_id
        )

    except Exception as e:
        logger.error(f"Chat error: {e}")
        raise HTTPException(status_code=500, detail=f"Chat error: {str(e)}")


@router.get("/health")
async def chat_health():
    """Health check for chat service"""
    try:
        # Test Qdrant connection
        qdrant = get_qdrant_client()
        collections = qdrant.get_collections()

        # Test OpenAI connection
        openai_client.models.list()

        return {
            "status": "healthy",
            "qdrant_collections": len(collections.collections),
            "openai": "connected"
        }
    except Exception as e:
        raise HTTPException(status_code=503, detail=f"Service unhealthy: {str(e)}")
