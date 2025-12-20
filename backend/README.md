---
title: AI Robotics Chatbot Backend
emoji: 🤖
colorFrom: green
colorTo: blue
sdk: docker
app_port: 7860
---

# Physical AI & Humanoid Robotics Chatbot Backend

This is the FastAPI backend for an AI-powered chatbot that helps students learn about Physical AI and Humanoid Robotics.

## Features

- 🤖 **RAG-Powered Chatbot**: Uses OpenAI GPT-4 with Retrieval-Augmented Generation
- 🔍 **Vector Search**: Qdrant vector database for semantic search
- 💾 **Chat History**: PostgreSQL database for storing conversations
- 🔐 **Authentication**: JWT-based user authentication
- 🌍 **Multi-language**: Support for translation features
- 📚 **Course Topics**: ROS 2, Gazebo, Unity, NVIDIA Isaac, VLA models

## API Endpoints

- `GET /` - Health check
- `GET /api/health` - Detailed health status
- `POST /api/chat/message` - Send message to chatbot
- `GET /api/docs` - Interactive API documentation (Swagger UI)
- `GET /api/redoc` - Alternative API documentation

## Technology Stack

- **Framework**: FastAPI
- **AI Model**: OpenAI GPT-4
- **Vector DB**: Qdrant Cloud
- **Database**: Neon PostgreSQL
- **Authentication**: JWT with bcrypt

## Usage

### Send a Chat Message

```bash
curl -X POST "https://YOUR_SPACE_URL/api/chat/message" \
  -H "Content-Type: application/json" \
  -d '{
    "message": "What is ROS 2?",
    "session_id": "test-session",
    "user_id": null
  }'
```

### Response Format

```json
{
  "response": "ROS 2 (Robot Operating System 2) is...",
  "citations": [
    {
      "module": "Module 1",
      "chapter": "ROS 2 Introduction",
      "section": "What is ROS 2"
    }
  ],
  "timestamp": "2025-12-18T10:30:00"
}
```

## Frontend

This backend powers the chatbot on:
https://shery-ai-robotics-book.vercel.app

## Environment Variables

Required secrets (set in Space Settings):
- `OPENAI_API_KEY` - OpenAI API key
- `QDRANT_URL` - Qdrant cloud URL
- `QDRANT_API_KEY` - Qdrant API key
- `DATABASE_URL` - PostgreSQL connection string
- `JWT_SECRET_KEY` - Secret for JWT tokens
- `CORS_ORIGINS` - Allowed frontend origins

## Documentation

For detailed deployment instructions, see `HUGGINGFACE_DEPLOYMENT.md` in the repository.

## Repository

https://github.com/shery123pk/shery_ai_robotics_book

## License

MIT License

---

Built with ❤️ using FastAPI, OpenAI, and Hugging Face Spaces
