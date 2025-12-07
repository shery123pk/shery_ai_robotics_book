# Implementation Plan: Physical AI & Humanoid Robotics Textbook

**Branch**: `main` | **Date**: 2025-12-06 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/physical-ai-textbook/spec.md`

## Summary

Build an AI-native interactive textbook for teaching Physical AI & Humanoid Robotics, deployed as a Docusaurus static site with an embedded RAG chatbot. The system consists of:
1. **Frontend**: Docusaurus v3 site with 4 modules (ROS 2, Gazebo/Unity, NVIDIA Isaac, VLA), deployed to GitHub Pages
2. **Backend**: FastAPI service providing RAG chatbot (OpenAI + Qdrant + Neon Postgres), authentication (better-auth.com), personalization, and translation
3. **Bonus Features**: Content personalization based on user background, Urdu translation, and Claude Code Subagents for content generation

**Technical Approach**: Static site generation (Docusaurus) for fast loading + serverless backend (Vercel/Railway) for dynamic features, all hosted on free tiers ($0/month cost).

## Technical Context

**Language/Version**:
- Frontend: TypeScript 5.x / JavaScript ES2022, Node.js 18+
- Backend: Python 3.11+
- Content: Markdown with MDX for interactive components

**Primary Dependencies**:
- Frontend: Docusaurus 3.x, React 18, Axios (API calls), react-markdown (dynamic rendering)
- Backend: FastAPI 0.104+, OpenAI Python SDK 1.x, Qdrant Client 1.7+, asyncpg 0.29+ (Postgres), better-auth SDK
- Development: Claude Code (Sonnet 4.5), Spec-Kit Plus

**Storage**:
- Vector Database: Qdrant Cloud Free Tier (embeddings for book content)
- Relational Database: Neon Serverless Postgres Free Tier (users, chat history, cached personalized content)
- Static Assets: GitHub Pages CDN (book content, images)
- Environment Variables: Vercel/Railway secrets management

**Testing**:
- Frontend: Jest + React Testing Library (component tests), Playwright (E2E)
- Backend: pytest + pytest-asyncio (unit/integration), httpx (API client tests)
- Content: Manual review + spell check (no automated tests for educational content quality)

**Target Platform**:
- Frontend: Modern browsers (Chrome 90+, Firefox 88+, Safari 14+, Edge 90+), mobile responsive
- Backend: Linux server (Vercel/Railway containerized deployment)
- Development: Cross-platform (Windows/macOS/Linux)

**Project Type**: Web application (frontend + backend)

**Performance Goals**:
- Page load time: < 2 seconds (p95) for any chapter
- Chatbot response: < 3 seconds (p95) for RAG queries
- API throughput: 10 concurrent requests without degradation
- Lighthouse score: > 90 (performance, accessibility, SEO)

**Constraints**:
- Budget: $0/month (free tier only)
- Security: HTTPS only, no plaintext secrets, bcrypt password hashing
- Accessibility: WCAG 2.1 Level A (alt text, keyboard navigation, RTL for Urdu)
- Deadline: Nov 30, 2025, 6:00 PM (hard stop)

**Scale/Scope**:
- Content: 4 modules, 12-16 chapters total (3-4 per module)
- Users: Designed for 100-500 concurrent readers (free tier limits)
- Embeddings: ~500-1000 text chunks (Qdrant 1GB free tier sufficient)
- Chat history: ~10,000 messages (Neon 0.5GB sufficient)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

✅ **I. Educational Excellence**: Plan includes progressive content structure (foundational → advanced), code examples, and self-assessment
✅ **II. Technical Accuracy**: Code examples versioned (ROS 2 Humble, Ubuntu 22.04), citations required
✅ **III. AI-Native Content Generation**: Claude Code + Spec-Kit Plus workflow documented, Subagents planned
✅ **IV. Modularity & Composability**: Each module self-contained, Docusaurus sidebar for independent navigation
✅ **V. Interactive AI Learning**: RAG chatbot with text-selection Q&A, <3s response time target
✅ **VI. Accessibility & Personalization**: Background questionnaire at signup, personalization API, Urdu translation with RTL
✅ **VII. Code Quality**: FastAPI async patterns, type hints, PEP 8, security (no hardcoded secrets)
✅ **VIII. Deployment & Performance**: GitHub Pages + Vercel free tier, <2s load time, $0/month cost

**Hackathon-Specific Override Applied**: Speed over perfection (80% complete > 100% polished)

## Project Structure

### Documentation (this feature)

```text
specs/physical-ai-textbook/
├── spec.md              ✅ Complete (50 functional requirements, 6 user stories)
├── plan.md              ← This file (architectural decisions)
├── data-model.md        → Next: Database schema, API contracts
├── api-contracts.md     → Next: REST endpoint specifications
└── tasks.md             → After plan: Testable implementation tasks
```

### Source Code (repository root)

```text
shery_ai_book/                     # Project root
│
├── docs/                          # Docusaurus content (static site source)
│   ├── intro.md                   # Course overview, learning outcomes
│   ├── hardware-requirements.md   # Hardware specs from hackathon doc
│   ├── module-1/                  # ROS 2: Robotic Nervous System
│   │   ├── _category_.json        # Sidebar metadata
│   │   ├── 01-ros2-intro.md
│   │   ├── 02-nodes-topics.md
│   │   ├── 03-services-actions.md
│   │   └── 04-urdf-robots.md
│   ├── module-2/                  # Gazebo & Unity: Digital Twin
│   │   ├── 01-simulation-intro.md
│   │   ├── 02-gazebo-physics.md
│   │   ├── 03-unity-rendering.md
│   │   └── 04-sensor-simulation.md
│   ├── module-3/                  # NVIDIA Isaac: AI-Robot Brain
│   │   ├── 01-isaac-overview.md
│   │   ├── 02-isaac-sim.md
│   │   ├── 03-isaac-ros.md
│   │   └── 04-navigation-nav2.md
│   └── module-4/                  # Vision-Language-Action (VLA)
│       ├── 01-vla-intro.md
│       ├── 02-voice-commands.md
│       ├── 03-llm-planning.md
│       └── 04-capstone-project.md
│
├── src/                           # Custom React components (Docusaurus)
│   ├── components/
│   │   ├── ChatBot/
│   │   │   ├── ChatBot.tsx        # Main chatbot UI component
│   │   │   ├── ChatMessage.tsx    # Individual message bubble
│   │   │   ├── ChatInput.tsx      # Message input with text selection handler
│   │   │   └── ChatBot.module.css # Scoped styles
│   │   ├── PersonalizeButton/
│   │   │   ├── PersonalizeButton.tsx  # Chapter personalization button
│   │   │   └── PersonalizeButton.module.css
│   │   ├── TranslateButton/
│   │   │   ├── TranslateButton.tsx    # Urdu translation button
│   │   │   └── TranslateButton.module.css
│   │   └── AuthPanel/
│   │       ├── SignUpForm.tsx     # Registration with background questions
│   │       ├── SignInForm.tsx     # Login form
│   │       └── AuthPanel.module.css
│   ├── pages/                     # Custom Docusaurus pages
│   │   └── index.tsx              # Custom homepage (optional, use docs/intro.md instead)
│   └── theme/                     # Docusaurus theme customization
│       └── DocItem/               # Wrap MDX docs to inject Personalize/Translate buttons
│           └── index.tsx
│
├── backend/                       # FastAPI backend service
│   ├── main.py                    # FastAPI app entry point, CORS config
│   ├── config.py                  # Environment variables (OpenAI key, DB URLs)
│   ├── models/                    # Pydantic models (request/response schemas)
│   │   ├── auth.py                # SignUpRequest, SignInRequest, UserProfile
│   │   ├── chat.py                # ChatRequest, ChatResponse, ChatHistory
│   │   ├── content.py             # PersonalizeRequest, TranslateRequest
│   │   └── database.py            # SQLAlchemy/async ORM models
│   ├── services/
│   │   ├── auth_service.py        # better-auth integration, JWT token handling
│   │   ├── rag_service.py         # RAG engine (OpenAI + Qdrant retrieval)
│   │   ├── personalize_service.py # Content personalization with LLM
│   │   ├── translate_service.py   # Urdu translation with LLM
│   │   └── embedding_service.py   # Generate embeddings for book content
│   ├── database/
│   │   ├── postgres.py            # Neon Postgres connection (asyncpg)
│   │   ├── qdrant.py              # Qdrant client initialization
│   │   └── migrations/            # SQL migration scripts (optional for hackathon)
│   ├── api/                       # API route handlers
│   │   ├── chat.py                # POST /api/chat, /api/chat/selection, GET /api/chat/history
│   │   ├── auth.py                # POST /api/auth/signup, /api/auth/signin
│   │   └── content.py             # POST /api/personalize, /api/translate
│   ├── utils/
│   │   ├── security.py            # Password hashing (bcrypt), JWT utilities
│   │   ├── rate_limit.py          # Rate limiting decorator (10 req/min)
│   │   └── logger.py              # Structured logging setup
│   ├── tests/
│   │   ├── test_auth.py           # Authentication endpoint tests
│   │   ├── test_chat.py           # RAG chatbot tests
│   │   └── test_content.py        # Personalization/translation tests
│   ├── requirements.txt           # Python dependencies
│   └── Dockerfile                 # Optional for local Docker deployment
│
├── scripts/                       # Utility scripts
│   ├── generate_embeddings.py    # One-time script to populate Qdrant with book embeddings
│   ├── seed_database.py           # Create test users and sample data
│   └── validate_links.py          # Check all markdown links are valid
│
├── .github/
│   └── workflows/
│       ├── deploy-frontend.yml    # Auto-deploy Docusaurus to GitHub Pages on push to main
│       └── deploy-backend.yml     # Auto-deploy FastAPI to Vercel/Railway on push to main
│
├── .specify/                      # Spec-Kit Plus configuration (already exists)
│   ├── memory/
│   │   └── constitution.md        ✅ Complete (v1.0.0)
│   ├── templates/                 # PHR, ADR, spec, plan, tasks templates
│   └── scripts/
│
├── history/                       # Prompt History Records
│   └── prompts/
│       ├── constitution/
│       │   └── 001-*.md           ✅ Complete
│       └── physical-ai-textbook/
│           └── 002-*.md           ✅ Complete
│
├── docusaurus.config.js           # Docusaurus site configuration
├── sidebars.js                    # Sidebar navigation (auto-generated from _category_.json)
├── package.json                   # Frontend dependencies (Node.js)
├── tsconfig.json                  # TypeScript configuration
├── .env.example                   # Environment variable template (no secrets)
├── .env                           # Local secrets (gitignored)
├── .gitignore                     # Ignore node_modules, .env, build artifacts
└── README.md                      # Project overview, setup instructions, submission checklist
```

**Structure Decision**: **Option 2: Web application** selected because project requires:
1. Static frontend (Docusaurus) for SEO-optimized textbook content
2. Dynamic backend (FastAPI) for RAG chatbot, authentication, and personalization
3. Clear separation of concerns: content generation (docs/) vs. business logic (backend/)

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| None | N/A | All complexity justified by hackathon requirements |

**Rationale**:
- Backend API required for RAG chatbot (can't do vector search in browser)
- better-auth.com requires server-side session management
- Personalization/translation need LLM API calls (can't expose API keys in frontend)
- Architecture follows constitution principle VIII: minimal complexity for requirements

## Phase 0: Research & Technology Validation

### 0.1 Docusaurus Setup Research
**Goal**: Validate Docusaurus v3 supports our requirements (embedded React components, GitHub Pages deployment)

**Research Tasks**:
- [ ] Verify Docusaurus 3.x supports custom React components in MDX (for chatbot embedding)
- [ ] Confirm GitHub Pages deployment workflow (manual vs. GitHub Actions)
- [ ] Check if Docusaurus supports RTL (right-to-left) layout for Urdu translation
- [ ] Investigate theme customization for injecting Personalize/Translate buttons at chapter start

**Deliverables**: `research.md` file documenting Docusaurus capabilities and limitations

### 0.2 RAG Stack Validation
**Goal**: Validate OpenAI + Qdrant + Neon Postgres stack works together

**Research Tasks**:
- [ ] Test OpenAI embeddings API (text-embedding-ada-002 model, 1536 dimensions)
- [ ] Verify Qdrant Cloud free tier limits (1GB storage = ~600k vectors at 1536 dims)
- [ ] Test Neon Serverless Postgres async connection with asyncpg
- [ ] Benchmark RAG retrieval latency (target: <1s for similarity search + LLM response <2s)

**Deliverables**: Proof-of-concept Python script demonstrating end-to-end RAG flow

### 0.3 Authentication Provider Evaluation
**Goal**: Validate better-auth.com meets requirements or select alternative

**Research Tasks**:
- [ ] Review better-auth.com documentation for FastAPI integration
- [ ] Check if it supports custom signup fields (software_experience, hardware_experience)
- [ ] Verify free tier limits (number of users, API calls)
- [ ] **Fallback Plan**: If better-auth.com incompatible, use manual JWT auth with bcrypt

**Deliverables**: Authentication implementation decision documented in `research.md`

### 0.4 Deployment Platform Selection
**Goal**: Choose between Vercel and Railway for backend hosting

**Comparison Matrix**:
| Criteria | Vercel | Railway |
|----------|--------|---------|
| Free tier hours | Serverless (unlimited invocations, 100GB bandwidth) | 500 hours/month |
| FastAPI support | Yes (via Python runtime) | Yes (Docker/Procfile) |
| Environment variables | Yes (secrets UI) | Yes (secrets UI) |
| Auto-deploy from GitHub | Yes | Yes |
| Cold start time | ~1-2s | ~5-10s |

**Decision**: **Vercel preferred** for serverless model (no cold start limit), fallback to Railway if Vercel Python runtime has issues.

**Deliverables**: Deployment platform decision in `research.md`

## Phase 1: Detailed Design

### 1.1 Database Schema Design

**Output**: `data-model.md` file with schema definitions

#### Neon Postgres Tables

**Table: users**
```sql
CREATE TABLE users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) UNIQUE NOT NULL,
    password_hash VARCHAR(255) NOT NULL,  -- bcrypt hashed
    software_experience VARCHAR(50) NOT NULL,  -- 'beginner' | 'intermediate' | 'advanced'
    hardware_experience VARCHAR(50) NOT NULL,  -- 'none' | 'hobbyist' | 'professional'
    created_at TIMESTAMP DEFAULT NOW(),
    last_login TIMESTAMP
);

CREATE INDEX idx_users_email ON users(email);
```

**Table: chat_messages**
```sql
CREATE TABLE chat_messages (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID REFERENCES users(id) ON DELETE CASCADE,  -- nullable for anonymous users
    session_id VARCHAR(255) NOT NULL,  -- browser session ID
    message_text TEXT NOT NULL,
    response_text TEXT NOT NULL,
    citations JSONB,  -- Array of {module, chapter, section}
    created_at TIMESTAMP DEFAULT NOW()
);

CREATE INDEX idx_chat_user_session ON chat_messages(user_id, session_id);
CREATE INDEX idx_chat_session ON chat_messages(session_id);
```

**Table: personalized_content**
```sql
CREATE TABLE personalized_content (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    chapter_id VARCHAR(100) NOT NULL,  -- e.g., 'module-1/02-nodes-topics'
    original_markdown TEXT NOT NULL,
    personalized_markdown TEXT NOT NULL,
    generated_at TIMESTAMP DEFAULT NOW(),
    UNIQUE(user_id, chapter_id)  -- Cache one personalized version per user per chapter
);

CREATE INDEX idx_personalized_user_chapter ON personalized_content(user_id, chapter_id);
```

#### Qdrant Collection Schema

**Collection: book_embeddings**
```json
{
  "vectors": {
    "size": 1536,
    "distance": "Cosine"
  },
  "payload_schema": {
    "module": "keyword",
    "chapter_id": "keyword",
    "section_title": "text",
    "chunk_text": "text",
    "chunk_index": "integer"
  }
}
```

**Indexing Strategy**:
- Split each chapter into chunks of ~500 tokens (to fit LLM context window)
- Generate embeddings for each chunk using OpenAI text-embedding-ada-002
- Store metadata: module, chapter_id, section_title for citation generation
- Estimated chunks: 16 chapters × ~40 chunks/chapter = 640 chunks (well within 1GB limit)

### 1.2 API Contract Definitions

**Output**: `api-contracts.md` file with OpenAPI/Swagger specs

#### Endpoint: `POST /api/chat`
**Purpose**: Send message to RAG chatbot, get response with citations

**Request**:
```json
{
  "message": "What is a ROS 2 node?",
  "session_id": "abc123",
  "user_id": "uuid-or-null"
}
```

**Response** (200 OK):
```json
{
  "response": "A ROS 2 node is a process that performs computation...",
  "citations": [
    {"module": "Module 1", "chapter": "ROS 2 Nodes and Topics", "section": "What is a Node?"}
  ],
  "timestamp": "2025-12-06T10:30:00Z"
}
```

**Error** (429 Too Many Requests):
```json
{
  "error": "Rate limit exceeded. Please try again in 60 seconds."
}
```

#### Endpoint: `POST /api/chat/selection`
**Purpose**: Answer question about selected text

**Request**:
```json
{
  "selected_text": "rclpy is the ROS 2 client library for Python...",
  "question": "How do I initialize a node with rclpy?",
  "session_id": "abc123",
  "user_id": "uuid-or-null"
}
```

**Response** (200 OK):
```json
{
  "response": "To initialize a node with rclpy, use: rclpy.init() followed by node = rclpy.create_node('node_name')",
  "citations": [
    {"module": "Module 1", "chapter": "ROS 2 Nodes and Topics", "section": "rclpy Basics"}
  ],
  "timestamp": "2025-12-06T10:31:00Z"
}
```

#### Endpoint: `POST /api/auth/signup`
**Purpose**: Create new user account with background questionnaire

**Request**:
```json
{
  "email": "student@example.com",
  "password": "SecurePassword123!",
  "software_experience": "intermediate",
  "hardware_experience": "hobbyist"
}
```

**Response** (201 Created):
```json
{
  "user_id": "uuid",
  "email": "student@example.com",
  "access_token": "jwt-token-here",
  "token_type": "bearer",
  "expires_in": 2592000
}
```

**Error** (400 Bad Request):
```json
{
  "error": "Email already registered"
}
```

#### Endpoint: `POST /api/auth/signin`
**Purpose**: Authenticate user and return session token

**Request**:
```json
{
  "email": "student@example.com",
  "password": "SecurePassword123!"
}
```

**Response** (200 OK):
```json
{
  "user_id": "uuid",
  "email": "student@example.com",
  "access_token": "jwt-token-here",
  "token_type": "bearer",
  "expires_in": 2592000
}
```

**Error** (401 Unauthorized):
```json
{
  "error": "Invalid email or password"
}
```

#### Endpoint: `GET /api/chat/history`
**Purpose**: Retrieve user's chat history (requires authentication)

**Headers**:
```
Authorization: Bearer <jwt-token>
```

**Query Params**:
- `limit`: Number of messages to return (default: 50, max: 100)
- `offset`: Pagination offset (default: 0)

**Response** (200 OK):
```json
{
  "messages": [
    {
      "id": "uuid",
      "message": "What is a ROS 2 node?",
      "response": "A ROS 2 node is...",
      "citations": [...],
      "timestamp": "2025-12-06T10:30:00Z"
    }
  ],
  "total": 127,
  "limit": 50,
  "offset": 0
}
```

#### Endpoint: `POST /api/personalize`
**Purpose**: Generate personalized chapter content based on user background (requires authentication)

**Headers**:
```
Authorization: Bearer <jwt-token>
```

**Request**:
```json
{
  "chapter_id": "module-1/02-nodes-topics",
  "original_markdown": "# ROS 2 Nodes and Topics\n\n..."
}
```

**Response** (200 OK):
```json
{
  "personalized_markdown": "# ROS 2 Nodes and Topics (Adapted for Intermediate Software / Hobbyist Hardware)\n\n...",
  "cached": false,
  "generated_at": "2025-12-06T10:35:00Z"
}
```

**Logic**:
1. Check `personalized_content` table for cached version (user_id + chapter_id)
2. If cached and < 7 days old, return cached version
3. Else, send original markdown + user background to LLM with prompt:
   ```
   Adapt this robotics textbook chapter for a student with:
   - Software experience: {software_experience}
   - Hardware experience: {hardware_experience}

   Keep all code blocks, images, and links intact. Adjust explanations:
   - Beginner software: Add inline definitions for technical terms
   - Advanced software: Skip basic explanations, add performance tips
   - No hardware: Explain physical concepts in detail
   - Professional hardware: Focus on best practices and standards
   ```
4. Cache result in `personalized_content` table
5. Return personalized markdown

#### Endpoint: `POST /api/translate`
**Purpose**: Translate chapter content to Urdu

**Request**:
```json
{
  "chapter_id": "module-1/02-nodes-topics",
  "original_markdown": "# ROS 2 Nodes and Topics\n\n..."
}
```

**Response** (200 OK):
```json
{
  "translated_markdown": "# ROS 2 نوڈز اور ٹاپکس\n\n...",
  "language": "ur",
  "generated_at": "2025-12-06T10:36:00Z"
}
```

**Logic**:
1. Send original markdown to LLM with prompt:
   ```
   Translate this robotics textbook chapter to Urdu (Pakistan).

   CRITICAL RULES:
   - Keep all code blocks in English (preserve syntax highlighting)
   - Keep technical terms in English: ROS 2, NVIDIA Isaac, Gazebo, URDF, rclpy, etc.
   - Preserve all markdown formatting (headings, lists, links)
   - Use formal academic Urdu
   - Do not translate URLs or image paths
   ```
2. Return translated markdown (no caching for simplicity in hackathon)

### 1.3 Component Architecture

**Output**: Component diagram and interaction flows

#### Frontend Component Hierarchy

```
App (Docusaurus)
├── Navbar
│   ├── Logo
│   ├── Module Navigation Dropdown
│   └── AuthPanel (SignIn/SignUp/UserProfile)
├── Sidebar (Auto-generated from _category_.json)
├── DocItem (MDX wrapper)
│   ├── PersonalizeButton (if logged in)
│   ├── TranslateButton
│   └── MDXContent (chapter markdown)
└── ChatBot (Fixed position, collapsible)
    ├── ChatHeader (minimize/expand)
    ├── ChatMessages (scrollable history)
    ├── ChatInput (text + send button)
    └── TextSelectionHandler (right-click context menu)
```

#### Backend Service Layer

```
FastAPI App
├── CORS Middleware
├── Rate Limiting Middleware
├── API Routes
│   ├── /api/chat → ChatService
│   ├── /api/chat/selection → ChatService (with context)
│   ├── /api/chat/history → ChatService (auth required)
│   ├── /api/auth/signup → AuthService
│   ├── /api/auth/signin → AuthService
│   ├── /api/personalize → PersonalizeService (auth required)
│   └── /api/translate → TranslateService
└── Services
    ├── AuthService → Neon Postgres (users table)
    ├── ChatService → RAGService → OpenAI + Qdrant + Neon (chat_messages)
    ├── PersonalizeService → OpenAI + Neon (personalized_content cache)
    └── TranslateService → OpenAI
```

### 1.4 Deployment Pipeline

**Output**: CI/CD workflow diagram

#### Frontend Deployment (GitHub Pages)

```yaml
# .github/workflows/deploy-frontend.yml
name: Deploy Docusaurus to GitHub Pages

on:
  push:
    branches: [main]
    paths:
      - 'docs/**'
      - 'src/**'
      - 'docusaurus.config.js'
      - 'sidebars.js'

jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-node@v3
        with:
          node-version: 18
      - run: npm ci
      - run: npm run build
      - uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./build
```

#### Backend Deployment (Vercel)

```yaml
# vercel.json
{
  "builds": [
    {
      "src": "backend/main.py",
      "use": "@vercel/python"
    }
  ],
  "routes": [
    {
      "src": "/api/(.*)",
      "dest": "backend/main.py"
    }
  ],
  "env": {
    "OPENAI_API_KEY": "@openai-api-key",
    "QDRANT_URL": "@qdrant-url",
    "QDRANT_API_KEY": "@qdrant-api-key",
    "NEON_DATABASE_URL": "@neon-database-url",
    "JWT_SECRET": "@jwt-secret"
  }
}
```

## Phase 2: Risk Mitigation Strategies

### High Risk: OpenAI API Costs
**Problem**: GPT-4 Turbo calls for chatbot + personalization + translation could exceed budget

**Mitigation**:
1. **Aggressive Caching**: Cache personalized content (7 days), cache common chatbot Q&A pairs
2. **Model Downgrade**: Use GPT-3.5-turbo for translation (cheaper, still good quality)
3. **Prompt Optimization**: Minimize token usage (concise system prompts, truncate long chapters)
4. **Rate Limiting**: Hard limit 10 chatbot messages per user per minute
5. **Budget Alert**: Set OpenAI spending limit to $20/month, monitor daily usage

**Fallback**: If budget exhausted before submission, disable personalization/translation, keep chatbot (core requirement)

### High Risk: Content Generation Time
**Problem**: Writing 12-16 comprehensive chapters manually may exceed deadline

**Mitigation**:
1. **Claude Code Subagents**: Create `/chapter-generator` and `/assessment-generator` subagents (bonus points + time savings)
2. **Template-Driven**: Use consistent chapter structure (Learning Objectives → Concepts → Code Example → Assessment)
3. **Breadth over Depth**: Target 3 chapters per module minimum (12 total), expand if time permits
4. **Parallel Generation**: Generate all Module 1 chapters in one Claude Code session, then modules 2-4
5. **Quality Threshold**: 80% complete acceptable (per constitution), prioritize technical accuracy over polish

**Fallback**: If behind schedule by Nov 27, reduce to 2 chapters per module (8 total), focus on making chatbot work perfectly

### Medium Risk: Qdrant Free Tier Limits
**Problem**: 1GB storage limit may be insufficient if chunks are too large

**Mitigation**:
1. **Chunk Size Optimization**: Target 500 tokens/chunk (not 1000) → reduces embedding storage
2. **Selective Indexing**: Index only main chapter content, skip assessment questions (not critical for RAG)
3. **Monitor Usage**: Check Qdrant dashboard weekly, estimate final size based on Module 1 embeddings
4. **Compression**: Remove unnecessary markdown formatting before embedding (e.g., extra whitespace)

**Fallback**: If limit exceeded, use Pinecone free tier (100k vectors, 1 index) or Weaviate Cloud

### Medium Risk: Authentication Complexity
**Problem**: better-auth.com integration may be more complex than expected

**Mitigation**:
1. **Research First**: Complete Phase 0.3 (auth provider evaluation) before implementing
2. **Simple Alternative Ready**: Manual JWT auth with bcrypt (3-4 hours implementation)
3. **Core Features First**: Implement signup/signin only, skip OAuth (not required for bonus points)
4. **Test Early**: Build auth endpoints in first week, not last minute

**Fallback**: If better-auth.com unusable, implement manual auth (still gets 50 bonus points)

## Phase 3: Development Milestones

### Week 1 (Dec 6-12): Foundation
- [ ] Set up Docusaurus project (1 day)
- [ ] Create initial module structure and placeholder chapters (1 day)
- [ ] Set up FastAPI backend skeleton (1 day)
- [ ] Set up Neon Postgres and Qdrant accounts (0.5 days)
- [ ] Implement database schema (0.5 days)
- [ ] Generate embeddings for placeholder content (1 day)
- [ ] **Deliverable**: Empty book structure deployed to GitHub Pages, backend running locally

### Week 2 (Dec 13-19): Core Features
- [ ] Generate Module 1 content (4 chapters) using Claude Code (2 days)
- [ ] Implement RAG chatbot backend (OpenAI + Qdrant) (2 days)
- [ ] Build ChatBot React component (1 day)
- [ ] Embed chatbot in Docusaurus (1 day)
- [ ] Implement text-selection Q&A (1 day)
- [ ] **Deliverable**: Module 1 complete + working chatbot (50/100 base points achieved)

### Week 3 (Dec 20-26): Content + Authentication
- [ ] Generate Module 2 content (4 chapters) (1.5 days)
- [ ] Generate Module 3 content (4 chapters) (1.5 days)
- [ ] Implement authentication (signup/signin) (2 days)
- [ ] Build AuthPanel React component (1 day)
- [ ] Integrate background questionnaire (0.5 days)
- [ ] **Deliverable**: Modules 2-3 complete + authentication (100/100 base + 50 bonus points)

### Week 4 (Dec 27-30): Bonus Features + Polish
- [ ] Generate Module 4 content (4 chapters) (1 day)
- [ ] Implement content personalization (1 day)
- [ ] Implement Urdu translation (1 day)
- [ ] Create Claude Code Subagents documentation (0.5 days)
- [ ] Deploy backend to Vercel (0.5 days)
- [ ] Create demo video (0.5 days)
- [ ] **Deliverable**: All modules + all bonus features (250+/300 points), submission ready

**Hard Stop**: Nov 30, 6:00 PM - submit regardless of completion status

## Success Metrics

### Phase 0 Completion Criteria
- [ ] Docusaurus 3.x validated for custom components
- [ ] RAG stack (OpenAI + Qdrant + Neon) tested end-to-end
- [ ] Authentication provider selected (better-auth.com or manual JWT)
- [ ] Deployment platform selected (Vercel or Railway)
- [ ] `research.md` file created with findings

### Phase 1 Completion Criteria
- [ ] `data-model.md` created with complete Postgres + Qdrant schemas
- [ ] `api-contracts.md` created with all 7 REST endpoint specs
- [ ] Component architecture diagram created
- [ ] CI/CD workflows defined (.github/workflows/)
- [ ] All architectural decisions documented

### Phase 2 Completion Criteria (Ready for Implementation)
- [ ] All high/medium risks have documented mitigation strategies
- [ ] Development milestones broken down by week
- [ ] Success metrics defined for each phase
- [ ] Plan reviewed and approved (no major unknowns remaining)

## Next Steps

After plan approval:
1. **Create Tasks** (`/sp.tasks`): Break down into testable tasks with red-green-refactor cycles
2. **Begin Phase 0**: Execute research tasks, validate technology choices
3. **Begin Phase 1**: Detailed design (database schema, API contracts)
4. **Begin Implementation**: Set up Docusaurus, start generating content

---

**Plan Status**: ✅ Complete - Ready for Task Breakdown (`/sp.tasks`)
