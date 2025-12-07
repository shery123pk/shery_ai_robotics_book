---
id: 003
title: Create Comprehensive Architectural Plan
stage: plan
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5
feature: physical-ai-textbook
branch: main
user: AEHTRAM
command: planning
labels: ["plan", "architecture", "database-schema", "api-design", "deployment"]
links:
  spec: specs/physical-ai-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/physical-ai-textbook/plan.md
tests:
  - Technical context defined (languages, dependencies, performance goals)
  - Project structure designed (frontend + backend separation)
  - Database schemas documented (Postgres + Qdrant)
  - API contracts specified (7 REST endpoints)
  - Deployment pipeline designed (GitHub Actions + Vercel)
  - Risk mitigation strategies documented
  - 4-week development milestones planned
---

## Prompt

User requested to create a comprehensive architectural plan (Option A) for the hackathon project. The plan needed to:
- Define technical context (languages, dependencies, platforms, performance goals)
- Design complete project structure (frontend Docusaurus + backend FastAPI)
- Specify database schemas (Neon Postgres tables + Qdrant vector collection)
- Define API contracts for all 7 REST endpoints (chat, auth, personalize, translate)
- Design component architecture (React components + FastAPI services)
- Create deployment pipeline (GitHub Pages + Vercel with CI/CD)
- Document risk mitigation strategies (API costs, time constraints, free tier limits)
- Break down 4-week development milestones leading to Nov 30 deadline

The plan follows Spec-Kit Plus template structure with constitution compliance checks and complexity justification.

## Response snapshot

Created comprehensive architectural plan with:

**Technical Stack Defined:**
- Frontend: TypeScript/JavaScript, Docusaurus 3.x, React 18, Node.js 18+
- Backend: Python 3.11+, FastAPI 0.104+, OpenAI SDK, Qdrant Client, asyncpg
- Storage: Qdrant Cloud (vector DB), Neon Postgres (relational DB), GitHub Pages (static assets)
- Deployment: Vercel (backend), GitHub Pages (frontend), GitHub Actions (CI/CD)

**Complete Project Structure:**
- `docs/` - Docusaurus content (4 modules, 12-16 chapters)
- `src/` - Custom React components (ChatBot, PersonalizeButton, TranslateButton, AuthPanel)
- `backend/` - FastAPI service (models, services, API routes, database)
- `scripts/` - Utility scripts (generate embeddings, seed database, validate links)
- `.github/workflows/` - CI/CD automation

**Database Schemas:**
- Postgres tables: `users` (authentication + background), `chat_messages` (chat history), `personalized_content` (cache)
- Qdrant collection: `book_embeddings` (1536-dim vectors with metadata for citations)

**7 REST API Endpoints:**
1. `POST /api/chat` - RAG chatbot (OpenAI + Qdrant retrieval)
2. `POST /api/chat/selection` - Text-selection Q&A with context
3. `GET /api/chat/history` - Retrieve user's chat history (auth required)
4. `POST /api/auth/signup` - Create account with background questionnaire
5. `POST /api/auth/signin` - Authenticate and return JWT token
6. `POST /api/personalize` - Generate personalized chapter (auth required)
7. `POST /api/translate` - Translate chapter to Urdu

**Risk Mitigation:**
- High Risk: OpenAI API costs → caching, model downgrade (GPT-3.5 for translation), rate limiting
- High Risk: Content generation time → Claude Code Subagents, template-driven, 80% threshold
- Medium Risk: Qdrant limits → chunk optimization, selective indexing, fallback to Pinecone
- Medium Risk: Authentication complexity → research first, simple JWT fallback ready

**4-Week Milestones:**
- Week 1 (Dec 6-12): Foundation (Docusaurus + FastAPI skeleton + database setup)
- Week 2 (Dec 13-19): Core Features (Module 1 + RAG chatbot = 50/100 points)
- Week 3 (Dec 20-26): Content + Auth (Modules 2-3 + authentication = 150/300 points)
- Week 4 (Dec 27-30): Bonus Features (Module 4 + personalization + translation + subagents = 250+/300 points)

**Deployment Pipeline:**
- GitHub Actions auto-deploy Docusaurus to GitHub Pages on push to main
- Vercel auto-deploy FastAPI backend from `backend/` directory
- Environment variables stored securely in Vercel secrets

## Outcome

- ✅ Impact: Complete architectural plan ready for task breakdown and implementation
- 🧪 Tests: Constitution compliance verified, all 8 principles satisfied
- 📁 Files: Created specs/physical-ai-textbook/plan.md with detailed architecture
- 🔁 Next prompts: Create implementation tasks (/sp.tasks) with testable acceptance criteria
- 🧠 Reflection: Plan balances thoroughness with hackathon speed constraints; clear separation of frontend (static) and backend (dynamic) enables parallel development; risk mitigation strategies address top 4 failure modes; 4-week timeline realistic with Claude Code automation

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Plan template validation PASS, Constitution check PASS (all 8 principles satisfied)
- Prompt variant (if applicable): Comprehensive single-plan approach (not phased incremental planning)
- Next experiment (smallest change to try): Proceed to task breakdown to define red-green-refactor cycles for TDD implementation
