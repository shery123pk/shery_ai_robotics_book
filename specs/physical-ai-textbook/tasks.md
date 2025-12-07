# Tasks: Physical AI & Humanoid Robotics Textbook

**Input**: Design documents from `/specs/physical-ai-textbook/`
**Prerequisites**: plan.md ✅, spec.md ✅, constitution.md ✅

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, etc.)
- Include exact file paths in descriptions

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [ ] T001 Initialize Docusaurus 3.x project with TypeScript in shery_ai_book/ directory
- [ ] T002 [P] Configure docusaurus.config.js with site metadata, GitHub Pages deployment settings
- [ ] T003 [P] Create .gitignore for node_modules/, build/, .env, backend/__pycache__/
- [ ] T004 [P] Initialize backend/ directory structure (models/, services/, api/, database/, tests/)
- [ ] T005 [P] Create backend/requirements.txt with FastAPI, OpenAI, Qdrant, asyncpg, better-auth dependencies
- [ ] T006 [P] Create backend/config.py for environment variable management (OpenAI key, DB URLs, JWT secret)
- [ ] T007 [P] Create .env.example with placeholder values for all required environment variables
- [ ] T008 [P] Create README.md with project overview, setup instructions, hackathon submission checklist
- [ ] T009 [P] Initialize Git repository and create .github/workflows/ directory for CI/CD

**Checkpoint**: Project structure ready for content and code development

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

### Backend Foundation

- [ ] T010 Create backend/main.py with FastAPI app initialization, CORS middleware configuration
- [ ] T011 Create backend/database/postgres.py with Neon Postgres async connection pool (asyncpg)
- [ ] T012 Create backend/database/qdrant.py with Qdrant Cloud client initialization
- [ ] T013 [P] Create backend/models/database.py with SQLAlchemy async ORM models for users table
- [ ] T014 [P] Create backend/models/auth.py with Pydantic schemas (SignUpRequest, SignInRequest, UserProfile, TokenResponse)
- [ ] T015 [P] Create backend/models/chat.py with Pydantic schemas (ChatRequest, ChatResponse, ChatHistory, SelectionRequest)
- [ ] T016 [P] Create backend/models/content.py with Pydantic schemas (PersonalizeRequest, TranslateRequest, ContentResponse)
- [ ] T017 Create backend/utils/security.py with bcrypt password hashing and JWT token generation/validation functions
- [ ] T018 Create backend/utils/rate_limit.py with decorator for 10 requests/minute rate limiting
- [ ] T019 [P] Create backend/utils/logger.py with structured logging setup (INFO level, JSON format)

### Database Setup

- [ ] T020 Create SQL migration script backend/database/migrations/001_create_users_table.sql
- [ ] T021 Create SQL migration script backend/database/migrations/002_create_chat_messages_table.sql
- [ ] T022 Create SQL migration script backend/database/migrations/003_create_personalized_content_table.sql
- [ ] T023 Run migrations against Neon Postgres database (manual execution or migration runner)
- [ ] T024 Create Qdrant collection 'book_embeddings' with 1536-dim vectors, Cosine distance, payload schema

### Frontend Foundation

- [ ] T025 Create docusaurus.config.js with theme configuration, navbar (logo, module navigation, auth), footer
- [ ] T026 Create sidebars.js with auto-generated sidebar from docs/ structure
- [ ] T027 Create src/css/custom.css for global styles (chatbot, RTL support, button styles)
- [ ] T028 [P] Create src/components/ChatBot/ChatBot.tsx skeleton component (empty UI, no API calls yet)
- [ ] T029 [P] Create src/components/AuthPanel/AuthPanel.tsx skeleton component (SignIn/SignUp forms, no API yet)
- [ ] T030 Configure TypeScript (tsconfig.json) for React 18 and strict mode

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Read Interactive Textbook Content (Priority: P1) 🎯 MVP

**Goal**: Deploy Docusaurus site with all 4 modules (12-16 chapters) to GitHub Pages

**Independent Test**: Visit published GitHub Pages URL, navigate to all modules, verify chapters load with code examples and images

### Content Generation for Module 1: ROS 2

- [ ] T031 [P] [US1] Create docs/intro.md with course overview, learning outcomes, weekly breakdown
- [ ] T032 [P] [US1] Create docs/hardware-requirements.md from hackathon specification
- [ ] T033 [US1] Create docs/module-1/_category_.json with sidebar metadata (label: "Module 1: ROS 2", position: 1)
- [ ] T034 [US1] Generate docs/module-1/01-ros2-intro.md using Claude Code (ROS 2 overview, installation, workspace setup)
- [ ] T035 [US1] Generate docs/module-1/02-nodes-topics.md using Claude Code (nodes, publishers, subscribers, rclpy examples)
- [ ] T036 [US1] Generate docs/module-1/03-services-actions.md using Claude Code (services, actions, client-server patterns)
- [ ] T037 [US1] Generate docs/module-1/04-urdf-robots.md using Claude Code (URDF format, robot description, visualization in RViz2)

### Content Generation for Module 2: Gazebo & Unity

- [ ] T038 [US1] Create docs/module-2/_category_.json with sidebar metadata
- [ ] T039 [US1] Generate docs/module-2/01-simulation-intro.md using Claude Code (physics engines, simulation benefits)
- [ ] T040 [US1] Generate docs/module-2/02-gazebo-physics.md using Claude Code (Gazebo setup, SDF format, physics simulation)
- [ ] T041 [US1] Generate docs/module-2/03-unity-rendering.md using Claude Code (Unity integration, high-fidelity rendering)
- [ ] T042 [US1] Generate docs/module-2/04-sensor-simulation.md using Claude Code (LiDAR, depth cameras, IMUs in simulation)

### Content Generation for Module 3: NVIDIA Isaac

- [ ] T043 [US1] Create docs/module-3/_category_.json with sidebar metadata
- [ ] T044 [US1] Generate docs/module-3/01-isaac-overview.md using Claude Code (Isaac platform overview, capabilities)
- [ ] T045 [US1] Generate docs/module-3/02-isaac-sim.md using Claude Code (Isaac Sim setup, synthetic data generation)
- [ ] T046 [US1] Generate docs/module-3/03-isaac-ros.md using Claude Code (Isaac ROS packages, VSLAM, hardware acceleration)
- [ ] T047 [US1] Generate docs/module-3/04-navigation-nav2.md using Claude Code (Nav2 stack, path planning, obstacle avoidance)

### Content Generation for Module 4: VLA

- [ ] T048 [US1] Create docs/module-4/_category_.json with sidebar metadata
- [ ] T049 [US1] Generate docs/module-4/01-vla-intro.md using Claude Code (Vision-Language-Action overview, convergence of LLMs and robotics)
- [ ] T050 [US1] Generate docs/module-4/02-voice-commands.md using Claude Code (OpenAI Whisper integration, speech-to-text)
- [ ] T051 [US1] Generate docs/module-4/03-llm-planning.md using Claude Code (LLMs for task planning, natural language to ROS actions)
- [ ] T052 [US1] Generate docs/module-4/04-capstone-project.md using Claude Code (autonomous humanoid project, voice command → navigation → manipulation)

### Deployment

- [ ] T053 [US1] Create .github/workflows/deploy-frontend.yml with GitHub Actions workflow for Docusaurus build and GitHub Pages deployment
- [ ] T054 [US1] Configure GitHub repository settings to enable GitHub Pages from gh-pages branch
- [ ] T055 [US1] Test local build with `npm run build` and verify all pages render correctly
- [ ] T056 [US1] Push to main branch, verify GitHub Actions deploys to GitHub Pages successfully
- [ ] T057 [US1] Test published site: verify responsive design on mobile (Chrome DevTools), check all links work

**Checkpoint**: User Story 1 complete - Textbook content deployed and accessible (50/100 base points achieved)

---

## Phase 4: User Story 2 - Ask Questions via RAG Chatbot (Priority: P1) 🎯 MVP

**Goal**: Embed functional RAG chatbot that answers questions about book content with citations

**Independent Test**: Open chatbot on any chapter, ask "What is a ROS 2 node?", verify response cites Module 1 content within 3 seconds

### Backend RAG Service

- [ ] T058 [US2] Create backend/services/embedding_service.py with function to generate OpenAI embeddings (text-embedding-ada-002)
- [ ] T059 [US2] Create backend/services/rag_service.py with RAG pipeline: query → embed → Qdrant search (top 5 results, similarity > 0.7) → LLM response with citations
- [ ] T060 [US2] Implement backend/api/chat.py with `POST /api/chat` endpoint (accepts message, session_id, user_id; returns response + citations)
- [ ] T061 [US2] Implement backend/api/chat.py with `POST /api/chat/selection` endpoint (accepts selected_text, question; returns contextual answer)
- [ ] T062 [US2] Implement backend/api/chat.py with `GET /api/chat/history` endpoint (requires JWT auth, returns user's chat messages with pagination)
- [ ] T063 [US2] Add rate limiting to chat endpoints (10 req/min per user using rate_limit.py decorator)
- [ ] T064 [US2] Add error handling for OpenAI API failures (return 503 with retry-after header)
- [ ] T065 [US2] Update backend/models/database.py with SQLAlchemy model for chat_messages table
- [ ] T066 [US2] Implement chat message persistence: save user message + response + citations to Neon Postgres after each interaction

### Embeddings Generation

- [ ] T067 [US2] Create scripts/generate_embeddings.py script to chunk all markdown files from docs/ (500 tokens per chunk)
- [ ] T068 [US2] Enhance script to extract metadata (module name, chapter_id, section title) from markdown frontmatter and headings
- [ ] T069 [US2] Enhance script to generate embeddings for each chunk using OpenAI API (batch processing, progress bar)
- [ ] T070 [US2] Enhance script to upload embeddings + metadata to Qdrant collection 'book_embeddings'
- [ ] T071 [US2] Run script to populate Qdrant with all book content (monitor: ~640 chunks, verify no duplicates)

### Frontend Chatbot Component

- [ ] T072 [US2] Implement src/components/ChatBot/ChatMessage.tsx component (displays message bubble, user vs assistant styling, citations as links)
- [ ] T073 [US2] Implement src/components/ChatBot/ChatInput.tsx component (textarea, send button, handles Enter key, disabled during API call)
- [ ] T074 [US2] Implement src/components/ChatBot/ChatBot.tsx main component:
  - Collapsible panel (fixed bottom-right position)
  - Message history display with scroll-to-bottom
  - Send message → POST /api/chat → display response
  - Session ID generation (UUID stored in localStorage)
  - Loading state (spinner during API call)
- [ ] T075 [US2] Create src/components/ChatBot/ChatBot.module.css with styles (fixed position, z-index, mobile responsive)
- [ ] T076 [US2] Implement text selection Q&A: add right-click context menu "Ask about this" that opens chatbot with selected text pre-filled
- [ ] T077 [US2] Embed ChatBot component in Docusaurus layout (src/theme/Root.tsx wrapper to inject chatbot on all pages)
- [ ] T078 [US2] Test chatbot: ask 10 questions about different modules, verify citations are correct, measure response time (target < 3s)

**Checkpoint**: User Story 2 complete - RAG chatbot functional (100/100 base points achieved)

---

## Phase 5: User Story 3 - Sign Up and Sign In (Priority: P2) 💰 +50 Bonus Points

**Goal**: Implement authentication with background questionnaire at signup

**Independent Test**: Click "Sign Up", fill form with email/password + background questions, verify account created and auto-login works

### Backend Authentication Service

- [ ] T079 [US3] Research better-auth.com FastAPI integration (read docs, check if custom fields supported)
- [ ] T080 [US3] **Decision Point**: If better-auth.com compatible, implement integration; else proceed with manual JWT auth (T081-T087)
- [ ] T081 [US3] Create backend/services/auth_service.py with `create_user` function (validate email format, hash password with bcrypt, insert to users table)
- [ ] T082 [US3] Enhance auth_service.py with `authenticate_user` function (fetch user by email, verify password hash, generate JWT token with 30-day expiry)
- [ ] T083 [US3] Enhance auth_service.py with `get_current_user` dependency (decode JWT from Authorization header, return user object or 401)
- [ ] T084 [US3] Implement backend/api/auth.py with `POST /api/auth/signup` endpoint (accepts email, password, software_experience, hardware_experience)
- [ ] T085 [US3] Implement backend/api/auth.py with `POST /api/auth/signin` endpoint (accepts email, password; returns access_token and user profile)
- [ ] T086 [US3] Add validation to signup: email must be unique, password min 8 chars, experience levels must be enum values
- [ ] T087 [US3] Add password security: enforce complexity (uppercase, lowercase, number), rate limit signup attempts (5 per hour per IP)

### Frontend Authentication Components

- [ ] T088 [US3] Implement src/components/AuthPanel/SignUpForm.tsx:
  - Email/password input fields
  - Background questionnaire (2 dropdown selects: software experience, hardware experience)
  - Submit button → POST /api/auth/signup → store token in localStorage → update UI
  - Error display for validation failures
- [ ] T089 [US3] Implement src/components/AuthPanel/SignInForm.tsx:
  - Email/password input fields
  - Submit button → POST /api/auth/signin → store token in localStorage → update UI
  - "Forgot password?" link (placeholder, no implementation for hackathon)
- [ ] T090 [US3] Implement src/components/AuthPanel/UserProfile.tsx component (displays username, logout button)
- [ ] T091 [US3] Implement src/components/AuthPanel/AuthPanel.tsx container:
  - Toggle between SignUp and SignIn forms
  - Show UserProfile if logged in (check localStorage for token)
  - Logout button clears token and refreshes UI
- [ ] T092 [US3] Embed AuthPanel in Docusaurus navbar (src/theme/Navbar/index.tsx override)
- [ ] T093 [US3] Create authentication context (src/contexts/AuthContext.tsx) to share user state across components
- [ ] T094 [US3] Test authentication flow: create account → logout → login → verify session persists after browser refresh

**Checkpoint**: User Story 3 complete - Authentication working (150/300 points achieved: 100 base + 50 bonus)

---

## Phase 6: User Story 4 - Personalize Chapter Content (Priority: P2) 💰 +50 Bonus Points

**Goal**: Allow logged-in users to click "Personalize Content" button and see chapter adapted to their background

**Independent Test**: Login as beginner software user, navigate to Module 1 Chapter 2, click "Personalize Content", verify content includes inline definitions and detailed comments

### Backend Personalization Service

- [ ] T095 [US4] Create backend/services/personalize_service.py with `personalize_chapter` function:
  - Input: chapter markdown, user background (software_experience, hardware_experience)
  - Check personalized_content cache (user_id + chapter_id)
  - If cached and < 7 days old, return cached version
  - Else, send to OpenAI API with system prompt: "Adapt this chapter for [background]. Keep code/images/links. Adjust explanations: beginner → add definitions, advanced → skip basics + add tips"
  - Cache result in personalized_content table
  - Return personalized markdown
- [ ] T096 [US4] Implement backend/api/content.py with `POST /api/personalize` endpoint (requires JWT auth, accepts chapter_id + original_markdown)
- [ ] T097 [US4] Add error handling: if LLM fails, return original content with warning "Personalization unavailable, showing original"
- [ ] T098 [US4] Update backend/models/database.py with SQLAlchemy model for personalized_content table
- [ ] T099 [US4] Optimize: use GPT-3.5-turbo for personalization (cheaper than GPT-4, sufficient quality)

### Frontend Personalization Component

- [ ] T100 [US4] Implement src/components/PersonalizeButton/PersonalizeButton.tsx:
  - Button displays "Personalize Content" with user's background in tooltip
  - Visible only if user is logged in
  - Click → POST /api/personalize with current chapter content → replace page content with personalized version
  - Loading state (spinner) during API call
  - "Reset to Original" button appears after personalization (toggle functionality)
- [ ] T101 [US4] Create src/components/PersonalizeButton/PersonalizeButton.module.css with button styles
- [ ] T102 [US4] Inject PersonalizeButton at top of each chapter page (src/theme/DocItem/index.tsx override)
- [ ] T103 [US4] Implement client-side caching: store personalized content in sessionStorage (avoid re-fetching on navigation)
- [ ] T104 [US4] Test personalization with different user backgrounds:
  - Beginner software / No hardware → verify detailed explanations
  - Advanced software / Professional hardware → verify concise, advanced tips
  - Verify code blocks, images, links preserved

**Checkpoint**: User Story 4 complete - Personalization working (200/300 points achieved)

---

## Phase 7: User Story 5 - Translate Content to Urdu (Priority: P2) 💰 +50 Bonus Points

**Goal**: Allow users to click "Translate to Urdu" button and see chapter in Urdu with RTL layout

**Independent Test**: Navigate to any chapter, click "Translate to Urdu", verify Urdu text appears with RTL formatting and English code blocks preserved

### Backend Translation Service

- [ ] T105 [US5] Create backend/services/translate_service.py with `translate_to_urdu` function:
  - Input: chapter markdown
  - Send to OpenAI API with system prompt: "Translate to Urdu (Pakistan). Keep code blocks in English. Keep technical terms in English: ROS 2, NVIDIA Isaac, Gazebo, URDF, rclpy. Preserve markdown formatting. Use formal academic Urdu"
  - Return translated markdown
- [ ] T106 [US5] Implement backend/api/content.py with `POST /api/translate` endpoint (accepts chapter_id + original_markdown, no auth required)
- [ ] T107 [US5] Add error handling: if translation times out (>10s), return error with retry button
- [ ] T108 [US5] Optimize: use GPT-3.5-turbo for translation (cheaper, good quality for Urdu)

### Frontend Translation Component

- [ ] T109 [US5] Implement src/components/TranslateButton/TranslateButton.tsx:
  - Button displays "Translate to Urdu" (or "Show Original English" if already translated)
  - Click → POST /api/translate → replace page content with Urdu version
  - Apply RTL (right-to-left) CSS class to content container
  - Toggle functionality (switch between Urdu and English)
  - Loading state during API call
- [ ] T110 [US5] Create src/components/TranslateButton/TranslateButton.module.css with button styles
- [ ] T111 [US5] Add RTL CSS styles in src/css/custom.css (.rtl class: direction: rtl, text-align: right, preserve code blocks as LTR)
- [ ] T112 [US5] Inject TranslateButton at top of each chapter page (src/theme/DocItem/index.tsx, next to PersonalizeButton)
- [ ] T113 [US5] Implement translation preference persistence: if user translates one chapter, auto-translate next chapters (store preference in sessionStorage)
- [ ] T114 [US5] Test translation: verify Urdu text is readable, code blocks remain in English with syntax highlighting, RTL layout works on mobile

**Checkpoint**: User Story 5 complete - Translation working (250/300 points achieved)

---

## Phase 8: User Story 6 - Use Claude Code Subagents (Priority: P2) 💰 +50 Bonus Points

**Goal**: Create custom Claude Code Subagents and Agent Skills for content generation, document usage

**Independent Test**: Invoke subagent to generate a new chapter, verify it produces correctly formatted markdown with code examples

### Subagent Creation

- [ ] T115 [US6] Create .claude/agents/chapter-generator/ directory structure for custom subagent
- [ ] T116 [US6] Define chapter-generator subagent prompt in .claude/agents/chapter-generator/prompt.md:
  - Input: module name, chapter topic, learning objectives
  - Output: Complete chapter markdown with headings, explanations, code examples (Python/ROS 2), assessment questions
  - Follow constitution principle I (educational excellence): progressive difficulty, clear examples
- [ ] T117 [US6] Create .claude/agents/assessment-generator/ directory for custom subagent
- [ ] T118 [US6] Define assessment-generator subagent prompt:
  - Input: chapter content
  - Output: 5 multiple-choice questions + 3 practical exercises with solutions
  - Align with learning objectives from chapter

### Agent Skills Creation

- [ ] T119 [US6] Create .claude/skills/ros2-code-generator.md skill:
  - Input: node description (e.g., "publisher that sends robot velocity")
  - Output: Complete ROS 2 Python code with rclpy, type hints, best practices, launch file
  - Include installation instructions and dependencies
- [ ] T120 [US6] Create .claude/skills/diagram-generator.md skill:
  - Input: system description (e.g., "ROS 2 publisher-subscriber architecture")
  - Output: Mermaid diagram code for embedding in markdown
  - Include architecture diagrams, sequence diagrams, flowcharts

### Documentation & Validation

- [ ] T121 [US6] Create docs/subagents-and-skills.md documentation page:
  - List all subagents created (chapter-generator, assessment-generator)
  - List all skills created (ros2-code-generator, diagram-generator)
  - Provide usage examples for each with actual invocations
  - Document time saved (estimate: manual chapter = 2 hours, subagent = 15 minutes)
- [ ] T122 [US6] Test chapter-generator: invoke to create a new chapter on "ROS 2 Launch Files", verify output quality
- [ ] T123 [US6] Test assessment-generator: invoke on Module 1 Chapter 2 content, verify questions are relevant and correctly formatted
- [ ] T124 [US6] Test ros2-code-generator: invoke to create "IMU sensor subscriber node", verify code runs (syntax check)
- [ ] T125 [US6] Update README.md with section on subagents/skills: how to invoke, expected inputs/outputs

**Checkpoint**: User Story 6 complete - Subagents documented (300/300 points achieved: 100 base + 200 bonus)

---

## Phase 9: Deployment & Final Polish

**Purpose**: Production deployment, CI/CD automation, final quality checks

### Backend Deployment

- [ ] T126 Create vercel.json configuration for FastAPI deployment (Python runtime, environment variables, routes)
- [ ] T127 Configure Vercel secrets for environment variables (OPENAI_API_KEY, QDRANT_URL, NEON_DATABASE_URL, JWT_SECRET)
- [ ] T128 Deploy backend to Vercel: connect GitHub repo, trigger deployment, verify API endpoints accessible
- [ ] T129 Update frontend API calls to use production backend URL (environment variable REACT_APP_API_URL)
- [ ] T130 Test production backend: send test requests to all 7 endpoints, verify responses and error handling

### CI/CD Automation

- [ ] T131 Update .github/workflows/deploy-frontend.yml to trigger on push to main (docs/, src/, docusaurus.config.js changes)
- [ ] T132 Create .github/workflows/deploy-backend.yml (optional if Vercel auto-deploys from GitHub)
- [ ] T133 Test CI/CD: make dummy change to docs/intro.md, push to main, verify auto-deployment to GitHub Pages

### Quality Checks

- [ ] T134 Run Lighthouse audit on published site: verify performance > 90, accessibility > 90, SEO > 90
- [ ] T135 Validate all external links using scripts/validate_links.py (check for 404s, update broken links)
- [ ] T136 Spell check all markdown content (use VSCode spell checker, fix typos in technical content)
- [ ] T137 Test mobile responsiveness: verify all pages work on iPhone/Android viewport (Chrome DevTools)
- [ ] T138 Security audit: verify no API keys in git history, .env in .gitignore, HTTPS only for API calls
- [ ] T139 Test chatbot accuracy: ask 50 questions (10 per module), manually verify responses are correct, update embeddings if needed

### Demo Video & Submission

- [ ] T140 Create demo video script (90 seconds max):
  - 0-15s: Book homepage, navigate to Module 1
  - 15-30s: Open chatbot, ask question, show response with citation
  - 30-45s: Demonstrate text-selection Q&A
  - 45-60s: Show authentication (signup with questionnaire)
  - 60-75s: Personalize content (show before/after)
  - 75-90s: Translate to Urdu (show RTL layout)
- [ ] T141 Record demo video using OBS/QuickTime (screen capture + voiceover or use NotebookLM for narration)
- [ ] T142 Upload demo video to YouTube (unlisted link)
- [ ] T143 Create submission checklist in README.md:
  - ✅ Public GitHub repo link
  - ✅ Published book URL (GitHub Pages)
  - ✅ Demo video link (YouTube unlisted)
  - ✅ WhatsApp number for live presentation invitation
- [ ] T144 Fill out Google Form submission: https://forms.gle/CQsSEGM3GeCrL43c8 (before Nov 30, 6:00 PM)

**Checkpoint**: Project complete and submitted

---

## Dependencies & Execution Order

### Phase Dependencies

- **Phase 1 (Setup)**: No dependencies - can start immediately ✅
- **Phase 2 (Foundational)**: Depends on Setup completion - BLOCKS all user stories
- **Phase 3 (US1: Textbook Content)**: Depends on Foundational (frontend foundation only)
- **Phase 4 (US2: RAG Chatbot)**: Depends on Foundational (full backend + frontend) + US1 (content needed for embeddings)
- **Phase 5 (US3: Authentication)**: Depends on Foundational (backend foundation) - Can run in parallel with US1/US2 content generation
- **Phase 6 (US4: Personalization)**: Depends on US3 (requires auth) + US1 (requires content)
- **Phase 7 (US5: Translation)**: Depends on US1 (requires content) - Does NOT require auth, can run in parallel with US3
- **Phase 8 (US6: Subagents)**: Can run in parallel with all other stories (documentation task)
- **Phase 9 (Deployment)**: Depends on desired user stories being complete

### Recommended Execution Order (Solo Developer)

**Week 1 (Dec 6-12): Foundation + Content**
1. Phase 1: Setup (T001-T009) - 1 day
2. Phase 2: Foundational (T010-T030) - 2 days
3. Phase 3: User Story 1 - Module 1 only (T031-T037) - 1 day
4. Phase 4: User Story 2 - Backend + embeddings (T058-T071) - 2 days
5. Deploy backend to Vercel (T126-T128) - 0.5 days

**Week 2 (Dec 13-19): Core Features Complete**
1. Phase 3: User Story 1 - Modules 2-4 (T038-T052) - 3 days
2. Phase 4: User Story 2 - Frontend chatbot (T072-T078) - 2 days
3. Phase 3: Deploy frontend (T053-T057) - 0.5 days
4. **Checkpoint**: Test MVP (textbook + chatbot) - 100/100 base points

**Week 3 (Dec 20-26): Bonus Features**
1. Phase 5: User Story 3 - Authentication (T079-T094) - 2 days
2. Phase 6: User Story 4 - Personalization (T095-T104) - 1.5 days
3. Phase 7: User Story 5 - Translation (T105-T114) - 1.5 days
4. **Checkpoint**: Test all bonus features - 250/300 points

**Week 4 (Dec 27-30): Polish + Submission**
1. Phase 8: User Story 6 - Subagents documentation (T115-T125) - 1 day
2. Phase 9: Quality checks (T134-T139) - 1 day
3. Phase 9: Demo video + submission (T140-T144) - 1 day
4. **Hard Stop**: Submit by Nov 30, 6:00 PM

### Parallel Opportunities

If working in a team or using Claude Code efficiently:
- **After Foundational phase completes**:
  - Developer/Agent A: US1 (content generation) in parallel with:
  - Developer/Agent B: US2 (backend RAG) in parallel with:
  - Developer/Agent C: US3 (authentication)
- **All setup tasks marked [P]** can run in parallel (T002, T003, T004, T005, T006, T007, T008, T009)
- **All foundational backend models** (T013, T014, T015, T016) can run in parallel
- **All Module 1 chapters** (T034-T037) can run in parallel using Claude Code
- **Personalization + Translation** (US4 + US5) can run in parallel (different files, no dependencies)

---

## Implementation Strategy

### MVP First (100 Base Points)

1. Complete Phase 1: Setup → Phase 2: Foundational
2. Complete Phase 3: User Story 1 (textbook content)
3. Complete Phase 4: User Story 2 (RAG chatbot)
4. **STOP and VALIDATE**: Test textbook + chatbot independently
5. Deploy to GitHub Pages + Vercel
6. **Submit if time runs out** (100/100 base points guaranteed)

### Incremental Bonus Features (200 Bonus Points)

After MVP is solid:
1. Add Phase 8: Subagents documentation first (easiest 50 bonus points, helps with content generation)
2. Add Phase 5: Authentication (unlocks personalization)
3. Add Phase 6: Personalization (depends on auth)
4. Add Phase 7: Translation (independent, can add anytime)

### Emergency Plan (If Behind Schedule by Nov 28)

**Priority 1** (100 points): Textbook + Chatbot only
- Skip all bonus features
- Reduce to 8 chapters total (2 per module)
- Deploy MVP, create basic demo video

**Priority 2** (+50 points): Add Subagents only
- Document existing Claude Code usage for content generation
- Skip auth/personalization/translation

---

## Notes

- [P] tasks = different files, no dependencies, can run in parallel
- [Story] label maps task to specific user story (US1-US6)
- Each user story should be independently completable and testable
- Commit after each task or logical group (granular commits help debugging)
- Stop at any checkpoint to validate story independently
- Constitution principle: Speed over perfection (80% complete > 100% polished)
- Hard deadline: Nov 30, 2025, 6:00 PM - submit regardless of completion status

---

**Tasks Status**: ✅ Complete - Ready for Implementation (144 tasks defined)
