# Feature Specification: Physical AI & Humanoid Robotics Textbook

**Feature Branch**: `main`
**Created**: 2025-12-06
**Status**: Active Development
**Input**: Hackathon I - Create textbook for Physical AI & Humanoid Robotics course with RAG chatbot, authentication, personalization, and translation features

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Read Interactive Textbook Content (Priority: P1)

A student visits the published textbook website to learn about Physical AI and Humanoid Robotics. They navigate through modules on ROS 2, Gazebo/Unity, NVIDIA Isaac, and Vision-Language-Action systems, reading chapters with code examples and diagrams.

**Why this priority**: Core deliverable worth 50/100 base points. Without this, there is no textbook.

**Independent Test**: Can be fully tested by deploying Docusaurus site to GitHub Pages and navigating all chapters. Delivers educational value immediately.

**Acceptance Scenarios**:

1. **Given** user visits the published book URL, **When** they land on the homepage, **Then** they see course overview, learning outcomes, and navigation to all 4 modules
2. **Given** user is on any chapter page, **When** they scroll through content, **Then** they see formatted text, code examples with syntax highlighting, and images with alt text
3. **Given** user is on mobile device, **When** they view any page, **Then** layout adapts responsively without horizontal scrolling
4. **Given** user navigates between chapters, **When** page loads, **Then** load time is under 2 seconds (95th percentile)

---

### User Story 2 - Ask Questions via RAG Chatbot (Priority: P1)

A student reading about ROS 2 nodes has a question about publisher-subscriber patterns. They open the embedded chatbot, type their question, and receive an answer based on the textbook content with citations.

**Why this priority**: Core deliverable worth 50/100 base points. RAG chatbot is mandatory for base functionality.

**Independent Test**: Can be tested by embedding chatbot component in Docusaurus, asking questions about any chapter content, and verifying responses are grounded in book text.

**Acceptance Scenarios**:

1. **Given** user is reading any chapter, **When** they click the chatbot icon, **Then** chatbot panel opens with conversation history (if logged in) or fresh session
2. **Given** chatbot is open, **When** user types a question about book content and submits, **Then** response appears within 3 seconds citing relevant sections
3. **Given** user selects text in the chapter, **When** they right-click and choose "Ask about this", **Then** chatbot opens with selected text as context for Q&A
4. **Given** chatbot receives question outside book scope, **When** it responds, **Then** it includes disclaimer "This answer uses general knowledge, not book content"
5. **Given** user is not logged in, **When** they use chatbot, **Then** conversation is session-based only (no persistence)

---

### User Story 3 - Sign Up and Sign In (Priority: P2)

A new student visits the textbook site and decides to create an account to save their progress and chatbot history. They sign up with email/password, answer background questions, and access personalized features.

**Why this priority**: Bonus feature worth 50 points. Enables personalization and persistent chatbot history. Critical for user engagement.

**Independent Test**: Can be tested by implementing better-auth.com integration, creating test accounts, and verifying authentication state persists across sessions.

**Acceptance Scenarios**:

1. **Given** user visits site without account, **When** they click "Sign Up", **Then** they see registration form with email, password, and background questionnaire
2. **Given** user fills signup form, **When** they submit, **Then** background questions appear asking: "What is your software development experience? (Beginner/Intermediate/Advanced)" and "What is your hardware/robotics experience? (None/Hobbyist/Professional)"
3. **Given** user completes signup, **When** account is created, **Then** they receive confirmation and are automatically logged in
4. **Given** user has account, **When** they click "Sign In" and enter credentials, **Then** they are authenticated and see personalized UI (username, logout button)
5. **Given** user is logged in, **When** they close browser and return, **Then** session persists for 30 days (remember me enabled by default)

---

### User Story 4 - Personalize Chapter Content (Priority: P2)

A student with beginner software experience but no robotics background starts reading Module 1 (ROS 2). They click the "Personalize Content" button at the chapter start, and the content adapts to include more foundational explanations and simplified code examples.

**Why this priority**: Bonus feature worth 50 points. Significantly improves learning experience by adapting to user background.

**Independent Test**: Can be tested by logging in with different background profiles, clicking personalize button, and verifying content changes appropriately.

**Acceptance Scenarios**:

1. **Given** logged-in user starts a chapter, **When** they see the "Personalize Content" button at the top, **Then** button shows their current profile (e.g., "Personalized for: Beginner Software / No Robotics")
2. **Given** user clicks "Personalize Content", **When** personalization runs, **Then** API call sends chapter content + user background to LLM, which returns adapted version
3. **Given** user has beginner background, **When** content is personalized, **Then** technical jargon includes inline definitions and code examples have detailed comments
4. **Given** user has advanced background, **When** content is personalized, **Then** content skips basic explanations and includes advanced tips and optimization techniques
5. **Given** personalization completes, **When** user views chapter, **Then** original content is replaced with personalized version, with option to "Reset to Original"

---

### User Story 5 - Translate Content to Urdu (Priority: P2)

A student who prefers learning in Urdu visits the textbook. At the start of each chapter, they click the "Translate to Urdu" button, and the chapter content is translated while preserving code blocks and technical terms.

**Why this priority**: Bonus feature worth 50 points. Expands accessibility to Urdu-speaking learners in Pakistan and globally.

**Independent Test**: Can be tested by clicking translate button on any chapter and verifying Urdu text appears with proper RTL formatting and preserved code blocks.

**Acceptance Scenarios**:

1. **Given** user starts any chapter, **When** they see the "Translate to Urdu" button, **Then** button is visible and clickable
2. **Given** user clicks "Translate to Urdu", **When** translation runs, **Then** API call sends chapter content to LLM with instruction to translate to Urdu while preserving code blocks, technical terms (ROS 2, NVIDIA Isaac), and formatting
3. **Given** translation completes, **When** user views chapter, **Then** text appears in Urdu with RTL (right-to-left) layout, code blocks remain in English with syntax highlighting intact
4. **Given** chapter is translated, **When** user clicks "Show Original English", **Then** content reverts to English instantly (toggle functionality)
5. **Given** user translates chapter, **When** they navigate to another chapter, **Then** translation preference persists (all chapters show Urdu)

---

### User Story 6 - Use Claude Code Subagents for Content Generation (Priority: P2)

During development, the textbook creator uses custom Claude Code Subagents to generate repetitive content structures (chapter templates, code examples, assessment questions) and Agent Skills for reusable components.

**Why this priority**: Bonus feature worth 50 points. Demonstrates advanced Claude Code usage and accelerates development.

**Independent Test**: Can be tested by creating subagent definitions, invoking them during content generation, and measuring time saved vs manual writing.

**Acceptance Scenarios**:

1. **Given** developer needs to create a new chapter, **When** they invoke `/chapter-generator` subagent with module name and topic, **Then** subagent generates chapter template with learning objectives, sections, and placeholder code blocks
2. **Given** developer needs ROS 2 code examples, **When** they invoke `ros2-code-skill` with node description, **Then** skill generates Python ROS 2 node code following best practices (rclpy, type hints, launch file)
3. **Given** developer needs assessment questions, **When** they invoke `/assessment-generator` subagent with chapter content, **Then** subagent generates 5 multiple-choice questions and 3 practical exercises
4. **Given** subagents are used, **When** final submission is evaluated, **Then** project includes documentation of subagents/skills created with usage examples

---

### Edge Cases

- **What happens when chatbot API is down?** UI shows "Chatbot temporarily unavailable. Please try again later." and allows reading content without disruption.
- **What happens when user selects non-text content (image) for chatbot Q&A?** System ignores selection and prompts "Please select text to ask questions about."
- **What happens when translation API times out?** UI shows error "Translation failed. Please try again." with retry button, content remains in English.
- **What happens when user has no internet during personalization?** Button shows "Offline - personalization unavailable" and is disabled.
- **What happens when user tries to access logged-in features without account?** System redirects to login page with message "Please sign in to access this feature."
- **What happens when vector database (Qdrant) is empty?** Chatbot responds "I don't have enough information to answer that yet. Please check back later."
- **What happens when user asks chatbot about content not yet in the book?** Chatbot responds "I couldn't find information about that in the textbook. This might be covered in a future update."

## Requirements *(mandatory)*

### Functional Requirements

#### Book Content (Module 1-4)

- **FR-001**: Book MUST include comprehensive content for Module 1: ROS 2 (Robotic Nervous System) covering nodes, topics, services, rclpy, and URDF
- **FR-002**: Book MUST include comprehensive content for Module 2: Gazebo & Unity (Digital Twin) covering physics simulation, sensors, and environment building
- **FR-003**: Book MUST include comprehensive content for Module 3: NVIDIA Isaac covering Isaac Sim, Isaac ROS, VSLAM, and Nav2
- **FR-004**: Book MUST include comprehensive content for Module 4: Vision-Language-Action (VLA) covering Whisper, LLMs for planning, and capstone project
- **FR-005**: Each module MUST include learning outcomes, weekly breakdown, code examples, and self-assessment questions
- **FR-006**: All code examples MUST be syntactically correct and include version information for dependencies
- **FR-007**: Book MUST include introduction, hardware requirements section, and weekly breakdown (13 weeks total)

#### Docusaurus Site

- **FR-008**: Site MUST be built using Docusaurus v3 with responsive design
- **FR-009**: Site MUST be deployed to GitHub Pages with public access
- **FR-010**: Site MUST have navigation sidebar with all modules and chapters organized hierarchically
- **FR-011**: Site MUST support dark mode and light mode themes
- **FR-012**: All pages MUST have meta tags for SEO (title, description, keywords)
- **FR-013**: Site MUST include search functionality (Docusaurus default or Algolia)

#### RAG Chatbot

- **FR-014**: Chatbot MUST use OpenAI GPT-4 Turbo via ChatKit SDK or OpenAI Agents SDK
- **FR-015**: Chatbot MUST use Qdrant Cloud Free Tier for vector storage of book content embeddings
- **FR-016**: Chatbot MUST use Neon Serverless Postgres for storing user chat history
- **FR-017**: Chatbot MUST support text-selection-based Q&A: user selects text, right-clicks, and asks question with that context
- **FR-018**: Chatbot responses MUST cite source sections (e.g., "According to Module 1, Chapter 3...")
- **FR-019**: Chatbot MUST maintain conversation context within a session (previous questions/answers)
- **FR-020**: Chatbot MUST handle rate limiting gracefully (show user-friendly error if API quota exceeded)
- **FR-021**: Chatbot UI MUST be embedded in Docusaurus site as a fixed-position panel (collapsible)

#### Authentication (Bonus Feature)

- **FR-022**: System MUST implement authentication using better-auth.com with email/password method
- **FR-023**: Signup form MUST include background questionnaire with two questions:
  - "What is your software development experience?" (Options: Beginner / Intermediate / Advanced)
  - "What is your hardware/robotics experience?" (Options: None / Hobbyist / Professional)
- **FR-024**: System MUST store user profiles (email, hashed password, software_level, hardware_level) in Neon Postgres
- **FR-025**: System MUST support OAuth login via Google (optional but recommended)
- **FR-026**: System MUST implement session management with 30-day expiry
- **FR-027**: System MUST protect chatbot history endpoints (only logged-in users can access their own history)

#### Content Personalization (Bonus Feature)

- **FR-028**: Each chapter page MUST display "Personalize Content" button at the top (visible only to logged-in users)
- **FR-029**: Personalization API MUST send chapter markdown + user background to LLM with instruction to adapt complexity
- **FR-030**: Personalized content MUST preserve all code blocks, images, and links from original
- **FR-031**: System MUST cache personalized content per user per chapter (avoid re-generating on every visit)
- **FR-032**: User MUST be able to toggle between personalized and original content with "Reset to Original" button

#### Urdu Translation (Bonus Feature)

- **FR-033**: Each chapter page MUST display "Translate to Urdu" button at the top
- **FR-034**: Translation API MUST send chapter markdown to LLM with instruction to translate to Urdu while preserving:
  - Code blocks (keep in English)
  - Technical terms (ROS 2, NVIDIA Isaac, Gazebo, URDF, etc.)
  - Markdown formatting (headings, lists, links)
- **FR-035**: Translated content MUST render with RTL (right-to-left) text direction for Urdu
- **FR-036**: User MUST be able to toggle between Urdu and English with "Show Original English" button
- **FR-037**: Translation preference MUST persist across chapters within session

#### Claude Code Subagents & Skills (Bonus Feature)

- **FR-038**: Project MUST include at least 2 custom Claude Code Subagents for content generation tasks
- **FR-039**: Project MUST include at least 2 Agent Skills for reusable components (e.g., code generation, assessment questions)
- **FR-040**: Subagents and skills MUST be documented in project README with usage examples
- **FR-041**: Subagents MUST demonstrate measurable productivity improvement (document time saved)

#### Backend API

- **FR-042**: Backend MUST be built with FastAPI (Python) with async endpoints
- **FR-043**: Backend MUST expose the following endpoints:
  - `POST /api/chat` - Send message to RAG chatbot, returns response with citations
  - `POST /api/chat/selection` - Send selected text + question, returns contextual answer
  - `GET /api/chat/history` - Retrieve user's chat history (requires auth)
  - `POST /api/auth/signup` - Create new user account with background questions
  - `POST /api/auth/signin` - Authenticate user, return session token
  - `POST /api/personalize` - Generate personalized chapter content (requires auth)
  - `POST /api/translate` - Translate chapter content to Urdu
- **FR-044**: Backend MUST implement rate limiting (10 requests/minute per user for chat endpoints)
- **FR-045**: Backend MUST validate all inputs and return appropriate HTTP status codes (400, 401, 429, 500)
- **FR-046**: Backend MUST log all API requests with timestamps for analytics

#### Deployment

- **FR-047**: Frontend MUST be deployed to GitHub Pages with custom domain support (optional)
- **FR-048**: Backend MUST be deployed to Vercel or Railway free tier
- **FR-049**: Environment variables (API keys, database URLs) MUST be stored securely (not in git)
- **FR-050**: GitHub Actions MUST automate deployment (push to main → auto-deploy)

### Key Entities

- **User**: Represents a student account with email, password_hash, software_experience_level (beginner/intermediate/advanced), hardware_experience_level (none/hobbyist/professional), created_at timestamp
- **ChatMessage**: Represents a single chatbot interaction with user_id (nullable if not logged in), session_id, message_text, response_text, citations (JSON array of source sections), timestamp
- **Chapter**: Represents book content with module_id, chapter_id, title, markdown_content, embeddings_generated (boolean flag)
- **PersonalizedContent**: Cached personalized chapter content with user_id, chapter_id, personalized_markdown, generated_at timestamp
- **VectorEmbedding**: Represents text chunks from book content stored in Qdrant with chapter_id, chunk_text, embedding_vector (1536 dimensions for OpenAI), metadata (module name, chapter title, section)

## Success Criteria *(mandatory)*

### Measurable Outcomes

#### Base Features (100 Points Total)

- **SC-001**: Docusaurus book deployed to public GitHub Pages URL with all 4 modules visible in navigation (50 points)
- **SC-002**: Each module contains at least 3 chapters with formatted markdown, code examples, and images (evaluated as part of 50 points)
- **SC-003**: RAG chatbot embedded in site responds to user questions within 3 seconds with citations to book content (50 points)
- **SC-004**: Text-selection Q&A feature works: user can select text, ask question, and receive contextual answer (evaluated as part of 50 points)
- **SC-005**: Site achieves Lighthouse performance score > 90 on desktop

#### Bonus Features (Up to 200 Points Total)

- **SC-006**: At least 2 custom Claude Code Subagents and 2 Agent Skills documented with usage examples (50 points)
- **SC-007**: Authentication system implemented with better-auth.com, including signup with background questionnaire (50 points)
- **SC-008**: Content personalization feature works: logged-in users can click "Personalize Content" and see adapted chapter text (50 points)
- **SC-009**: Urdu translation feature works: users can click "Translate to Urdu" and see chapter content in Urdu with RTL layout (50 points)

#### Hackathon-Specific

- **SC-010**: Public GitHub repository URL submitted with README containing setup instructions
- **SC-011**: Demo video under 90 seconds showcasing: book navigation, chatbot Q&A, text-selection, and at least 1 bonus feature
- **SC-012**: Submission completed before deadline: Sunday, Nov 30, 2025 at 6:00 PM
- **SC-013**: All bonus features attempted score higher than partial implementation of fewer features (per constitution: Subagents > Auth > Personalization > Translation priority)

#### User Experience

- **SC-014**: 90% of test users can navigate to Module 3 content within 30 seconds of landing on homepage
- **SC-015**: Chatbot answers at least 80% of questions about book content correctly (based on manual evaluation of 50 test questions)
- **SC-016**: Mobile users can read chapters without horizontal scrolling or broken layouts
- **SC-017**: Zero JavaScript errors in browser console on critical user paths (homepage, chapter reading, chatbot)

#### Technical Performance

- **SC-018**: Backend API p95 latency < 2 seconds for chat endpoints under normal load (5 concurrent requests)
- **SC-019**: Vector database contains embeddings for 100% of book content (all chapters chunked and indexed)
- **SC-020**: Authentication system handles signup, login, and session validation without errors (100% success rate in testing)
- **SC-021**: Total hosting costs remain at $0/month using free tiers (Vercel/Railway, Neon, Qdrant, GitHub Pages)

## Out of Scope

The following are explicitly NOT part of this project:

- **Real-time collaboration**: Multiple users editing or annotating content simultaneously
- **Course enrollment system**: No student registration for formal courses, payment processing, or certificates
- **Video content**: No embedded video lectures or tutorials (only text, images, and code)
- **Interactive code execution**: No in-browser Python/ROS 2 code execution environment
- **Mobile native apps**: Web-only, no iOS/Android native applications
- **Admin dashboard**: No content management system for editing chapters via UI (content edited in markdown files via git)
- **Advanced analytics**: No user behavior tracking beyond basic page views and chatbot usage counts
- **Multi-language support beyond Urdu**: No Spanish, Arabic, Chinese, etc. translations
- **Offline mode**: No PWA (Progressive Web App) with offline reading capability
- **Community features**: No forums, comments, or user-generated content
- **Gamification**: No badges, leaderboards, or progress tracking

## Technical Constraints

- **Budget**: $0/month hosting costs (must use free tiers only)
- **Performance**: Page load time < 2 seconds (p95), chatbot response < 3 seconds (p95)
- **Browser support**: Modern browsers only (Chrome 90+, Firefox 88+, Safari 14+, Edge 90+)
- **Accessibility**: WCAG 2.1 Level A compliance minimum (alt text, keyboard navigation)
- **Security**: No plaintext passwords, API keys in environment variables, HTTPS only
- **Content quality**: All code examples must be tested on Ubuntu 22.04 with ROS 2 Humble/Iron
- **Deadline**: Hard stop at Nov 30, 2025, 6:00 PM - incomplete features acceptable if core + some bonuses delivered

## Dependencies

### External Services (Free Tiers Required)

- **OpenAI API**: GPT-4 Turbo for chatbot, personalization, and translation (need API key with credits)
- **Qdrant Cloud**: Free tier (1GB storage) for vector embeddings
- **Neon Serverless Postgres**: Free tier (0.5GB storage, 1 database) for user data and chat history
- **better-auth.com**: Free tier for authentication (need to verify API limits)
- **GitHub Pages**: Free for public repositories
- **Vercel or Railway**: Free tier for backend API deployment

### Development Tools

- **Node.js**: v18+ for Docusaurus build
- **Python**: 3.11+ for FastAPI backend
- **Claude Code**: Sonnet 4.5 with Spec-Kit Plus for content generation
- **Git**: Version control and deployment automation

### Third-Party Libraries

- **Frontend**: Docusaurus 3.x, React 18, react-i18next (if needed for Urdu)
- **Backend**: FastAPI, OpenAI Python SDK, Qdrant client, asyncpg (Postgres), better-auth SDK
- **Development**: ESLint, Prettier, Black, Ruff, pytest

## Risk Analysis

### High Risks

1. **OpenAI API Costs**: GPT-4 Turbo usage for chatbot + personalization + translation could exceed budget
   - **Mitigation**: Use GPT-3.5-turbo for non-critical features, implement aggressive caching, set monthly spending limits

2. **Content Generation Time**: Writing comprehensive content for 4 modules manually may exceed deadline
   - **Mitigation**: Use Claude Code Subagents for templated content, prioritize breadth over depth (3-4 chapters per module minimum)

3. **RAG Accuracy**: Chatbot may hallucinate or provide incorrect answers
   - **Mitigation**: Use retrieval with high similarity thresholds (0.7+), include citations, add disclaimer for uncertain answers

### Medium Risks

4. **Free Tier Limits**: Qdrant/Neon/Vercel free tiers may have strict limits that are hit during development
   - **Mitigation**: Monitor usage closely, have backup plan to switch providers if needed (e.g., Pinecone → Qdrant fallback)

5. **Authentication Security**: Implementing auth from scratch could introduce vulnerabilities
   - **Mitigation**: Use better-auth.com library (trusted, audited), follow security best practices (HTTPS, bcrypt, session tokens)

6. **Mobile Responsiveness**: Complex layouts with chatbot panel may break on small screens
   - **Mitigation**: Test early on mobile (Chrome DevTools), use CSS Grid/Flexbox, make chatbot full-screen on mobile

### Low Risks

7. **Translation Quality**: Urdu translation may have grammatical errors or poor phrasing
   - **Mitigation**: Include disclaimer "AI-generated translation, may contain errors", allow toggle to English

8. **Browser Compatibility**: Site may not work on older browsers
   - **Mitigation**: Document browser requirements in README, focus on modern browsers (90%+ market share)

## Next Steps (After Spec Approval)

1. **Create Plan** (`/sp.plan`): Architect technical implementation (component structure, API design, database schema, deployment pipeline)
2. **Create Tasks** (`/sp.tasks`): Break down into testable tasks with red-green-refactor cycles
3. **Set Up Docusaurus**: Initialize project, configure theme, create module structure
4. **Generate Module Content**: Use Claude Code to draft chapters for all 4 modules
5. **Build Backend API**: Implement FastAPI endpoints, integrate OpenAI/Qdrant/Neon
6. **Embed Chatbot**: Create React component, integrate with backend
7. **Implement Authentication**: Integrate better-auth.com, create signup/signin UI
8. **Add Personalization**: Create personalization API, add UI buttons
9. **Add Translation**: Create translation API, add UI buttons with RTL support
10. **Create Subagents/Skills**: Document reusable Claude Code components
11. **Deploy**: Push to GitHub Pages and Vercel/Railway
12. **Create Demo Video**: Record 90-second walkthrough
13. **Submit**: Fill out Google Form before Nov 30, 6:00 PM

---

**Specification Status**: ✅ Complete - Ready for Planning Phase
