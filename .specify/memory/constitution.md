<!--
Sync Impact Report:
Version: Template → 1.0.0
Type: Initial Constitution
Changes:
  - Created comprehensive constitution for Physical AI & Humanoid Robotics Textbook
  - Defined 8 core principles covering education, technical accuracy, and AI-native development
  - Established technology stack requirements
  - Set quality gates and development workflow
Templates Updated: ✅ All templates aligned
Follow-up: None
-->

# Physical AI & Humanoid Robotics Textbook Constitution

## Core Principles

### I. Educational Excellence (NON-NEGOTIABLE)
All content MUST prioritize student learning outcomes above all else:
- Clear learning objectives for each module and chapter
- Progressive difficulty: foundational concepts before advanced topics
- Practical examples with real-world applications
- Visual aids (diagrams, code examples, simulations) required for complex concepts
- Self-assessment questions at chapter end
- No assumed knowledge without explicit prerequisites stated

**Rationale**: This is an educational textbook; pedagogy drives all content decisions.

### II. Technical Accuracy & Currency
All technical content MUST be verified and current:
- Code examples must run on specified versions (ROS 2 Humble/Iron, Ubuntu 22.04, NVIDIA Isaac Sim latest)
- Hardware specifications must reflect 2025 availability and pricing
- API documentation links must be validated before publication
- Deprecated approaches must be flagged with migration guidance
- All claims about robotics/AI capabilities must cite sources or demonstrations

**Rationale**: Outdated or incorrect robotics content can lead to project failures and student frustration.

### III. AI-Native Content Generation
Content creation leverages AI tools while maintaining quality:
- Use Claude Code + Spec-Kit Plus for structured content generation
- AI-generated content MUST be reviewed for technical accuracy
- Source material (research papers, official docs) MUST be cited
- Custom Claude Code Subagents for repetitive content tasks
- Agent Skills for reusable textbook components (code blocks, diagrams, assessments)

**Rationale**: AI accelerates content creation while Spec-Kit Plus ensures consistency and traceability.

### IV. Modularity & Composability
Content organized in discrete, reusable modules:
- Each module (ROS 2, Gazebo, Isaac, VLA) is self-contained
- Chapters can be read independently with clear prerequisite links
- Code examples bundled per module with installation scripts
- Weekly breakdown aligns with university semester structure (13 weeks)
- Module dependencies explicitly documented in introduction

**Rationale**: Enables instructors to customize course sequences and students to navigate non-linearly.

### V. Interactive AI Learning (RAG Chatbot)
Embedded RAG chatbot enhances student engagement:
- Chatbot MUST answer questions using book content as primary source
- Text-selection Q&A: users can select text and ask contextual questions
- Response latency < 3 seconds for 95th percentile queries
- Fallback to general knowledge when book content insufficient (with disclaimer)
- Conversation history preserved per user session
- Feedback mechanism for incorrect/unhelpful responses

**Rationale**: 24/7 AI tutor reduces barriers to learning and provides immediate clarification.

### VI. Accessibility & Personalization
Content adapts to diverse learner backgrounds:
- User background questionnaire at signup (software/hardware experience level)
- Personalization button per chapter: adjusts complexity and examples based on background
- Urdu translation support for non-English speakers (translation button per chapter)
- Alternative text for all images and diagrams
- Mobile-responsive design for Docusaurus site
- Keyboard navigation support

**Rationale**: Robotics education should be accessible regardless of prior experience or language.

### VII. Code Quality & Reproducibility
All code examples must be production-ready:
- Follow ROS 2 best practices (launch files, parameter YAML, package structure)
- Python code adheres to PEP 8 with type hints
- Dockerfile/installation scripts provided for environment setup
- Version pinning for all dependencies (prevent "works on my machine" issues)
- Security: no hardcoded credentials, API keys in .env with examples

**Rationale**: Students learn professional practices from examples; broken code erodes trust.

### VIII. Deployment & Performance
Book site must be fast, reliable, and cost-effective:
- GitHub Pages deployment with CDN (< 2s page load globally)
- Backend API (FastAPI) hosted on free tier services (Vercel/Railway)
- Database costs < $0/month (Neon Free Tier + Qdrant Cloud Free Tier)
- Graceful degradation: chatbot unavailable should not break reading experience
- Analytics: track most-read chapters and chatbot usage patterns

**Rationale**: Zero-cost hosting enables wide distribution; performance impacts learning experience.

## Technology Stack Requirements

### Frontend
- **Framework**: Docusaurus v3 (static site generator for documentation)
- **Deployment**: GitHub Pages or Vercel
- **Chatbot UI**: Custom React component embedded in Docusaurus
- **Translation**: i18n support for Urdu (react-i18next or Docusaurus native)

### Backend (RAG Chatbot)
- **API Framework**: FastAPI (Python) with async endpoints
- **AI/LLM**: OpenAI GPT-4 Turbo via ChatKit SDK or OpenAI Agents SDK
- **Vector Database**: Qdrant Cloud Free Tier (for embeddings)
- **Relational Database**: Neon Serverless Postgres (user data, chat history)
- **Authentication**: better-auth.com (email/password + OAuth)
- **Deployment**: Vercel or Railway (free tier)

### Content Generation
- **AI Assistant**: Claude Code (Sonnet 4.5) with Spec-Kit Plus
- **Version Control**: Git with conventional commits
- **CI/CD**: GitHub Actions for automated deployment

### Development Environment
- **OS**: Cross-platform (Windows/macOS for book writing; Linux recommended for ROS 2 examples)
- **Package Manager**: npm/pnpm (frontend), pip + Poetry (backend)
- **Linting**: ESLint (JS/TS), Black + Ruff (Python)

## Development Workflow

### Content Creation Pipeline
1. **Spec Phase**: Define chapter learning outcomes and structure using `/sp.specify`
2. **Plan Phase**: Architect content flow and code examples using `/sp.plan`
3. **Tasks Phase**: Break down into writing tasks using `/sp.tasks`
4. **Implementation**: Write content with Claude Code, commit incrementally
5. **Review**: Technical accuracy review + readability check
6. **Integration**: Embed into Docusaurus, test chatbot Q&A on new content
7. **Deployment**: Merge to main → auto-deploy to GitHub Pages

### Quality Gates (Pre-Deployment Checklist)
- [ ] All code examples tested on Ubuntu 22.04 with specified dependencies
- [ ] External links validated (no 404s)
- [ ] Images have alt text
- [ ] Chatbot can answer at least 3 key questions per chapter
- [ ] Mobile layout tested (Chrome DevTools)
- [ ] Lighthouse score > 90 (performance, accessibility, SEO)
- [ ] Spell check passed (no technical jargon false positives)

### Git Workflow
- **Branch Naming**: `module-<number>/<feature>` (e.g., `module-1/ros2-nodes`)
- **Commits**: Conventional commits (`docs: add ROS 2 nodes chapter`, `feat: add chatbot text selection`)
- **Pull Requests**: Required for module completion; self-review acceptable for hackathon
- **Tagging**: Release tags for major milestones (`v1.0.0-module1-complete`)

## Governance

### Constitution Authority
This constitution supersedes all other development practices. When conflicts arise:
1. Constitution principles take precedence
2. Architectural Decision Records (ADRs) document exceptions with rationale
3. Amendments require version bump and sync impact report

### Amendment Process
- **Minor amendments** (clarifications, examples): Direct edit with PATCH version bump
- **Major amendments** (new principles, removed constraints): Create ADR first using `/sp.adr`, then update constitution with MINOR/MAJOR bump
- **Emergency fixes** (security, broken links): Immediate fix allowed, document in commit message

### Compliance Enforcement
- All PRs must reference this constitution for significant changes
- Complexity (e.g., adding new backend services) must justify necessity
- Runtime development guidance defers to this constitution

### Hackathon-Specific Overrides
- **Speed over perfection**: 80% complete > 100% polished for initial submission
- **Bonus points priority**: Subagents/Skills > Authentication > Personalization > Translation
- **Submission deadline**: Nov 30, 2025, 6:00 PM (non-negotiable hard stop)

**Version**: 1.0.0 | **Ratified**: 2025-12-06 | **Last Amended**: 2025-12-06
