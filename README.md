# Physical AI & Humanoid Robotics Textbook

An AI-native interactive textbook for teaching Physical AI and Humanoid Robotics, featuring an embedded RAG chatbot, content personalization, and Urdu translation.

**Hackathon Submission**: Panaversity AI Hackathon I

---

## 🎯 Project Overview

This project is an interactive textbook built with **Docusaurus 3** and powered by an AI chatbot using **OpenAI GPT-4**, **Qdrant**, and **Neon Postgres**. The book covers:

- **Module 1**: ROS 2 (Robot Operating System)
- **Module 2**: Gazebo & Unity (Digital Twin Simulation)
- **Module 3**: NVIDIA Isaac (AI-Robot Brain)
- **Module 4**: Vision-Language-Action (VLA) Systems

### Features

- ✅ **Interactive Textbook** - Responsive Docusaurus site with 12-16 chapters
- ✅ **RAG Chatbot** - Ask questions about book content with citations
- ✅ **Text-Selection Q&A** - Select text and ask contextual questions
- ✅ **Authentication** - Sign up/sign in with background questionnaire
- ✅ **Content Personalization** - Adapt chapter difficulty based on user experience level
- ✅ **Urdu Translation** - Translate chapters to Urdu with RTL layout
- ✅ **Claude Code Subagents** - Custom AI agents for content generation

---

## 🚀 Quick Start

### Prerequisites

- **Node.js** 18+ (for Docusaurus frontend)
- **Python** 3.11+ (for FastAPI backend)
- **npm** or **pnpm** (package manager)
- **Git** (version control)

### Installation

1. **Clone the repository**:
   ```bash
   git clone https://github.com/your-username/shery_ai_book.git
   cd shery_ai_book
   ```

2. **Install frontend dependencies**:
   ```bash
   npm install
   ```

3. **Install backend dependencies**:
   ```bash
   cd backend
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   pip install -r requirements.txt
   cd ..
   ```

4. **Configure environment variables**:
   ```bash
   cp .env.example .env
   # Edit .env and add your API keys:
   # - OPENAI_API_KEY (from https://platform.openai.com/api-keys)
   # - QDRANT_URL and QDRANT_API_KEY (from https://cloud.qdrant.io/)
   # - DATABASE_URL (from https://neon.tech/)
   # - JWT_SECRET_KEY (generate with: openssl rand -hex 32)
   ```

### Development

1. **Start the Docusaurus development server**:
   ```bash
   npm start
   ```
   Opens at `http://localhost:3000`

2. **Start the FastAPI backend** (in a separate terminal):
   ```bash
   cd backend
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   uvicorn main:app --reload --port 8000
   ```
   API available at `http://localhost:8000`

3. **Generate embeddings** (one-time setup):
   ```bash
   cd backend
   python ../scripts/generate_embeddings.py
   ```

### Build for Production

```bash
npm run build
```

The static site will be generated in `build/`.

---

## 📁 Project Structure

```
shery_ai_book/
├── docs/                      # Textbook content (Markdown)
│   ├── intro.md
│   ├── hardware-requirements.md
│   ├── module-1/              # ROS 2 chapters
│   ├── module-2/              # Gazebo & Unity chapters
│   ├── module-3/              # NVIDIA Isaac chapters
│   └── module-4/              # VLA chapters
├── src/                       # Custom React components
│   ├── components/
│   │   ├── ChatBot/           # RAG chatbot UI
│   │   ├── AuthPanel/         # Authentication UI
│   │   ├── PersonalizeButton/ # Content personalization
│   │   └── TranslateButton/   # Urdu translation
│   └── css/                   # Custom styles
├── backend/                   # FastAPI backend
│   ├── main.py                # FastAPI app entry point
│   ├── config.py              # Environment configuration
│   ├── models/                # Pydantic schemas
│   ├── services/              # Business logic (RAG, auth, etc.)
│   ├── api/                   # API route handlers
│   ├── database/              # Database connections
│   └── utils/                 # Utilities (security, logging)
├── specs/                     # Spec-Kit Plus documentation
│   └── physical-ai-textbook/
│       ├── spec.md            # Requirements specification
│       ├── plan.md            # Architectural plan
│       └── tasks.md           # Implementation tasks
├── .specify/                  # Spec-Kit Plus configuration
├── docusaurus.config.ts       # Docusaurus configuration
├── package.json               # Frontend dependencies
└── README.md                  # This file
```

---

## 🧪 Testing

### Frontend Tests

```bash
npm test
```

### Backend Tests

```bash
cd backend
pytest
```

### E2E Tests

```bash
npm run test:e2e
```

---

## 🌐 Deployment

### Frontend (GitHub Pages)

Automatically deployed via GitHub Actions on push to `main`:

```yaml
# .github/workflows/deploy-frontend.yml
```

Manual deployment:

```bash
GIT_USER=<your-github-username> npm run deploy
```

### Backend (Vercel)

1. Install Vercel CLI: `npm i -g vercel`
2. Deploy: `vercel --prod`

Or connect your GitHub repo to Vercel for automatic deployments.

---

## 🎓 Learning Outcomes

After completing this textbook, students will be able to:

1. Understand Physical AI principles and embodied intelligence
2. Master ROS 2 for robotic control
3. Simulate robots with Gazebo and Unity
4. Develop with NVIDIA Isaac AI robot platform
5. Design humanoid robots for natural interactions
6. Integrate GPT models for conversational robotics

---

## 🏆 Hackathon Submission Checklist

- [ ] **Public GitHub repository** with README
- [ ] **Published book** (GitHub Pages or Vercel) - [Live URL]
- [ ] **Demo video** (under 90 seconds) - [YouTube Link]
- [ ] **Submission form** completed before Nov 30, 6:00 PM

### Scoring Breakdown

| Feature | Points | Status |
|---------|--------|--------|
| Docusaurus book deployed | 50 | ⬜ |
| RAG chatbot with text-selection | 50 | ⬜ |
| Authentication + questionnaire | +50 | ⬜ |
| Content personalization | +50 | ⬜ |
| Urdu translation | +50 | ⬜ |
| Claude Code Subagents | +50 | ⬜ |
| **Total** | **300** | **0** |

---

## 🛠️ Technology Stack

### Frontend
- **Docusaurus 3** - Static site generator
- **React 18** - UI components
- **TypeScript** - Type-safe JavaScript

### Backend
- **FastAPI** - Modern Python web framework
- **OpenAI GPT-4 Turbo** - RAG chatbot, personalization, translation
- **Qdrant Cloud** - Vector database for embeddings
- **Neon Serverless Postgres** - User data, chat history
- **better-auth (or JWT)** - Authentication

### Development
- **Claude Code (Sonnet 4.5)** - AI-assisted development
- **Spec-Kit Plus** - Specification-driven development
- **GitHub Actions** - CI/CD automation

---

## 📚 Documentation

- **Specification**: `specs/physical-ai-textbook/spec.md`
- **Architectural Plan**: `specs/physical-ai-textbook/plan.md`
- **Implementation Tasks**: `specs/physical-ai-textbook/tasks.md`
- **Constitution**: `.specify/memory/constitution.md`

---

## 🤝 Contributing

This is a hackathon submission project. Contributions are not currently accepted, but feedback is welcome!

---

## 📄 License

This project is created for educational purposes as part of the Panaversity AI Hackathon.

---

## 🙏 Acknowledgments

- **Panaversity** for organizing the hackathon
- **PIAIC & GIAIC** for AI education initiatives
- **Claude Code** (Anthropic) for AI-assisted development
- **Spec-Kit Plus** for specification-driven development framework

---

## 📧 Contact

- **WhatsApp**: [Your number for live presentation invitation]
- **GitHub**: [Your GitHub profile]
- **Email**: [Your email]

---

**Built with ❤️ using Claude Code and Spec-Kit Plus**
