# Quickstart Guide: Educational Textbook for Physical AI & Humanoid Robotics

## Prerequisites

- Node.js (v18 or higher)
- Python (v3.10 or higher)
- Git
- Docker (optional, for containerized development)

## Getting Started

### 1. Clone the Repository

```bash
git clone https://github.com/your-org/physical-ai-humanoid-robotics.git
cd physical-ai-humanoid-robotics
```

### 2. Set up the Docusaurus Frontend

```bash
# Navigate to the textbook directory
cd my-book

# Install dependencies
npm install

# Start the development server
npm start
```

The Docusaurus site will be available at `http://localhost:3000`.

### 3. Set up the Backend Service

```bash
# From the project root
cd backend

# Create a virtual environment
python -m venv venv

# Activate the virtual environment
# On Linux/Mac:
source venv/bin/activate
# On Windows:
venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Set up environment variables
cp .env.example .env
# Edit .env with your configuration values

# Start the backend server
uvicorn src.api.main:app --reload --port 8000
```

The backend API will be available at `http://localhost:8000`.

### 4. Environment Configuration

Create a `.env` file in the backend directory with the following variables:

```env
# Database configuration
DATABASE_URL=postgresql://user:password@localhost/dbname

# Vector database configuration
QDRANT_URL=http://localhost:6333

# OpenAI configuration
OPENAI_API_KEY=your_openai_api_key

# Authentication
JWT_SECRET_KEY=your_jwt_secret_key
ACCESS_TOKEN_EXPIRE_MINUTES=30
```

### 5. Running with Docker (Optional)

```bash
# From the project root
docker-compose up --build
```

### 6. Adding Content

Content is organized in the `my-book/docs/` directory:

```
my-book/docs/
├── module-1/
│   ├── chapter-1.md
│   ├── chapter-2.md
│   ├── chapter-3.md
│   └── chapter-4.md
├── module-2/
│   ├── chapter-1.md
│   ├── chapter-2.md
│   ├── chapter-3.md
│   └── chapter-4.md
├── module-3/
│   ├── chapter-1.md
│   ├── chapter-2.md
│   ├── chapter-3.md
│   └── chapter-4.md
└── module-4/
    ├── chapter-1.md
    ├── chapter-2.md
    ├── chapter-3.md
    └── chapter-4.md
```

Each chapter document follows the standard Docusaurus Markdown format with frontmatter:

```markdown
---
title: Chapter Title
sidebar_position: 1
description: Brief description of the chapter
---

# Chapter Title

Your chapter content goes here...
```

### 7. Running Tests

#### Frontend Tests
```bash
cd my-book
npm test
```

#### Backend Tests
```bash
cd backend
python -m pytest
```

### 8. Building for Production

#### Frontend
```bash
cd my-book
npm run build
```

#### Backend
```bash
cd backend
# Docker build
docker build -t textbook-backend .
```

### 9. Deployment

The frontend can be deployed to GitHub Pages or Vercel, while the backend can be deployed to any cloud provider that supports containerized applications.

For GitHub Pages deployment:
```bash
cd my-book
GIT_USER=<your-github-username> \
USE_SSH=true \
npm run deploy
```

For Vercel deployment:
1. Link your project to Vercel: `vercel`
2. Follow the prompts to configure your project
3. Deploy: `vercel --prod`