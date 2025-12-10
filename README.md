# Physical AI & Humanoid Robotics Book

A comprehensive Docusaurus-based book with integrated RAG chatbot on Physical AI & Humanoid Robotics, created using Spec-Kit Plus and Claude Code.

## Overview

This project implements a complete educational resource on Physical AI & Humanoid Robotics featuring:

- **4 Core Modules + Capstone Project**:
  - Module 1: ROS 2 (Robot Operating System 2)
  - Module 2: Simulation Environments (Gazebo & Unity)
  - Module 3: NVIDIA Isaac Platform
  - Module 4: Vision-Language-Action (VLA) Systems
  - Capstone: Autonomous Humanoid Robot

- **Integrated RAG Chatbot**:
  - FastAPI backend with Qdrant vector storage
  - OpenAI integration for natural language understanding
  - Context-aware responses based on book content
  - Source attribution for all answers

## Architecture

### Frontend (Docusaurus)
- Educational content organized in 4 modules + capstone
- Interactive RAG chat interface
- GitHub Pages deployment ready

### Backend (FastAPI)
- `/api/v1/chat` - Main RAG endpoint
- `/api/v1/documents` - Document search and retrieval
- Qdrant integration for vector storage
- OpenAI integration for response generation

## Prerequisites

- Node.js (v20.0 or higher)
- Python (v3.8 or higher)
- OpenAI API key
- Qdrant Cloud account (or local instance)

## Setup

### Frontend Setup

```bash
# Install dependencies
npm install

# Start development server
npm run start
```

The site will be available at `http://localhost:3000`

### Backend Setup

```bash
# Navigate to backend directory
cd backend

# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Create .env file with your configuration
cp .env.example .env
# Edit .env with your API keys and settings

# Start the backend server
python main.py
```

The backend will be available at `http://localhost:8000`

## Configuration

### Environment Variables

Create a `.env` file in the `backend/` directory with the following:

```env
OPENAI_API_KEY=your_openai_api_key_here
OPENAI_BASE_URL=https://api.openai.com/v1
OPENAI_MODEL=gpt-3.5-turbo
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
NEON_POSTGRES_URL=your_neon_postgres_url
```

### Docusaurus Configuration

The site is configured for GitHub Pages deployment. Update the following in `docusaurus.config.ts`:

- `url`: Your GitHub Pages URL
- `baseUrl`: Base path for your site
- `organizationName`: Your GitHub username/organization
- `projectName`: Your repository name

## Deployment

### GitHub Pages

1. Update the deployment configuration in `docusaurus.config.ts`
2. Run the deployment command:

```bash
GIT_USER=<Your GitHub username> npm run deploy
```

### Backend Deployment

The FastAPI backend can be deployed to any cloud platform that supports Python applications (AWS, GCP, Azure, Heroku, etc.).

## Content Structure

The book content is organized as follows:

```
docs/
├── module-1-ros2/           # ROS 2 fundamentals
├── module-2-simulation/     # Gazebo & Unity simulation
├── module-3-isaac/          # NVIDIA Isaac platform
├── module-4-vla-systems/    # Vision-Language-Action systems
└── capstone-autonomous-humanoid/  # Capstone project
```

## Development

### Adding New Content

1. Create new markdown files in the appropriate module directory
2. Update `sidebars.ts` to include the new content in the navigation
3. Ensure content follows the technical accuracy and student-friendly principles

### Extending the RAG System

1. Add new documents to the Qdrant vector database
2. Update the backend services as needed
3. Test the integration with the frontend chat interface

## Technologies Used

- **Frontend**: Docusaurus, React, TypeScript
- **Backend**: FastAPI, Python
- **Vector Storage**: Qdrant Cloud
- **AI/ML**: OpenAI API, Sentence Transformers
- **Database**: Neon Postgres (for metadata)
- **Deployment**: GitHub Pages

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Support

For support, please open an issue in the GitHub repository."# Hackathon-Q-4" 
