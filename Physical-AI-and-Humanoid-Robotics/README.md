# Docusaurus RAG Chatbot

A Retrieval-Augmented Generation (RAG) chatbot for Docusaurus documentation sites. This system allows users to ask questions about documentation content and receive answers based on the actual documentation, with citations to source material.

## Features

- **Documentation Ingestion**: Scans and processes Markdown files from documentation directories
- **Intelligent Chunking**: Preserves document structure and headings while chunking content
- **Vector Storage**: Uses Qdrant Cloud for efficient similarity search
- **Metadata Storage**: Stores document metadata in Neon Postgres
- **Global Queries**: Ask questions about the entire documentation set
- **Selected Text Queries**: Ask questions about specific selected text
- **Source Citations**: Answers include clickable links to source documentation
- **Feedback System**: Users can provide feedback on answer quality
- **Embeddable Widget**: React widget for easy Docusaurus integration

## Architecture

The system consists of:

- **Backend**: FastAPI application handling document processing and question answering
- **Frontend**: React widget for Docusaurus integration
- **Vector Database**: Qdrant Cloud for embedding storage and similarity search
- **Metadata Database**: Neon Postgres for document and query metadata

## Prerequisites

- Python 3.11+
- Node.js 18+
- Docker and Docker Compose (for local development)
- Access to Qdrant Cloud Free Tier
- Access to Neon Postgres Free Tier

## Setup

### Backend Setup

1. Navigate to the backend directory:
   ```bash
   cd backend
   ```

2. Create a virtual environment:
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

3. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

4. Create a `.env` file with your configuration:
   ```env
   # Qdrant Configuration
   QDRANT_URL=https://your-qdrant-url
   QDRANT_API_KEY=your-qdrant-api-key
   QDRANT_COLLECTION_NAME=physical_ai_humanoid_docs_v1

   # Database Configuration
   DATABASE_URL=postgresql://user:password@host:port/dbname

   # Embedding Configuration
   EMBEDDING_BACKEND=hf_local
   EMBEDDING_MODEL_NAME=sentence-transformers/all-MiniLM-L6-v2

   # LLM Configuration (Optional)
   LLM_BACKEND=gemini
   GEMINI_API_KEY=your-gemini-api-key
   GEMINI_MODEL=gemini-2.5-flash

   # API Key for administrative endpoints
   ADMIN_API_KEY=your-admin-api-key
   ```

5. Start the backend server:
   ```bash
   uvicorn main:app --reload --port 8000
   ```

### Frontend Setup

1. Navigate to the frontend directory:
   ```bash
   cd frontend/rag-chat-widget
   ```

2. Install dependencies:
   ```bash
   npm install
   ```

3. Build the widget:
   ```bash
   npm run build
   ```

### Documentation Ingestion

To process your documentation:

1. Place your Markdown files in the `docs/` directory
2. Run the embedding script:
   ```bash
   cd scripts
   python embed_docs.py --directory "../docs" --chunk-size 800 --overlap 100
   ```

Or use the API endpoint:
```bash
curl -X POST "http://localhost:8000/api/embed" \
  -H "Content-Type: application/json" \
  -H "X-API-Key: your-admin-api-key" \
  -d '{
    "directory": "Physical-AI-and-Humanoid-Robotics/docs",
    "chunk_size": 800,
    "overlap": 100
  }'
```

## API Endpoints

- `POST /api/embed` - Process and embed documentation
- `POST /api/ask-global` - Ask a question about the documentation
- `POST /api/ask-selected` - Ask about selected text only
- `POST /api/feedback` - Submit feedback on an answer
- `GET /api/health` - Check system health

## Docusaurus Integration

To integrate the chat widget into your Docusaurus site:

1. Copy the built widget file to your Docusaurus static directory
2. Include the widget in your site's layout

## Local Development

Use Docker Compose for local development:

```bash
cd docker
docker-compose up
```

This will start all services including the backend, frontend, Qdrant, and PostgreSQL.

## Configuration

Key configuration options:

- `chunk_size`: Size of text chunks in tokens (default: 800)
- `chunk_overlap`: Overlap between chunks in tokens (default: 100)
- `max_document_pages`: Maximum number of pages to process (default: 1000)
- `document_processing_timeout`: Timeout for processing a single document (default: 120 seconds)
- `default_top_k`: Default number of results to retrieve (default: 6)
- `max_top_k`: Maximum number of results to retrieve (default: 20)

## Security

- Administrative endpoints (`/embed`, `/feedback`) require API key authentication
- API key should be provided in the `X-API-Key` header
- Input validation is performed on all endpoints

## License

MIT
