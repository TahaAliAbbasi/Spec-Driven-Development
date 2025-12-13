# Quickstart Guide: Embedding Pipeline

**Feature**: 2-knowledge-ingestion
**Date**: 2025-12-13

## Prerequisites

- Python 3.9 or higher
- UV package manager
- Cohere API key
- Qdrant Cloud account and API key
- Access to target website: https://physical-ai-and-humanoid-robotics-lemon.vercel.app/

## Setup

1. Create the backend directory:
```bash
mkdir backend
cd backend
```

2. Initialize the project with UV:
```bash
uv init
uv add cohere qdrant-client requests beautifulsoup4 python-dotenv
```

3. Create environment file:
```bash
touch .env
```

4. Add required environment variables to `.env`:
```
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_URL=your_qdrant_cluster_url
```

## Usage

1. Create the main.py file with the embedding pipeline implementation

2. Run the pipeline:
```bash
python main.py
```

## Expected Output

- All pages from the target website will be discovered and processed
- Content will be extracted, chunked, and embedded
- Vectors will be stored in the 'physical_ai_humanoid_docs_v1' collection in Qdrant
- Processed chunk count and success metrics will be logged

## Configuration

The pipeline uses these default parameters:
- Cohere model: embed-multilingual-v3.0
- Chunk size: 512 tokens with 10% overlap
- Collection name: physical_ai_humanoid_docs_v1
- Vector dimensions: 1024