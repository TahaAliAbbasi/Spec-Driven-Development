"""
Script to scrape the website and store embeddings in Qdrant with detailed output
"""
import requests
from bs4 import BeautifulSoup
import time
import os
from typing import List, Dict, Any
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.models import Distance, VectorParams
import cohere

# Load environment variables
load_dotenv()

# Initialize clients
cohere_client = cohere.Client(os.getenv("COHERE_API_KEY"))
qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY"),
)

def scrape_page(url: str) -> Dict[str, Any]:
    """
    Scrape a single page and extract content
    """
    try:
        headers = {
            'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/91.0.4472.124 Safari/537.36'
        }
        response = requests.get(url, headers=headers)
        response.raise_for_status()

        # Properly handle encoding
        response.encoding = response.apparent_encoding
        soup = BeautifulSoup(response.text, 'html.parser')

        # Remove script and style elements
        for script in soup(["script", "style"]):
            script.decompose()

        # Extract title
        title = soup.find('title')
        title_text = title.get_text().strip() if title else ""

        # Extract main content
        content = soup.get_text()

        # Clean up text
        lines = (line.strip() for line in content.splitlines())
        chunks = (phrase.strip() for line in lines for phrase in line.split("  "))
        content_text = ' '.join(chunk for chunk in chunks if chunk)

        print(f"Scraped URL: {url}")
        print(f"Title: {title_text}")
        print(f"Content preview: {content_text[:200]}...")
        print(f"Content length: {len(content_text)} characters")
        print("-" * 50)

        return {
            'url': url,
            'title': title_text,
            'content': content_text
        }
    except Exception as e:
        print(f"Error scraping {url}: {str(e)}")
        return None

def chunk_text(text: str, max_chunk_size: int = 1000) -> List[str]:
    """
    Split text into chunks of specified size
    """
    chunks = []
    words = text.split()

    current_chunk = []
    current_length = 0

    for word in words:
        if current_length + len(word) + 1 <= max_chunk_size:
            current_chunk.append(word)
            current_length += len(word) + 1
        else:
            if current_chunk:
                chunks.append(' '.join(current_chunk))
            current_chunk = [word]
            current_length = len(word) + 1

    if current_chunk:
        chunks.append(' '.join(current_chunk))

    return chunks

def create_embeddings(texts: List[str]) -> List[List[float]]:
    """
    Create embeddings for a list of texts using Cohere
    """
    try:
        print(f"Creating embeddings for {len(texts)} text chunks...")
        response = cohere_client.embed(
            texts=texts,
            model="embed-multilingual-v3.0",
            input_type="search_document"
        )
        print(f"Successfully created {len(response.embeddings)} embeddings")
        return response.embeddings
    except Exception as e:
        print(f"Error creating embeddings: {str(e)}")
        return []

def store_in_qdrant(data_list: List[Dict[str, Any]]):
    """
    Store data in Qdrant collection
    """
    collection_name = os.getenv("QDRANT_COLLECTION_NAME", "physical_ai_humanoid_docs_v3")

    # Check if collection exists, create if it doesn't
    try:
        collection_info = qdrant_client.get_collection(collection_name)
        print(f"Collection {collection_name} exists with {collection_info.points_count} points")
        # Clear existing points
        qdrant_client.delete_collection(collection_name)
        print(f"Deleted existing collection {collection_name}")
    except:
        print(f"Collection {collection_name} does not exist, will create it")

    # Create collection
    qdrant_client.create_collection(
        collection_name=collection_name,
        vectors_config=VectorParams(size=1024, distance=Distance.COSINE),  # Cohere v3 returns 1024-dim vectors
    )
    print(f"Created collection {collection_name}")

    all_texts = []
    all_metadata = []

    for i, data in enumerate(data_list):
        # Chunk the content
        content_chunks = chunk_text(data['content'])
        print(f"Page {i+1}: Chunked content into {len(content_chunks)} chunks")

        for j, chunk in enumerate(content_chunks):
            all_texts.append(chunk)
            all_metadata.append({
                'content': chunk,
                'source_url': data['url'],
                'title': data['title'],
                'chunk_id': f"{i}_{j}",
                'token_count': len(chunk.split())
            })

    # Create all embeddings at once
    if all_texts:
        embeddings = create_embeddings(all_texts)

        # Prepare points
        points = []
        for idx, (embedding, metadata) in enumerate(zip(embeddings, all_metadata)):
            points.append(
                models.PointStruct(
                    id=idx,  # Use numeric ID
                    vector=embedding,
                    payload=metadata
                )
            )

        # Upload points to Qdrant
        print(f"Uploading {len(points)} points to Qdrant...")
        qdrant_client.upload_points(
            collection_name=collection_name,
            points=points
        )
        print(f"Successfully uploaded {len(points)} points to Qdrant")

def get_qdrant_stats():
    """
    Get statistics about the Qdrant collection
    """
    collection_name = os.getenv("QDRANT_COLLECTION_NAME", "physical_ai_humanoid_docs_v3")
    try:
        collection_info = qdrant_client.get_collection(collection_name)
        print(f"Collection stats: {collection_info.points_count} points")
        return collection_info.points_count
    except Exception as e:
        print(f"Error getting collection stats: {str(e)}")
        return 0

def main():
    """
    Main function to run the scraping and embedding process
    """
    website_url = "https://physical-ai-and-humanoid-robotics-lemon.vercel.app/"

    print("Starting website scraping process...")

    # Get initial stats
    initial_count = get_qdrant_stats()

    # Crawl the website
    pages_data = []
    main_page = scrape_page(website_url)
    if main_page:
        pages_data.append(main_page)

    if not pages_data:
        print("No pages were scraped successfully")
        return

    print(f"Scraped {len(pages_data)} pages")

    # Store in Qdrant
    store_in_qdrant(pages_data)

    # Get final stats
    final_count = get_qdrant_stats()
    print(f"Added {final_count - initial_count} new points to Qdrant")

    print("Website scraping and embedding process completed!")

if __name__ == "__main__":
    main()