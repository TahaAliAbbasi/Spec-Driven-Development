"""
Script to scrape the website and store embeddings in Qdrant
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

        soup = BeautifulSoup(response.content, 'html.parser')

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
        response = cohere_client.embed(
            texts=texts,
            model="embed-multilingual-v3.0",
            input_type="search_document"
        )
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
        qdrant_client.get_collection(collection_name)
        print(f"Collection {collection_name} exists")
    except:
        print(f"Creating collection {collection_name}")
        qdrant_client.create_collection(
            collection_name=collection_name,
            vectors_config=VectorParams(size=1024, distance=Distance.COSINE),  # Cohere v3 returns 1024-dim vectors
        )

    points = []
    for i, data in enumerate(data_list):
        # Chunk the content
        content_chunks = chunk_text(data['content'])

        for j, chunk in enumerate(content_chunks):
            # Create embedding for the chunk
            embedding = create_embeddings([chunk])
            if not embedding:
                continue

            vector = embedding[0]

            # Create a unique ID for this chunk
            point_id = f"{i}_{j}"

            # Prepare payload
            payload = {
                'content': chunk,
                'source_url': data['url'],
                'title': data['title'],
                'chunk_id': point_id,
                'token_count': len(chunk.split())
            }

            # Add to points
            points.append(
                models.PointStruct(
                    id=point_id,
                    vector=vector,
                    payload=payload
                )
            )

    # Upload points to Qdrant
    if points:
        print(f"Uploading {len(points)} points to Qdrant...")
        qdrant_client.upload_points(
            collection_name=collection_name,
            points=points
        )
        print(f"Successfully uploaded {len(points)} points to Qdrant")

def crawl_website(base_url: str) -> List[Dict[str, Any]]:
    """
    Crawl the website and extract all pages
    """
    print(f"Starting to crawl: {base_url}")

    # For now, just scrape the main page since we don't know the site structure
    pages_data = []

    # Scrape the main page
    main_page = scrape_page(base_url)
    if main_page:
        pages_data.append(main_page)

    # If the site has more pages, you would add them here
    # For example, if there are specific URLs you want to scrape:
    # additional_urls = [
    #     f"{base_url}/page1",
    #     f"{base_url}/page2",
    #     # Add more URLs as needed
    # ]
    #
    # for url in additional_urls:
    #     page_data = scrape_page(url)
    #     if page_data:
    #         pages_data.append(page_data)

    return pages_data

def main():
    """
    Main function to run the scraping and embedding process
    """
    website_url = "https://physical-ai-and-humanoid-robotics-lemon.vercel.app/"

    print("Starting website scraping process...")

    # Crawl the website
    pages_data = crawl_website(website_url)

    if not pages_data:
        print("No pages were scraped successfully")
        return

    print(f"Scraped {len(pages_data)} pages")

    # Store in Qdrant
    store_in_qdrant(pages_data)

    print("Website scraping and embedding process completed!")

if __name__ == "__main__":
    main()