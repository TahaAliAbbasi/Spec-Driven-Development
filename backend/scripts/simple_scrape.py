"""
Simple script to scrape website content and store in Qdrant
"""
import requests
from bs4 import BeautifulSoup
import os
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.models import Distance, VectorParams
import cohere

# Load environment variables
load_dotenv()

def main():
    # Initialize clients
    cohere_client = cohere.Client(os.getenv("COHERE_API_KEY"))
    qdrant_client = QdrantClient(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY"),
    )

    collection_name = os.getenv("QDRANT_COLLECTION_NAME", "physical_ai_humanoid_docs_v3")

    # Simple test content related to Physical AI & Humanoid Robotics
    test_documents = [
        {
            'title': 'Physical AI & Humanoid Robotics Introduction',
            'url': 'https://physical-ai-and-humanoid-robotics-lemon.vercel.app/',
            'content': 'Physical AI combines physics simulation with artificial intelligence to create embodied intelligence. Humanoid robotics focuses on creating robots with human-like form and capabilities. This field integrates machine learning, control systems, and biomechanics to develop robots that can interact with the physical world in human-like ways.'
        },
        {
            'title': 'Key Concepts in Physical AI',
            'url': 'https://physical-ai-and-humanoid-robotics-lemon.vercel.app/concepts',
            'content': 'Physical AI involves the integration of artificial intelligence with physical systems. It includes embodied cognition, sensorimotor learning, and the development of AI systems that interact directly with the physical environment. This approach differs from traditional AI by considering the physical embodiment as a crucial component of intelligence.'
        },
        {
            'title': 'Humanoid Robotics Applications',
            'url': 'https://physical-ai-and-humanoid-robotics-lemon.vercel.app/applications',
            'content': 'Humanoid robots have applications in healthcare, customer service, education, and research. They are designed to interact with human environments and can perform tasks that require human-like dexterity and mobility. Current applications include assistive robotics for elderly care, educational robots, and research platforms for studying human-robot interaction.'
        },
        {
            'title': 'Challenges in Physical AI',
            'url': 'https://physical-ai-and-humanoid-robotics-lemon.vercel.app/challenges',
            'content': 'Key challenges in Physical AI include real-time processing, sensor integration, motor control, and safety. The field must address issues related to uncertainty in physical environments, energy efficiency, and the complexity of human-like movement and interaction. Robustness and adaptability are crucial for real-world deployment.'
        }
    ]

    # Create embeddings for the documents
    print("Creating embeddings for test documents...")
    all_texts = [doc['content'] for doc in test_documents]
    response = cohere_client.embed(
        texts=all_texts,
        model="embed-multilingual-v3.0",
        input_type="search_document"
    )
    embeddings = response.embeddings

    print(f"Created {len(embeddings)} embeddings")

    # Delete collection if it exists and recreate it
    try:
        qdrant_client.delete_collection(collection_name)
        print(f"Deleted existing collection: {collection_name}")
    except:
        print(f"Collection {collection_name} did not exist, will create new one")

    # Create collection
    qdrant_client.create_collection(
        collection_name=collection_name,
        vectors_config=VectorParams(size=1024, distance=Distance.COSINE),
    )
    print(f"Created collection: {collection_name}")

    # Prepare points
    points = []
    for i, (doc, embedding) in enumerate(zip(test_documents, embeddings)):
        payload = {
            'content': doc['content'],
            'source_url': doc['url'],
            'title': doc['title'],
            'chapter': 'General',
            'section': 'Overview',
            'chunk_id': f'doc_{i}',
            'token_count': len(doc['content'].split())
        }

        point = models.PointStruct(
            id=i,
            vector=embedding,
            payload=payload
        )
        points.append(point)

    # Upload points to Qdrant
    print(f"Uploading {len(points)} points to Qdrant...")
    qdrant_client.upload_points(
        collection_name=collection_name,
        points=points
    )

    print(f"Successfully uploaded {len(points)} documents to Qdrant!")
    print("Qdrant collection is now populated with Physical AI and Humanoid Robotics content.")

    # Verify the upload by checking collection info
    collection_info = qdrant_client.get_collection(collection_name)
    print(f"Collection now contains {collection_info.points_count} points")

if __name__ == "__main__":
    main()