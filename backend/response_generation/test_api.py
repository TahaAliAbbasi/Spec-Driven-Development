"""Test script for API endpoints"""


import requests
import json


def test_api_endpoints():
    base_url = "http://localhost:8000"

    # Test health endpoint
    print("Testing health endpoint...")
    health_response = requests.get(f"{base_url}/api/health/answer")
    print(f"Health endpoint status: {health_response.status_code}")
    print(f"Health endpoint response: {health_response.json()}")
    print()

    # Test root endpoint
    print("Testing root endpoint...")
    root_response = requests.get(f"{base_url}/")
    print(f"Root endpoint status: {root_response.status_code}")
    print(f"Root endpoint response: {root_response.json()}")
    print()

    # Test answer endpoint with a sample request
    # Note: This will likely fail without a valid OPENROUTER_API_KEY, but should validate input
    print("Testing answer endpoint with sample request...")
    sample_payload = {
        "query": "What is RAG in AI?",
        "context_bundle": {
            "chunks": [
                {
                    "chunk_id": "chunk1",
                    "content": "RAG stands for Retrieval Augmented Generation. It is a technique in AI that combines information retrieval with generative models to produce more accurate and contextually relevant responses.",
                    "metadata": {"source": "ai_document.pdf", "source_url": "http://example.com/doc1"}
                }
            ],
            "status": "success"
        },
        "mode": "global"
    }

    headers = {
        'Content-Type': 'application/json'
    }

    try:
        answer_response = requests.post(f"{base_url}/api/answer",
                                      data=json.dumps(sample_payload),
                                      headers=headers)
        print(f"Answer endpoint status: {answer_response.status_code}")
        print(f"Answer endpoint response: {answer_response.json()}")
    except requests.exceptions.ConnectionError:
        print("Could not connect to the API - service might not be running")
    except Exception as e:
        print(f"Error testing answer endpoint: {str(e)}")

    print()

    # Test answer endpoint with selected_text_only mode
    print("Testing answer endpoint with selected_text_only mode...")
    sample_payload_selected = {
        "query": "What is RAG?",
        "context_bundle": {
            "chunks": [
                {
                    "chunk_id": "chunk1",
                    "content": "RAG stands for Retrieval Augmented Generation.",
                    "metadata": {"source": "ai_document.pdf", "source_url": "http://example.com/doc1"}
                }
            ],
            "status": "success"
        },
        "mode": "selected_text_only"
    }

    try:
        answer_response_selected = requests.post(f"{base_url}/api/answer",
                                               data=json.dumps(sample_payload_selected),
                                               headers=headers)
        print(f"Answer endpoint (selected_text_only) status: {answer_response_selected.status_code}")
        print(f"Answer endpoint (selected_text_only) response: {answer_response_selected.json()}")
    except requests.exceptions.ConnectionError:
        print("Could not connect to the API - service might not be running")
    except Exception as e:
        print(f"Error testing answer endpoint (selected_text_only): {str(e)}")


if __name__ == "__main__":
    test_api_endpoints()