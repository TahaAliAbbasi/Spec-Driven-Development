"""Basic tests for the response generation service"""


import pytest
from fastapi.testclient import TestClient
from main import app
from models.agent_models import AgentInput, AgentOutput, StatusEnum


def test_health_endpoint():
    """Test the health check endpoint"""
    client = TestClient(app)
    response = client.get("/api/health/answer")
    assert response.status_code == 200
    assert response.json() == {"status": "healthy", "service": "answer-generation"}


def test_agent_input_validation():
    """Test AgentInput model validation"""
    # Valid input
    valid_input = {
        "query": "What is RAG?",
        "context_bundle": {
            "chunks": [
                {
                    "chunk_id": "chunk1",
                    "content": "RAG stands for Retrieval Augmented Generation.",
                    "metadata": {"source": "doc1.pdf"}
                }
            ],
            "status": "success"
        },
        "mode": "global"
    }

    agent_input = AgentInput(**valid_input)
    assert agent_input.query == "What is RAG?"
    assert agent_input.mode == "global"
    assert len(agent_input.context_bundle["chunks"]) == 1


def test_agent_input_validation_fails():
    """Test AgentInput model validation with invalid data"""
    # Invalid mode
    invalid_input = {
        "query": "What is RAG?",
        "context_bundle": {
            "chunks": [
                {
                    "chunk_id": "chunk1",
                    "content": "RAG stands for Retrieval Augmented Generation."
                }
            ]
        },
        "mode": "invalid_mode"
    }

    try:
        AgentInput(**invalid_input)
        assert False, "Should have raised validation error"
    except Exception:
        pass  # Expected


def test_agent_output_validation():
    """Test AgentOutput model validation"""
    # Valid output with answered status
    valid_output = AgentOutput(
        answer="RAG is Retrieval Augmented Generation",
        citations=[],
        used_chunks=[],
        status=StatusEnum.ANSWERED
    )
    assert valid_output.status == StatusEnum.ANSWERED
    assert valid_output.answer == "RAG is Retrieval Augmented Generation"


if __name__ == "__main__":
    test_health_endpoint()
    test_agent_input_validation()
    test_agent_input_validation_fails()
    test_agent_output_validation()
    print("All basic tests passed!")