"""
Unit tests for the Retrieval & Context Assembly Service
"""
import os
import sys
from unittest.mock import Mock, patch, MagicMock

# Add the backend directory to the path so we can import the main module
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from retrieval.main import (
    RetrievalRequest,
    RetrievedChunk,
    ContextBundle,
    count_tokens,
    vectorize_text,
    filter_chunks,
    assemble_context,
    RetrievalController
)


def test_retrieval_request_validation():
    """Test validation of RetrievalRequest model"""
    # Test valid request
    req = RetrievalRequest(query="test query", top_k=5)
    assert req.query == "test query"
    assert req.top_k == 5

    # Test selected_text instead of query
    req2 = RetrievalRequest(selected_text="selected text", top_k=3)
    assert req2.selected_text == "selected text"
    assert req2.top_k == 3

    # Test invalid top_k
    try:
        RetrievalRequest(query="test", top_k=25)  # Should fail
        assert False, "Should have raised validation error"
    except ValueError:
        pass  # Expected

    # Test invalid query length
    try:
        RetrievalRequest(query="x" * 1001)  # Should fail (too long)
        assert False, "Should have raised validation error"
    except ValueError:
        pass  # Expected


def test_retrieved_chunk_validation():
    """Test validation of RetrievedChunk model"""
    chunk = RetrievedChunk(
        content="test content",
        source_url="https://example.com",
        chapter="test",
        section="test",
        chunk_id="test-id",
        relevance_score=0.8,
        token_count=10
    )
    assert chunk.content == "test content"
    assert chunk.relevance_score == 0.8
    assert chunk.token_count == 10

    # Test invalid relevance score
    try:
        RetrievedChunk(
            content="test",
            source_url="https://example.com",
            chapter="test",
            section="test",
            chunk_id="test-id",
            relevance_score=1.5,  # Invalid
            token_count=10
        )
        assert False, "Should have raised validation error"
    except ValueError:
        pass  # Expected


def test_count_tokens():
    """Test token counting function"""
    text = "This is a test sentence with several words."
    tokens = count_tokens(text)
    # Using Phase 1 heuristic: words * 1.33
    expected = max(1, int(len(text.split()) * 1.33))
    assert tokens == expected

    # Test empty string
    assert count_tokens("") == 0

    # Test single word
    assert count_tokens("hello") == 1


def test_filter_chunks():
    """Test metadata filtering functionality"""
    chunks = [
        {
            'content': 'chunk 1',
            'source_url': 'https://example.com',
            'chapter': 'intro',
            'section': 'overview',
            'chunk_id': 'id1',
            'relevance_score': 0.9,
            'token_count': 10,
            'language': 'en',
            'version': '1.0'
        },
        {
            'content': 'chunk 2',
            'source_url': 'https://example.com',
            'chapter': 'advanced',
            'section': 'details',
            'chunk_id': 'id2',
            'relevance_score': 0.7,
            'token_count': 15,
            'language': 'es',
            'version': '1.0'
        }
    ]

    # Test chapter filtering
    filters = {'chapter_filter': 'intro'}
    filtered = filter_chunks(chunks, filters)
    assert len(filtered) == 1
    assert filtered[0]['chapter'] == 'intro'

    # Test language filtering
    filters = {'language_constraint': 'es'}
    filtered = filter_chunks(chunks, filters)
    assert len(filtered) == 1
    assert filtered[0]['language'] == 'es'

    # Test multiple filters
    filters = {'chapter_filter': 'intro', 'language_constraint': 'en'}
    filtered = filter_chunks(chunks, filters)
    assert len(filtered) == 1
    assert filtered[0]['chapter'] == 'intro'
    assert filtered[0]['language'] == 'en'


def test_assemble_context():
    """Test context assembly functionality"""
    chunks = [
        {
            'content': 'test content 1',
            'source_url': 'https://example.com',
            'chapter': 'intro',
            'section': 'overview',
            'chunk_id': 'id1',
            'relevance_score': 0.9,
            'token_count': 10
        },
        {
            'content': 'test content 2',
            'source_url': 'https://example.com',
            'chapter': 'advanced',
            'section': 'details',
            'chunk_id': 'id2',
            'relevance_score': 0.7,
            'token_count': 15
        }
    ]

    # Test assembly within token limits
    bundle = assemble_context(chunks, max_tokens=100)
    assert len(bundle.retrieved_chunks) == 2
    assert bundle.total_tokens == 25  # 10 + 15
    assert bundle.status.value == 'success'

    # Test assembly with token limit exceeded
    bundle = assemble_context(chunks, max_tokens=12)  # Only enough for first chunk
    assert len(bundle.retrieved_chunks) == 1
    assert bundle.total_tokens == 10
    assert bundle.status.value in ['partial', 'success']  # Could be partial if more available


def test_vectorize_text():
    """Test vectorization function (mocked API call)"""
    # Mock the Cohere client
    mock_client = Mock()
    mock_response = Mock()
    mock_response.embeddings = [[0.1, 0.2, 0.3]]  # Mock embedding
    mock_client.embed.return_value = mock_response

    result = vectorize_text("test text", mock_client)
    assert result == [0.1, 0.2, 0.3]
    mock_client.embed.assert_called_once()


def test_retrieval_controller():
    """Test retrieval controller (basic instantiation)"""
    mock_cohere = Mock()
    mock_qdrant = Mock()

    controller = RetrievalController(mock_cohere, mock_qdrant)
    assert controller.cohere_client == mock_cohere
    assert controller.qdrant_client == mock_qdrant


if __name__ == "__main__":
    # Run the tests
    test_retrieval_request_validation()
    test_retrieved_chunk_validation()
    test_count_tokens()
    test_filter_chunks()
    test_assemble_context()
    test_vectorize_text()
    test_retrieval_controller()

    print("All tests passed! :)")