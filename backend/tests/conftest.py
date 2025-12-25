"""
Pytest configuration file
"""
import pytest
import sys
import os

# Add the backend directory to the path so tests can import modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

@pytest.fixture
def sample_user_query():
    """Fixture for a sample UserQuery"""
    from backend.models.query import UserQuery
    return UserQuery(query_text="Test query for fixture")


@pytest.fixture
def sample_retrieved_context():
    """Fixture for sample RetrievedContext"""
    from backend.models.context import RetrievedContext
    return RetrievedContext(
        content="Test content for fixture",
        source_url="https://example.com/test",
        chapter="Test Chapter",
        section="Test Section",
        chunk_id="test_chunk_123",
        relevance_score=0.85,
        token_count=150
    )


@pytest.fixture
def mock_agent():
    """Fixture for a mocked ChatAgent"""
    from unittest.mock import Mock
    mock_agent = Mock()
    mock_agent.validate_query.return_value = True
    return mock_agent