"""
Unit tests for data models
"""
import pytest
from datetime import datetime
from backend.models.query import UserQuery
from backend.models.response import GeneratedResponse
from backend.models.citation import Citation
from backend.models.context import RetrievedContext
from backend.models.session import ConversationSession
from backend.models.message import Message, MessageRole


class TestUserQuery:
    """Tests for UserQuery model"""

    def test_user_query_creation(self):
        """Test creating a valid UserQuery"""
        query = UserQuery(query_text="Test query")
        assert query.query_text == "Test query"
        assert query.timestamp is not None
        assert query.session_id is None
        assert query.user_id is None

    def test_user_query_with_session(self):
        """Test creating a UserQuery with session info"""
        query = UserQuery(
            query_text="Test query",
            session_id="session123",
            user_id="user123"
        )
        assert query.query_text == "Test query"
        assert query.session_id == "session123"
        assert query.user_id == "user123"

    def test_user_query_validation(self):
        """Test UserQuery validation"""
        # Valid query
        UserQuery(query_text="Valid query")

        # Empty query should fail
        with pytest.raises(ValueError):
            UserQuery(query_text="")

        # Query too long should fail
        with pytest.raises(ValueError):
            UserQuery(query_text="x" * 1001)  # More than 1000 characters


class TestGeneratedResponse:
    """Tests for GeneratedResponse model"""

    def test_generated_response_creation(self):
        """Test creating a valid GeneratedResponse"""
        citation = Citation(
            title="Test Title",
            url="https://example.com",
            author="Test Author",
            relevance_score=0.9
        )

        response = GeneratedResponse(
            response_text="Test response",
            citations=[citation],
            confidence_score=0.85
        )

        assert response.response_text == "Test response"
        assert len(response.citations) == 1
        assert response.confidence_score == 0.85
        assert response.timestamp is not None


class TestCitation:
    """Tests for Citation model"""

    def test_citation_creation(self):
        """Test creating a valid Citation"""
        citation = Citation(
            title="Test Title",
            url="https://example.com",
            author="Test Author",
            relevance_score=0.9
        )

        assert citation.title == "Test Title"
        assert citation.url == "https://example.com"
        assert citation.author == "Test Author"
        assert citation.relevance_score == 0.9


class TestRetrievedContext:
    """Tests for RetrievedContext model"""

    def test_retrieved_context_creation(self):
        """Test creating a valid RetrievedContext"""
        context = RetrievedContext(
            content="Test content",
            source_url="https://example.com",
            chapter="Chapter 1",
            section="Section 1.1",
            chunk_id="chunk123",
            relevance_score=0.8,
            token_count=100
        )

        assert context.content == "Test content"
        assert context.source_url == "https://example.com"
        assert context.chapter == "Chapter 1"
        assert context.section == "Section 1.1"
        assert context.chunk_id == "chunk123"
        assert context.relevance_score == 0.8
        assert context.token_count == 100


class TestConversationSession:
    """Tests for ConversationSession model"""

    def test_conversation_session_creation(self):
        """Test creating a valid ConversationSession"""
        session = ConversationSession(
            session_id="session123",
            created_at=datetime.now(),
            last_accessed=datetime.now(),
            messages=[]
        )

        assert session.session_id == "session123"


class TestMessage:
    """Tests for Message model"""

    def test_message_creation(self):
        """Test creating a valid Message"""
        message = Message(
            role=MessageRole.USER,
            content="Test message content"
        )

        assert message.role == MessageRole.USER
        assert message.content == "Test message content"