"""
Unit tests for agents
"""
import pytest
from unittest.mock import Mock, patch, MagicMock
from backend.agents.chat_agent import ChatAgent
from backend.models.query import UserQuery
from backend.models.context import RetrievedContext


class TestChatAgent:
    """Tests for ChatAgent"""

    @patch('backend.agents.chat_agent.openai')
    def test_chat_agent_initialization(self, mock_openai):
        """Test ChatAgent initialization"""
        agent = ChatAgent()
        assert agent.client is not None
        assert agent.assistant_instructions is not None
        assert agent.assistant_model is not None
        assert agent.retrieval_tool is not None
        assert agent.response_tool is not None

    def test_validate_query(self):
        """Test query validation functionality"""
        agent = ChatAgent()

        # Valid queries
        assert agent.validate_query("What is AI?") == True
        assert agent.validate_query("How does machine learning work?") == True

        # Invalid queries
        assert agent.validate_query("") == False
        assert agent.validate_query("   ") == False
        assert agent.validate_query("?") == False  # Too short
        assert agent.validate_query("x") == False  # Too short

    @patch('backend.agents.chat_agent.ChatAgent.retrieve_and_process_query')
    def test_process_query(self, mock_retrieve_and_process):
        """Test process query functionality"""
        agent = ChatAgent()

        # Mock the retrieval and processing
        mock_response = Mock()
        mock_response.response_text = "Test response"
        mock_retrieve_and_process.return_value = mock_response

        user_query = UserQuery(query_text="Test query")
        response = agent.process_query(user_query)

        assert response.response_text == "Test response"

    @patch('backend.agents.chat_agent.ChatAgent.retrieve_context')
    @patch('backend.agents.chat_agent.ChatAgent.generate_response')
    def test_retrieve_and_process_query(self, mock_generate_response, mock_retrieve_context):
        """Test retrieve and process query functionality"""
        agent = ChatAgent()

        # Mock the context retrieval
        mock_context = [
            RetrievedContext(
                content="Test context content",
                source_url="https://example.com",
                chapter="Chapter 1",
                section="Section 1",
                chunk_id="chunk123",
                relevance_score=0.8,
                token_count=100
            )
        ]
        mock_retrieve_context.return_value = mock_context

        # Mock the response generation
        from backend.models.response import GeneratedResponse
        mock_response = GeneratedResponse(
            response_text="Generated response",
            citations=[],
            confidence_score=0.85
        )
        mock_generate_response.return_value = mock_response

        user_query = UserQuery(query_text="Test query")
        result = agent.retrieve_and_process_query(user_query)

        assert result.response_text == "Generated response"
        assert result.confidence_score == 0.85
        mock_retrieve_context.assert_called_once_with(user_query)
        mock_generate_response.assert_called_once_with(user_query, mock_context)

    def test_query_length_validation(self):
        """Test that very long queries are rejected"""
        agent = ChatAgent()

        # Very long query should be rejected
        long_query = "A" * 2000  # Much longer than allowed
        assert agent.validate_query(long_query) == False

    def test_special_characters_handling(self):
        """Test handling of special characters in queries"""
        agent = ChatAgent()

        # Queries with special characters should be valid
        assert agent.validate_query("What is AI & ML?") == True
        assert agent.validate_query("How about AI... ML?") == True
        assert agent.validate_query("AI vs ML: What's better?") == True