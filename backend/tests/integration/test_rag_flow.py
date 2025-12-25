"""
Integration tests for the complete RAG flow
"""
import pytest
import os
from unittest.mock import Mock, patch, MagicMock
from backend.agents.chat_agent import ChatAgent
from backend.models.query import UserQuery
from backend.models.context import RetrievedContext
from backend.services.qdrant_service import QdrantService
from backend.services.cohere_service import CohereService
from backend.services.conversation_manager import ConversationManager
from backend.services.response_service import ResponseService


class TestRAGFlowIntegration:
    """Integration tests for the complete RAG pipeline"""

    def setup_method(self):
        """Setup for each test"""
        # Mock the external services to avoid actual API calls during testing
        self.mock_qdrant = Mock(spec=QdrantService)
        self.mock_cohere = Mock(spec=CohereService)
        self.mock_conversation = Mock(spec=ConversationManager)
        self.mock_response_service = Mock(spec=ResponseService)

    @patch('backend.agents.chat_agent.QdrantService')
    @patch('backend.agents.chat_agent.CohereService')
    @patch('backend.agents.chat_agent.ConversationManager')
    @patch('backend.agents.chat_agent.ResponseService')
    def test_complete_rag_flow(self, mock_response_service, mock_conversation, mock_cohere, mock_qdrant):
        """Test the complete RAG flow from query to response"""
        # Mock the services
        mock_qdrant_instance = Mock()
        mock_qdrant.return_value = mock_qdrant_instance

        mock_cohere_instance = Mock()
        mock_cohere.return_value = mock_cohere_instance

        mock_conversation_instance = Mock()
        mock_conversation.return_value = mock_conversation_instance

        mock_response_instance = Mock()
        mock_response_service.return_value = mock_response_instance

        # Mock the embedding generation
        mock_cohere_instance.embed_text.return_value = [0.1, 0.2, 0.3, 0.4]

        # Mock the Qdrant search results
        mock_search_result = [
            {
                'content': 'Test content from knowledge base',
                'source_url': 'https://example.com/test',
                'chapter': 'Chapter 1',
                'section': 'Section 1.1',
                'chunk_id': 'chunk123',
                'relevance_score': 0.85,
                'token_count': 100
            }
        ]
        mock_qdrant_instance.search.return_value = mock_search_result

        # Mock conversation management
        mock_conversation_instance.create_session.return_value = "session123"
        mock_conversation_instance.add_message.return_value = None

        # Create the agent
        agent = ChatAgent()

        # Mock the OpenAI client response
        mock_completion = Mock()
        mock_completion.choices = [Mock()]
        mock_completion.choices[0].message = Mock()
        mock_completion.choices[0].message.content = "This is a generated response based on the context."

        # Mock the OpenAI client
        with patch.object(agent.client.chat.completions, 'create', return_value=mock_completion):
            # Execute the complete RAG flow
            user_query = UserQuery(query_text="What is Physical AI?")
            response = agent.retrieve_and_process_query(user_query)

            # Assertions
            assert response is not None
            assert "generated response" in response.response_text.lower()
            assert response.confidence_score >= 0.0

            # Verify that all steps in the RAG flow were called
            mock_cohere_instance.embed_text.assert_called_once_with("What is Physical AI?")
            mock_qdrant_instance.search.assert_called_once()
            mock_conversation_instance.add_message.assert_called()

    @patch('backend.agents.chat_agent.ChatAgent.validate_query', return_value=True)
    @patch('backend.services.qdrant_service.QdrantClient')
    @patch('backend.services.cohere_service.Client')
    def test_rag_flow_with_context_retrieval(self, mock_cohere_client, mock_qdrant_client, mock_validate):
        """Test RAG flow with proper context retrieval and response generation"""
        # Create a real agent but with mocked external dependencies
        agent = ChatAgent()

        # Mock the services within the agent
        agent.cohere_service = Mock()
        agent.qdrant_service = Mock()
        agent.conversation_manager = Mock()
        agent.response_service = Mock()

        # Mock embedding and search
        agent.cohere_service.embed_text.return_value = [0.5, 0.6, 0.7, 0.8]

        mock_search_results = [
            {
                'content': 'Physical AI combines physics simulation with AI to create embodied intelligence.',
                'source_url': 'https://example.com/physical-ai',
                'chapter': 'Introduction',
                'section': 'Definition',
                'chunk_id': 'chunk456',
                'relevance_score': 0.92,
                'token_count': 85
            }
        ]
        agent.qdrant_service.search.return_value = mock_search_results

        # Mock conversation management
        agent.conversation_manager.create_session.return_value = "session456"
        agent.conversation_manager.add_message.return_value = None

        # Mock response service
        agent.response_service.is_confidence_sufficient.return_value = True

        # Mock OpenAI response
        mock_completion = Mock()
        mock_completion.choices = [Mock()]
        mock_completion.choices[0].message = Mock()
        mock_completion.choices[0].message.content = "Physical AI is an approach that combines physics simulation with artificial intelligence to create embodied intelligence systems."

        with patch.object(agent.client.chat.completions, 'create', return_value=mock_completion):
            # Execute the RAG flow
            user_query = UserQuery(query_text="Explain Physical AI")
            response = agent.retrieve_and_process_query(user_query)

            # Verify the response
            assert response.response_text is not None
            assert len(response.response_text) > 0
            assert response.confidence_score >= 0.0

            # Verify the flow steps
            agent.cohere_service.embed_text.assert_called_once()
            agent.qdrant_service.search.assert_called_once()
            agent.conversation_manager.add_message.assert_called()

    def test_rag_flow_error_handling(self):
        """Test RAG flow error handling for various failure scenarios"""
        agent = ChatAgent()

        # Mock services to simulate failures
        agent.cohere_service = Mock()
        agent.qdrant_service = Mock()
        agent.conversation_manager = Mock()

        # Test case: Cohere embedding fails
        agent.cohere_service.embed_text.side_effect = Exception("Cohere API error")

        user_query = UserQuery(query_text="Test query")

        # This should handle the error gracefully
        try:
            # We'll mock the flow to avoid actual API calls
            retrieved_context = []
            response = agent.process_query(user_query, retrieved_context)
            assert response is not None
        except Exception as e:
            # If an exception is raised, it should be handled appropriately
            assert "error" in str(e).lower() or "exception" in str(e).lower()

    @patch('backend.agents.chat_agent.ChatAgent.validate_query', return_value=True)
    def test_rag_flow_with_empty_context(self, mock_validate):
        """Test RAG flow when no context is retrieved"""
        agent = ChatAgent()

        # Mock services
        agent.cohere_service = Mock()
        agent.qdrant_service = Mock()
        agent.conversation_manager = Mock()

        # Mock embedding and search (return empty results)
        agent.cohere_service.embed_text.return_value = [0.1, 0.2]
        agent.qdrant_service.search.return_value = []  # No results

        # Mock conversation management
        agent.conversation_manager.create_session.return_value = "session789"
        agent.conversation_manager.add_message.return_value = None

        # Mock OpenAI response for when no context is available
        mock_completion = Mock()
        mock_completion.choices = [Mock()]
        mock_completion.choices[0].message = Mock()
        mock_completion.choices[0].message.content = "I couldn't find specific information about that in the knowledge base, but I can provide general information."

        with patch.object(agent.client.chat.completions, 'create', return_value=mock_completion):
            user_query = UserQuery(query_text="Unknown topic query")
            response = agent.retrieve_and_process_query(user_query)

            # Should still return a response even with no context
            assert response.response_text is not None
            assert len(response.response_text) > 0

    @patch('backend.agents.chat_agent.ChatAgent.validate_query', return_value=True)
    def test_conversation_context_preservation(self, mock_validate):
        """Test that conversation context is preserved across multiple queries"""
        agent = ChatAgent()

        # Mock services
        agent.cohere_service = Mock()
        agent.qdrant_service = Mock()
        agent.conversation_manager = Mock()
        agent.response_service = Mock()

        # Setup mocks
        agent.cohere_service.embed_text.return_value = [0.5, 0.6]
        agent.qdrant_service.search.return_value = [
            {
                'content': 'Test context for conversation',
                'source_url': 'https://example.com',
                'chapter': 'Test',
                'section': 'Test',
                'chunk_id': 'test_chunk',
                'relevance_score': 0.8,
                'token_count': 50
            }
        ]

        # Mock conversation management
        agent.conversation_manager.create_session.return_value = "persistent_session"
        agent.conversation_manager.add_message.return_value = None
        agent.conversation_manager.get_session.return_value = Mock()
        agent.conversation_manager.get_session.return_value.messages = []

        # Mock response service
        agent.response_service.is_confidence_sufficient.return_value = True

        # Mock OpenAI response
        mock_completion = Mock()
        mock_completion.choices = [Mock()]
        mock_completion.choices[0].message = Mock()
        mock_completion.choices[0].message.content = "Response considering conversation context."

        with patch.object(agent.client.chat.completions, 'create', return_value=mock_completion):
            # First query
            user_query1 = UserQuery(query_text="First query", session_id="persistent_session")
            response1 = agent.retrieve_and_process_query(user_query1)

            # Second query in same session
            user_query2 = UserQuery(query_text="Follow-up query", session_id="persistent_session")
            response2 = agent.retrieve_and_process_query(user_query2)

            # Verify both responses were generated
            assert response1.response_text is not None
            assert response2.response_text is not None

            # Verify conversation manager was called for both queries
            assert agent.conversation_manager.add_message.call_count == 2


class TestEndToEndIntegration:
    """End-to-end integration tests"""

    @patch('backend.agents.chat_agent.QdrantService')
    @patch('backend.agents.chat_agent.CohereService')
    @patch('backend.agents.chat_agent.ConversationManager')
    def test_end_to_end_rag_process(self, mock_conversation, mock_cohere, mock_qdrant):
        """Test the complete end-to-end RAG process"""
        # Mock all external dependencies
        mock_qdrant_instance = Mock()
        mock_qdrant.return_value = mock_qdrant_instance

        mock_cohere_instance = Mock()
        mock_cohere.return_value = mock_cohere_instance

        mock_conversation_instance = Mock()
        mock_conversation.return_value = mock_conversation_instance

        # Setup mock returns
        mock_cohere_instance.embed_text.return_value = [0.1, 0.2, 0.3]
        mock_qdrant_instance.search.return_value = [
            {
                'content': 'Relevant information about the topic',
                'source_url': 'https://example.com/topic',
                'chapter': 'Topic Chapter',
                'section': 'Topic Section',
                'chunk_id': 'relevant_chunk',
                'relevance_score': 0.88,
                'token_count': 95
            }
        ]
        mock_conversation_instance.create_session.return_value = "test_session"
        mock_conversation_instance.add_message.return_value = None

        # Create agent and execute full flow
        agent = ChatAgent()

        # Mock OpenAI response
        mock_completion = Mock()
        mock_completion.choices = [Mock()]
        mock_completion.choices[0].message = Mock()
        mock_completion.choices[0].message.content = "This is a comprehensive answer based on the retrieved context."

        with patch.object(agent.client.chat.completions, 'create', return_value=mock_completion):
            user_query = UserQuery(query_text="Comprehensive topic question")
            response = agent.retrieve_and_process_query(user_query)

            # Verify complete flow execution
            assert response is not None
            assert "comprehensive answer" in response.response_text.lower()

            # Verify all components were engaged
            mock_cohere_instance.embed_text.assert_called_once()
            mock_qdrant_instance.search.assert_called_once()
            mock_conversation_instance.add_message.assert_called()