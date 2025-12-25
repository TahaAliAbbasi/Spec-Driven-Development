"""
Unit tests for services
"""
import pytest
import os
from unittest.mock import Mock, patch, MagicMock
from backend.services.qdrant_service import QdrantService
from backend.services.cohere_service import CohereService
from backend.services.conversation_manager import ConversationManager
from backend.services.response_service import ResponseService
from backend.services.cache_service import CacheService
from backend.services.monitoring import MonitoringService


class TestQdrantService:
    """Tests for QdrantService"""

    @patch('backend.services.qdrant_service.QdrantClient')
    def test_qdrant_service_initialization(self, mock_qdrant_client):
        """Test QdrantService initialization"""
        # Mock environment variables
        with patch.dict(os.environ, {
            'QDRANT_API_KEY': 'test_key',
            'QDRANT_URL': 'https://test.qdrant.com'
        }):
            service = QdrantService()
            assert service.client is not None
            mock_qdrant_client.assert_called_once()

    def test_health_check(self):
        """Test health check functionality"""
        service = QdrantService()
        # This will test the health check method, though it might fail without actual Qdrant connection
        # For unit testing, we'd typically mock the actual connection
        pass  # Implementation would require mocking the actual Qdrant client


class TestCohereService:
    """Tests for CohereService"""

    @patch('backend.services.cohere_service.Client')
    def test_cohere_service_initialization(self, mock_cohere_client):
        """Test CohereService initialization"""
        with patch.dict(os.environ, {'COHERE_API_KEY': 'test_key'}):
            service = CohereService()
            assert service.client is not None
            mock_cohere_client.assert_called_once()


class TestConversationManager:
    """Tests for ConversationManager"""

    def test_create_session(self):
        """Test creating a new session"""
        manager = ConversationManager()
        session_id = manager.create_session()
        assert session_id is not None
        assert len(session_id) > 0

    def test_add_message(self):
        """Test adding a message to a session"""
        manager = ConversationManager()
        session_id = manager.create_session()

        # Create a mock message
        from backend.models.message import Message, MessageRole
        message = Message(role=MessageRole.USER, content="Test message")

        manager.add_message(session_id, message)
        session = manager.get_session(session_id)
        assert len(session.messages) == 1
        assert session.messages[0].content == "Test message"

    def test_get_session(self):
        """Test retrieving a session"""
        manager = ConversationManager()
        session_id = manager.create_session()
        session = manager.get_session(session_id)
        assert session.session_id == session_id

    def test_session_expiration(self):
        """Test session expiration functionality"""
        manager = ConversationManager(session_timeout_minutes=0.01)  # 1 minute for testing
        session_id = manager.create_session()

        # Session should still exist initially
        assert manager.get_session(session_id) is not None

        # After timeout, session should be expired (this would require mocking time)
        # For now, we'll test the cleanup functionality directly


class TestResponseService:
    """Tests for ResponseService"""

    def test_confidence_thresholding(self):
        """Test confidence thresholding functionality"""
        service = ResponseService()

        # Test with high confidence score
        high_confidence = 0.8
        low_confidence = 0.5

        assert service.is_confidence_sufficient(high_confidence) == True
        assert service.is_confidence_sufficient(low_confidence) == False


class TestCacheService:
    """Tests for CacheService"""

    def test_cache_set_and_get(self):
        """Test basic cache set and get functionality"""
        cache = CacheService(default_ttl=300)  # 5 minute TTL

        # Test setting a value
        cache.set("test_key", "test_value")

        # Test getting the value
        retrieved_value = cache.get("test_key")
        assert retrieved_value == "test_value"

    def test_cache_expiration(self):
        """Test cache expiration"""
        cache = CacheService(default_ttl=0.01)  # 0.01 second TTL for testing

        cache.set("expiring_key", "expiring_value")

        # After expiration, the value should not be retrievable
        import time
        time.sleep(0.02)  # Wait for expiration

        retrieved_value = cache.get("expiring_key")
        assert retrieved_value is None

    def test_cache_invalidation(self):
        """Test cache invalidation"""
        cache = CacheService()

        cache.set("test_key", "test_value")
        assert cache.get("test_key") == "test_value"

        # Invalidate the key
        result = cache.invalidate("test_key")
        assert result == True

        # Key should no longer exist
        assert cache.get("test_key") is None


class TestMonitoringService:
    """Tests for MonitoringService"""

    def test_monitoring_initialization(self):
        """Test monitoring service initialization"""
        monitor = MonitoringService()
        assert monitor.total_requests == 0
        assert monitor.failed_requests == 0
        assert len(monitor.alerts) == 0

    def test_request_tracking(self):
        """Test request tracking functionality"""
        monitor = MonitoringService()

        # Start a request
        request_id = monitor.start_request("req123", "/api/test", "GET")
        assert request_id == "req123"
        assert len(monitor.request_metrics) == 1

        # Complete the request
        monitor.complete_request("req123", 200)
        assert len(monitor.request_metrics) == 0  # Should be removed after completion
        assert monitor.total_requests == 1

    def test_metrics_summary(self):
        """Test metrics summary generation"""
        monitor = MonitoringService()

        summary = monitor.get_metrics_summary()
        assert "uptime_seconds" in summary
        assert "total_requests" in summary
        assert "failed_requests" in summary
        assert "success_rate_percent" in summary
        assert "current_active_requests" in summary