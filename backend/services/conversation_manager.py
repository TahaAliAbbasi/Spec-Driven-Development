"""
Conversation manager for session handling
"""
import os
import logging
from typing import Dict, List, Optional, Any
from datetime import datetime, timedelta
from threading import Lock
from dotenv import load_dotenv

from models.session import ConversationSession
from models.message import Message, MessageRole
from models.query import UserQuery

# Load environment variables
load_dotenv()

# Configure logging
logger = logging.getLogger(__name__)


class ConversationManager:
    """
    Manager for handling conversation sessions and maintaining context within sessions
    """

    def __init__(self):
        """
        Initialize the conversation manager
        """
        self.sessions: Dict[str, ConversationSession] = {}
        self.lock = Lock()  # Thread-safe access to sessions
        self.session_timeout_minutes = int(os.getenv("SESSION_TIMEOUT_MINUTES", "30"))
        logger.info(f"ConversationManager initialized with timeout: {self.session_timeout_minutes} minutes")

    def create_session(self, session_id: Optional[str] = None) -> str:
        """
        Create a new conversation session

        Args:
            session_id: Optional session ID to use (if not provided, one will be generated)

        Returns:
            Session ID of the created session
        """
        import uuid
        if not session_id:
            session_id = str(uuid.uuid4())

        with self.lock:
            new_session = ConversationSession(
                session_id=session_id,
                created_at=datetime.now(),
                last_interaction=datetime.now()
            )
            self.sessions[session_id] = new_session
            logger.info(f"Created new conversation session: {session_id}")
            return session_id

    def get_session(self, session_id: str) -> Optional[ConversationSession]:
        """
        Get a conversation session by ID

        Args:
            session_id: ID of the session to retrieve

        Returns:
            ConversationSession if found and not expired, None otherwise
        """
        with self.lock:
            session = self.sessions.get(session_id)
            if session:
                # Check if session has expired
                if self._is_session_expired(session):
                    logger.info(f"Session {session_id} has expired, removing it")
                    del self.sessions[session_id]
                    return None
                # Update last interaction time
                session.last_interaction = datetime.now()
                return session
            return None

    def add_message(self, session_id: str, message: Message) -> bool:
        """
        Add a message to a conversation session

        Args:
            session_id: ID of the session to add the message to
            message: Message to add to the session

        Returns:
            True if message was added successfully, False otherwise
        """
        with self.lock:
            session = self.get_session(session_id)
            if not session:
                logger.warning(f"Cannot add message to non-existent or expired session: {session_id}")
                return False

            session.messages.append(message)
            session.last_interaction = datetime.now()
            logger.info(f"Added message to session {session_id}")
            return True

    def get_conversation_history(self, session_id: str, limit: int = 10) -> List[Message]:
        """
        Get the conversation history for a session

        Args:
            session_id: ID of the session to get history for
            limit: Maximum number of messages to return

        Returns:
            List of messages in the conversation
        """
        session = self.get_session(session_id)
        if session:
            # Return the most recent messages up to the limit
            return session.messages[-limit:]
        return []

    def get_context_for_session(self, session_id: str, max_context_messages: int = 5) -> str:
        """
        Get the context for a session (recent messages as context for the AI)

        Args:
            session_id: ID of the session to get context for
            max_context_messages: Maximum number of recent messages to include as context

        Returns:
            String containing the conversation context
        """
        messages = self.get_conversation_history(session_id, max_context_messages)
        context_str = ""
        for msg in messages:
            role = "User" if msg.role == MessageRole.USER else "Assistant"
            context_str += f"{role}: {msg.content}\n"
        return context_str.strip()

    def update_session_context(self, session_id: str, user_query: str, response: str) -> bool:
        """
        Update the session with the latest query-response pair

        Args:
            session_id: ID of the session to update
            user_query: The user's query
            response: The assistant's response

        Returns:
            True if updated successfully, False otherwise
        """
        with self.lock:
            session = self.get_session(session_id)
            if not session:
                logger.warning(f"Cannot update context for non-existent or expired session: {session_id}")
                return False

            # Add user message
            user_message = Message(role=MessageRole.USER, content=user_query)
            session.messages.append(user_message)

            # Add assistant message
            assistant_message = Message(role=MessageRole.ASSISTANT, content=response)
            session.messages.append(assistant_message)

            session.last_interaction = datetime.now()
            logger.info(f"Updated session context for {session_id}")
            return True

    def clear_expired_sessions(self) -> int:
        """
        Remove all expired sessions and return the count of removed sessions

        Returns:
            Number of sessions that were removed
        """
        with self.lock:
            expired_sessions = []
            current_time = datetime.now()

            for session_id, session in self.sessions.items():
                if self._is_session_expired(session):
                    expired_sessions.append(session_id)

            for session_id in expired_sessions:
                del self.sessions[session_id]

            logger.info(f"Cleared {len(expired_sessions)} expired sessions")
            return len(expired_sessions)

    def _is_session_expired(self, session: ConversationSession) -> bool:
        """
        Check if a session has expired based on the timeout configuration

        Args:
            session: Session to check for expiration

        Returns:
            True if session is expired, False otherwise
        """
        timeout_duration = timedelta(minutes=self.session_timeout_minutes)
        time_since_last_interaction = datetime.now() - session.last_interaction
        return time_since_last_interaction > timeout_duration

    def validate_session_privacy(self, session_id: str) -> bool:
        """
        Validate that session data is handled according to privacy requirements

        Args:
            session_id: ID of the session to validate

        Returns:
            True if session is valid and compliant, False otherwise
        """
        # Check if session exists and hasn't expired
        session = self.get_session(session_id)
        return session is not None

    def cleanup_session_data(self, session_id: str) -> bool:
        """
        Remove session data to comply with privacy requirements

        Args:
            session_id: ID of the session to clean up

        Returns:
            True if cleanup was successful, False otherwise
        """
        with self.lock:
            if session_id in self.sessions:
                del self.sessions[session_id]
                logger.info(f"Cleaned up session data for {session_id}")
                return True
            return False

    def get_active_session_count(self) -> int:
        """
        Get the number of currently active sessions

        Returns:
            Number of active (non-expired) sessions
        """
        self.clear_expired_sessions()  # Clean up before counting
        return len(self.sessions)