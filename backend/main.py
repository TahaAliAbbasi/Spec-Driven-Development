"""
RAG Chatbot Backend Service
Unified service with OpenAI Agent controller orchestrating RAG functionality
"""
import os
import logging
from typing import Optional
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from slowapi import Limiter, _rate_limit_exceeded_handler
from slowapi.util import get_remote_address
from slowapi.errors import RateLimitExceeded
from contextlib import asynccontextmanager

import uvicorn
from fastapi import FastAPI, HTTPException, Depends, Request, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from dotenv import load_dotenv
import asyncio
import json

# Load environment variables
load_dotenv()

# Import configuration
from config import Config

# Configure logging
logging.basicConfig(
    level=os.getenv("LOG_LEVEL", "INFO").upper(),
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Application lifespan for startup/shutdown events
@asynccontextmanager
async def lifespan(app: FastAPI):
    """Application lifespan for startup and shutdown events"""
    logger.info("Starting RAG Chatbot Backend Service...")
    # Startup logic can go here
    yield
    # Shutdown logic can go here
    logger.info("Shutting down RAG Chatbot Backend Service...")

def get_rate_limit_string(requests_per_window: int = None, window_seconds: int = None) -> str:
    """
    Generate a rate limit string based on configuration
    """
    if requests_per_window is None:
        requests_per_window = Config.RATE_LIMIT_REQUESTS
    if window_seconds is None:
        window_seconds = Config.RATE_LIMIT_WINDOW

    # Convert window seconds to appropriate time unit for rate limiting
    if window_seconds >= 3600:  # >= 1 hour
        window_str = f"{window_seconds // 3600}hour"
    elif window_seconds >= 60:  # >= 1 minute
        window_str = f"{window_seconds // 60}minute"
    else:  # seconds
        window_str = f"{window_seconds}second"

    return f"{requests_per_window}/{window_str}"

# Initialize rate limiter
limiter = Limiter(key_func=get_remote_address)

# Initialize FastAPI app
app = FastAPI(
    title="RAG Chatbot Backend API",
    description="API for RAG Chatbot with OpenAI Agent Controller",
    version="1.0.0",
    lifespan=lifespan
)

# Add rate limiter to app
app.state.limiter = limiter
app.add_exception_handler(RateLimitExceeded, _rate_limit_exceeded_handler)

# Security scheme for API key authentication
security_scheme = HTTPBearer()

def verify_api_key(credentials: HTTPAuthorizationCredentials = Depends(security_scheme)):
    """
    Verify the API key from the Authorization header
    """
    api_key = os.getenv("FRONTEND_API_KEY")  # In production, use a dedicated frontend API key
    if not api_key:
        # For development, skip API key verification
        return True

    if credentials.credentials != api_key:
        raise HTTPException(
            status_code=401,
            detail="Invalid API Key"
        )
    return True

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, configure this properly
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Basic health check endpoint
@app.get("/")
@limiter.limit(get_rate_limit_string(requests_per_window=150))  # Rate limit for root endpoint
async def root(request: Request):
    return {"message": "RAG Chatbot Backend Service is running"}

@app.get("/api/health")
@limiter.limit(get_rate_limit_string(requests_per_window=200))  # Higher limit for health checks
async def health_check(request: Request):
    """Health check endpoint"""
    return {
        "status": "healthy",
        "service": "rag-chatbot-backend",
        "timestamp": __import__('datetime').datetime.now().isoformat()
    }

from models.query import UserQuery
from models.response import GeneratedResponse
from agents.chat_agent import ChatAgent
from services.qdrant_service import QdrantService
from services.cohere_service import CohereService
from services.conversation_manager import ConversationManager
from services.response_service import ResponseService
from services.cache_service import cache_service
from config import Config

# Initialize services
chat_agent = ChatAgent()
qdrant_service = QdrantService()
cohere_service = CohereService()
conversation_manager = ConversationManager()
response_service = ResponseService()

@app.post("/api/chat", response_model=GeneratedResponse)
@limiter.limit(get_rate_limit_string())  # Rate limit from config
async def chat(request: Request, user_query: UserQuery, credentials: bool = Depends(verify_api_key)):
    """Chat endpoint managed by agent"""
    try:
        # Validate the query
        if not chat_agent.validate_query(user_query.query_text):
            raise HTTPException(status_code=400, detail="Invalid query provided")

        # Get or create session
        if not user_query.session_id:
            session_id = conversation_manager.create_session()
            user_query.session_id = session_id
        else:
            session_id = user_query.session_id

        # Add user message to session
        from models.message import Message, MessageRole
        user_message = Message(role=MessageRole.USER, content=user_query.query_text)
        conversation_manager.add_message(session_id, user_message)

        # Generate embedding for the query
        query_embedding = cohere_service.embed_text(user_query.query_text)

        # Search in Qdrant for relevant context
        search_results = qdrant_service.search(query_embedding, top_k=5)

        # Convert search results to RetrievedContext objects
        from models.context import RetrievedContext
        retrieved_context = []
        for result in search_results:
            context = RetrievedContext(
                content=result['content'],
                source_url=result['source_url'],
                chapter=result['chapter'],
                section=result['section'],
                chunk_id=result['chunk_id'],
                relevance_score=result['relevance_score'],
                token_count=result['token_count']
            )
            retrieved_context.append(context)

        # Process the query with the agent (retrieval is now handled internally)
        response = chat_agent.retrieve_and_process_query(user_query)

        # Add assistant message to session
        assistant_message = Message(role=MessageRole.ASSISTANT, content=response.response_text)
        conversation_manager.add_message(session_id, assistant_message)

        return response
    except Exception as e:
        logger.error(f"Error in chat endpoint: {str(e)}")
        raise HTTPException(status_code=500, detail="Internal server error during chat processing")

from pydantic import BaseModel
from typing import List, Optional

# Pydantic models for backward compatibility with existing service
class RetrievalRequest(BaseModel):
    query: Optional[str] = None
    selected_text: Optional[str] = None
    top_k: int = 5
    chapter_filter: Optional[str] = None
    section_filter: Optional[str] = None
    language_constraint: str = "en"
    version_constraint: str = "1.0"

class RetrievedChunk(BaseModel):
    content: str
    source_url: str
    chapter: str
    section: str
    chunk_id: str
    relevance_score: float
    token_count: int

class ContextBundle(BaseModel):
    retrieved_chunks: List[RetrievedChunk]

# Backward compatible retrieve endpoint
@app.post("/api/retrieve", response_model=ContextBundle)
@limiter.limit(get_rate_limit_string(requests_per_window=75))  # Slightly more permissive for retrieval
async def retrieve(request: Request, retrieval_request: RetrievalRequest):
    """Retrieve endpoint for backward compatibility with existing service"""
    try:
        # Generate cache key based on query and parameters
        cache_params = {
            'query': retrieval_request.query,
            'top_k': retrieval_request.top_k,
            'chapter_filter': retrieval_request.chapter_filter,
            'section_filter': retrieval_request.section_filter,
            'language_constraint': retrieval_request.language_constraint,
            'version_constraint': retrieval_request.version_constraint
        }
        cache_key = f"retrieve:{hash(str(sorted(cache_params.items())))}"

        # Try to get from cache first
        cached_result = cache_service.get(cache_key)
        if cached_result is not None:
            logger.info(f"Cache hit for retrieve endpoint with key: {cache_key}")
            return cached_result

        # Use the retrieval tool to get context
        if retrieval_request.query:
            retrieved_context = chat_agent.retrieval_tool.retrieve_context(retrieval_request.query, retrieval_request.top_k)
        else:
            # Fallback if no query provided
            result = ContextBundle(retrieved_chunks=[])
            cache_service.set(cache_key, result, ttl=300)  # Cache for 5 minutes
            return result

        # Convert to the expected format for backward compatibility
        chunks = []
        for ctx in retrieved_context:
            chunk = RetrievedChunk(
                content=ctx.content,
                source_url=ctx.source_url,
                chapter=ctx.chapter,
                section=ctx.section,
                chunk_id=ctx.chunk_id,
                relevance_score=ctx.relevance_score,
                token_count=ctx.token_count
            )
            chunks.append(chunk)

        result = ContextBundle(retrieved_chunks=chunks)

        # Cache the successful result
        cache_service.set(cache_key, result, ttl=300)  # Cache for 5 minutes
        logger.info(f"Cache set for retrieve endpoint with key: {cache_key}")

        return result
    except Exception as e:
        logger.error(f"Error in retrieve endpoint: {str(e)}")
        raise HTTPException(status_code=500, detail="Internal server error during retrieval")

class AgentInput(BaseModel):
    query: str
    context_bundle: Optional[ContextBundle] = None
    mode: str = "standard"

from models.citation import Citation

class AgentOutput(BaseModel):
    response: str
    status: str = "success"
    citations: List[Citation] = []
    confidence_score: float = 0.0

# Backward compatible answer endpoint
@app.post("/api/answer", response_model=AgentOutput)
@limiter.limit(get_rate_limit_string(requests_per_window=60))  # Rate limit for answer endpoint
async def answer(request: Request, agent_input: AgentInput):
    """Answer endpoint for backward compatibility with existing service"""
    try:
        # Generate cache key based on query and parameters
        cache_params = {
            'query': agent_input.query,
            'mode': agent_input.mode,
            'has_context_bundle': agent_input.context_bundle is not None
        }
        cache_key = f"answer:{hash(str(sorted(cache_params.items())))}"

        # Try to get from cache first
        cached_result = cache_service.get(cache_key)
        if cached_result is not None:
            logger.info(f"Cache hit for answer endpoint with key: {cache_key}")
            return cached_result

        # Create a user query object from the input
        from models.query import UserQuery
        user_query = UserQuery(query_text=agent_input.query)

        # If context bundle is provided, use it; otherwise retrieve context
        if agent_input.context_bundle:
            # Convert context bundle to RetrievedContext objects
            from models.context import RetrievedContext
            retrieved_context = []
            for chunk in agent_input.context_bundle.retrieved_chunks:
                ctx = RetrievedContext(
                    content=chunk.content,
                    source_url=chunk.source_url,
                    chapter=chunk.chapter,
                    section=chunk.section,
                    chunk_id=chunk.chunk_id,
                    relevance_score=chunk.relevance_score,
                    token_count=chunk.token_count
                )
                retrieved_context.append(ctx)

            # Process with the provided context
            response = chat_agent.process_query(user_query, retrieved_context)
        else:
            # Retrieve context automatically
            response = chat_agent.retrieve_and_process_query(user_query)

        # Convert to the expected format for backward compatibility
        output = AgentOutput(
            response=response.response_text,
            status="success",
            citations=response.citations,
            confidence_score=response.confidence_score
        )

        # Cache the successful result
        cache_service.set(cache_key, output, ttl=600)  # Cache for 10 minutes (responses take more resources to generate)
        logger.info(f"Cache set for answer endpoint with key: {cache_key}")

        return output
    except Exception as e:
        logger.error(f"Error in answer endpoint: {str(e)}")
        raise HTTPException(status_code=500, detail="Internal server error during answer generation")

# WebSocket connection manager
class ConnectionManager:
    def __init__(self):
        self.active_connections: list[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        self.active_connections.remove(websocket)

    async def send_personal_message(self, message: str, websocket: WebSocket):
        await websocket.send_text(message)

    async def broadcast(self, message: str):
        for connection in self.active_connections:
            try:
                await connection.send_text(message)
            except:
                # If sending fails, remove the connection
                self.disconnect(connection)


manager = ConnectionManager()


@app.websocket("/ws/chat")
async def websocket_chat(websocket: WebSocket):
    """
    WebSocket endpoint for real-time chat communication
    """
    await manager.connect(websocket)
    try:
        while True:
            # Receive message from the client
            data = await websocket.receive_text()

            try:
                # Parse the received data as JSON
                message_data = json.loads(data)
                query_text = message_data.get("query", "")
                session_id = message_data.get("session_id", None)

                # Validate the query
                if not query_text or not chat_agent.validate_query(query_text):
                    error_response = {
                        "type": "error",
                        "message": "Invalid query provided",
                        "timestamp": __import__('datetime').datetime.now().isoformat()
                    }
                    await manager.send_personal_message(json.dumps(error_response), websocket)
                    continue

                # Create a UserQuery object
                user_query = UserQuery(query_text=query_text, session_id=session_id)

                # Get or create session
                if not user_query.session_id:
                    session_id = conversation_manager.create_session()
                    user_query.session_id = session_id
                else:
                    session_id = user_query.session_id

                # Add user message to session
                from models.message import Message, MessageRole
                user_message = Message(role=MessageRole.USER, content=user_query.query_text)
                conversation_manager.add_message(session_id, user_message)

                # Send typing indicator
                typing_response = {
                    "type": "typing",
                    "message": "Processing your query...",
                    "timestamp": __import__('datetime').datetime.now().isoformat()
                }
                await manager.send_personal_message(json.dumps(typing_response), websocket)

                # Process the query with the agent
                response = chat_agent.retrieve_and_process_query(user_query)

                # Add assistant message to session
                assistant_message = Message(role=MessageRole.ASSISTANT, content=response.response_text)
                conversation_manager.add_message(session_id, assistant_message)

                # Send the response back to the client
                response_data = {
                    "type": "response",
                    "response": response.response_text,
                    "citations": [citation.model_dump() for citation in response.citations],
                    "confidence_score": response.confidence_score,
                    "session_id": session_id,
                    "timestamp": __import__('datetime').datetime.now().isoformat()
                }

                await manager.send_personal_message(json.dumps(response_data), websocket)

            except json.JSONDecodeError:
                error_response = {
                    "type": "error",
                    "message": "Invalid JSON format",
                    "timestamp": __import__('datetime').datetime.now().isoformat()
                }
                await manager.send_personal_message(json.dumps(error_response), websocket)
            except Exception as e:
                error_response = {
                    "type": "error",
                    "message": f"Error processing message: {str(e)}",
                    "timestamp": __import__('datetime').datetime.now().isoformat()
                }
                await manager.send_personal_message(json.dumps(error_response), websocket)

    except WebSocketDisconnect:
        manager.disconnect(websocket)
        logger.info("WebSocket connection disconnected")


if __name__ == "__main__":
    port = int(os.getenv("PORT", "8000"))
    uvicorn.run(app, host="0.0.0.0", port=port)