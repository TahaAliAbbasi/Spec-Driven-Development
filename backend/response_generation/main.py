from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from dotenv import load_dotenv
import logging

from models.agent_models import AgentInput, AgentOutput
from agents.rag_answer_agent import RAGAnswerAgent
from utils.logging_utils import setup_logging
from utils.exceptions import ConstitutionalViolationError, ContextInsufficientError, SelectedTextOnlyViolationError

# Load environment variables
load_dotenv()

# Setup logging
setup_logging()
logger = logging.getLogger(__name__)

app = FastAPI(
    title="Phase 3 - Agent-Based Response Generation API",
    description="API for generating responses using RAG agents with constitutional compliance",
    version="1.0.0"
)

# Add CORS middleware if needed
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, configure this properly
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize the RAG Answer Agent
rag_agent = RAGAnswerAgent()

@app.get("/")
async def root():
    return {"message": "Phase 3 Response Generation Service"}

@app.get("/api/health/answer")
async def health_check():
    return {"status": "healthy", "service": "answer-generation"}

@app.post("/api/answer", response_model=AgentOutput)
async def generate_answer(agent_input: AgentInput):
    """
    Generate a response based on user query and context bundle.

    This endpoint processes a user query with provided context to generate
    a constitutionally-compliant response with proper citations.
    """
    try:
        logger.info(f"Received request with mode: {agent_input.mode}")

        # Process the request using the RAG agent
        result = rag_agent.process_request(agent_input)

        logger.info(f"Request processed successfully with status: {result.status}")
        return result

    except ConstitutionalViolationError as e:
        logger.error(f"Constitutional violation: {str(e)}")
        raise HTTPException(status_code=422, detail={
            "error": "CONSTITUTIONAL_VIOLATION",
            "message": str(e)
        })
    except ContextInsufficientError as e:
        logger.warning(f"Context insufficient: {str(e)}")
        raise HTTPException(status_code=422, detail={
            "error": "CONTEXT_INSUFFICIENT",
            "message": str(e)
        })
    except SelectedTextOnlyViolationError as e:
        logger.warning(f"Selected-text-only violation: {str(e)}")
        raise HTTPException(status_code=422, detail={
            "error": "SELECTED_TEXT_ONLY_VIOLATION",
            "message": str(e)
        })
    except Exception as e:
        logger.error(f"Unexpected error processing request: {str(e)}")
        raise HTTPException(status_code=500, detail={
            "error": "AGENT_PROCESSING_ERROR",
            "message": "An error occurred during agent processing"
        })

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="localhost", port=8000)