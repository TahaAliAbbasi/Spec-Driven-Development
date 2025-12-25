"""RAGAnswerAgent - Primary agent for context-grounded response generation"""


from typing import Dict, Any, List, Optional
from models.agent_models import AgentInput, AgentOutput, ChunkReference, StatusEnum
from .openrouter_provider_adapter import OpenRouterProviderAdapter
from utils.text_utils import count_tokens, validate_context_sufficiency, extract_sentences, find_chunk_for_content
from utils.context_utils import normalize_context_bundle, filter_chunks_by_mode
from utils.exceptions import ConstitutionalViolationError, ContextInsufficientError, SelectedTextOnlyViolationError
from utils.logging_utils import get_logger, log_constitutional_check, log_citation_trace
from config import settings
import logging


class RAGAnswerAgent:
    """
    Primary agent responsible for context-grounded response generation.
    This agent receives user queries and ContextBundles, then generates responses
    with proper source citations while enforcing constitutional compliance.
    """

    def __init__(self):
        """Initialize the RAG Answer Agent with required components."""
        self.provider_adapter = OpenRouterProviderAdapter()
        self.logger = get_logger(__name__)

        # Fixed system prompt for deterministic behavior
        self.system_prompt = (
            "You are a helpful AI assistant that answers questions based strictly on the provided context. "
            "Follow these rules carefully:\n"
            "1. Only use information from the provided context to answer questions.\n"
            "2. Do not generate any information not present in the provided context.\n"
            "3. If the context is insufficient to answer the query, respond with 'I cannot answer this question based on the provided context.'\n"
            "4. Provide specific citations for information when possible.\n"
            "5. Maintain a professional and helpful tone."
        )

    def process_request(self, agent_input: AgentInput) -> AgentOutput:
        """
        Process an agent input and generate a response with citations.

        Args:
            agent_input: The input containing query, context_bundle, and mode

        Returns:
            AgentOutput containing the answer, citations, and status
        """
        try:
            self.logger.info(f"Processing request with mode: {agent_input.mode}")

            # Apply constitutional validation
            self._validate_constitutional_requirements(agent_input)

            # Normalize and filter context based on mode
            normalized_context = normalize_context_bundle(agent_input.context_bundle)
            filtered_context = filter_chunks_by_mode(normalized_context, agent_input.mode)

            # Check context sufficiency
            if not validate_context_sufficiency(filtered_context, agent_input.query):
                self.logger.warning("Context is insufficient to answer the query")
                return AgentOutput(
                    answer="I cannot answer this question based on the provided context.",
                    citations=[],
                    used_chunks=[],
                    status=StatusEnum.INSUFFICIENT_CONTEXT
                )

            # Generate response using the provider adapter
            response_data = self.provider_adapter.generate_response(
                query=agent_input.query,
                context_chunks=filtered_context.get('chunks', []),
                system_prompt=self.system_prompt
            )

            # Extract citations from the response
            citations_data = self.provider_adapter.extract_citations_from_response(
                response_text=response_data["text"],
                context_chunks=filtered_context.get('chunks', [])
            )

            # Convert citations to proper format
            citations = [
                ChunkReference(
                    chunk_id=citation["chunk_id"],
                    source_url=citation["source_url"]
                )
                for citation in citations_data
            ]

            # Extract used chunk IDs
            used_chunks = [citation.chunk_id for citation in citations]

            # Create and return the AgentOutput
            agent_output = AgentOutput(
                answer=response_data["text"],
                citations=citations,
                used_chunks=used_chunks,
                status=StatusEnum.ANSWERED
            )

            self.logger.info(f"Successfully generated response with {len(citations)} citations")
            return agent_output

        except ContextInsufficientError:
            return AgentOutput(
                answer="I cannot answer this question based on the provided context.",
                citations=[],
                used_chunks=[],
                status=StatusEnum.INSUFFICIENT_CONTEXT
            )
        except SelectedTextOnlyViolationError as e:
            self.logger.warning(f"Selected-text-only violation: {str(e)}")
            return AgentOutput(
                answer="I cannot answer this question due to selected-text-only mode restrictions.",
                citations=[],
                used_chunks=[],
                status=StatusEnum.REFUSED,
                warnings=[str(e)]
            )
        except Exception as e:
            self.logger.error(f"Error processing request: {str(e)}")
            raise e

    def _validate_constitutional_requirements(self, agent_input: AgentInput):
        """Validate that the request meets constitutional requirements."""
        self.logger.debug("Validating constitutional requirements")

        # Validate context bundle format
        from utils.context_utils import validate_context_bundle_format
        if not validate_context_bundle_format(agent_input.context_bundle):
            raise ConstitutionalViolationError("Context bundle format is invalid")

        # For selected_text_only mode, check if context bundle status allows answering
        if agent_input.mode == "selected_text_only":
            context_status = agent_input.context_bundle.get('status', 'success')
            if context_status != 'success':
                raise SelectedTextOnlyViolationError(
                    f"Selected-text-only mode requires context bundle status 'success', got '{context_status}'"
                )

        # Log the constitutional check
        log_constitutional_check(
            check_type="Request Validation",
            passed=True,
            details={
                "mode": agent_input.mode,
                "context_status": agent_input.context_bundle.get('status', 'unknown')
            }
        )

    def _validate_response_grounding(self, response: str, context_chunks: List[Dict[str, Any]]) -> bool:
        """
        Validate that the response is properly grounded in the provided context.
        This is a basic implementation that checks if response content relates to context.
        """
        if not response or not context_chunks:
            return False

        # For now, we'll do a simple check to see if response content relates to context
        # In a more sophisticated implementation, we might use semantic similarity
        response_lower = response.lower()
        context_content = " ".join([chunk.get("content", "").lower() for chunk in context_chunks])

        # This is a simplified check - in practice, you'd want more sophisticated validation
        return len(response) > 0 and len(context_content) > 0

    def _extract_sentence_citations(self, response: str, context_chunks: List[Dict[str, Any]]) -> List[ChunkReference]:
        """
        Extract citations for each sentence in the response.
        This is a basic implementation that maps sentences to context chunks.
        """
        sentences = extract_sentences(response)
        citations = []

        for sentence in sentences:
            # Find which chunk this sentence relates to
            for chunk in context_chunks:
                chunk_content = chunk.get("content", "")
                chunk_id = chunk.get("chunk_id", "")
                source_url = chunk.get("metadata", {}).get("source_url", "")

                # If sentence content appears in chunk, create a citation
                if sentence.lower() in chunk_content.lower() and chunk_id and source_url:
                    citation = ChunkReference(chunk_id=chunk_id, source_url=source_url)
                    if citation not in citations:
                        citations.append(citation)
                        log_citation_trace(chunk_id, source_url, sentence[:50])
                    break

        return citations