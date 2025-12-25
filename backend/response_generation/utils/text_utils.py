"""Utility functions for text processing, token counting, and context validation"""


def count_tokens(text: str) -> int:
    """
    Count tokens in text using a simple heuristic.
    This implementation uses the common approximation of 1 token ≈ 4 characters.
    For more accuracy, we'll use the 1.33 factor from Phase 1 (words * 1.33).
    """
    if not text:
        return 0

    # Count words and apply the Phase 1 heuristic (words * 1.33)
    words = len(text.split())
    return int(words * 1.33)


def validate_context_sufficiency(context_bundle: dict, query: str) -> bool:
    """
    Validate if the context bundle is sufficient to answer the query.
    This is a basic implementation that checks if there's any content in the context.
    """
    if not context_bundle or 'chunks' not in context_bundle:
        return False

    chunks = context_bundle.get('chunks', [])
    if not chunks:
        return False

    # Check if any chunk has meaningful content
    for chunk in chunks:
        content = chunk.get('content', '').strip()
        if content:
            return True

    return False


def extract_sentences(text: str) -> list[str]:
    """
    Extract sentences from text using basic punctuation rules.
    """
    import re

    # Split text on sentence-ending punctuation
    sentences = re.split(r'[.!?]+', text)
    # Remove empty strings and strip whitespace
    sentences = [s.strip() for s in sentences if s.strip()]

    return sentences


def find_chunk_for_content(content: str, context_bundle: dict) -> str:
    """
    Find the chunk_id that contains the given content.
    Returns the first matching chunk_id or empty string if not found.
    """
    chunks = context_bundle.get('chunks', [])

    for chunk in chunks:
        chunk_content = chunk.get('content', '')
        chunk_id = chunk.get('chunk_id', '')

        # Check if the content exists in this chunk (case insensitive)
        if content.lower() in chunk_content.lower():
            return chunk_id

    return ""