"""ContextBundle compatibility layer to handle Phase 2 output format"""


def validate_context_bundle_format(context_bundle: dict) -> bool:
    """
    Validate that the context_bundle follows the expected format from Phase 2.
    """
    if not isinstance(context_bundle, dict):
        return False

    # Check for required top-level keys
    required_keys = ['chunks']
    for key in required_keys:
        if key not in context_bundle:
            return False

    # Validate chunks structure
    chunks = context_bundle.get('chunks', [])
    if not isinstance(chunks, list):
        return False

    for chunk in chunks:
        if not isinstance(chunk, dict):
            return False

        # Each chunk should have required fields
        if 'chunk_id' not in chunk or 'content' not in chunk:
            return False

    return True


def normalize_context_bundle(context_bundle: dict) -> dict:
    """
    Normalize the context_bundle to ensure consistent format.
    """
    if not validate_context_bundle_format(context_bundle):
        raise ValueError("Invalid context_bundle format")

    normalized = context_bundle.copy()

    # Ensure chunks have consistent structure
    chunks = []
    for chunk in normalized.get('chunks', []):
        normalized_chunk = {
            'chunk_id': chunk.get('chunk_id', ''),
            'content': chunk.get('content', ''),
            'metadata': chunk.get('metadata', {})
        }
        chunks.append(normalized_chunk)

    normalized['chunks'] = chunks
    return normalized


def filter_chunks_by_mode(context_bundle: dict, mode: str) -> dict:
    """
    Filter context_bundle chunks based on the requested mode.
    For selected_text_only mode, this would apply additional filtering,
    but the actual validation happens at the constitutional level.
    """
    if mode == "selected_text_only":
        # In selected_text_only mode, we may need to apply additional filtering
        # based on what text was selected by the user
        status = context_bundle.get('status', 'success')
        if status != 'success':
            # This would be caught by constitutional validation later
            pass

    return normalize_context_bundle(context_bundle)


def get_chunk_by_id(context_bundle: dict, chunk_id: str) -> dict:
    """
    Retrieve a specific chunk by its ID from the context bundle.
    """
    chunks = context_bundle.get('chunks', [])
    for chunk in chunks:
        if chunk.get('chunk_id') == chunk_id:
            return chunk

    raise ValueError(f"Chunk with ID '{chunk_id}' not found in context bundle")