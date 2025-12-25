# Data Model: OpenRouter Integration

## Overview
This document defines the data models for the OpenRouter integration in the Agent-Based Response Generation & Orchestration system. The models maintain compatibility with existing functionality while enabling OpenRouter API integration.

## Entity Models

### 1. OpenRouter Configuration
**Purpose**: Stores OpenRouter API configuration parameters

```python
class OpenRouterConfig:
    api_key: str                    # OPENROUTER_API_KEY - Secret API key for authentication
    base_url: str                   # OPENROUTER_BASE_URL - API endpoint (default: https://openrouter.ai/api/v1)
    model: str                      # OPENROUTER_MODEL - Model identifier (e.g., gpt-4.1-mini)
    temperature: float              # MODEL_TEMPERATURE - Response determinism (default: 0)
    max_tokens: int                 # MAX_TOKENS - Maximum response length (default: 2048)
```

**Validation Rules**:
- api_key must not be empty
- base_url must be a valid URL
- model must be a valid OpenRouter model identifier
- temperature must be 0 for constitutional compliance
- max_tokens must be positive

### 2. OpenRouter API Request
**Purpose**: Defines the structure for OpenRouter API requests

```python
class OpenRouterRequest:
    model: str                      # Model identifier to use
    messages: List[Message]         # Conversation messages (system, user, assistant)
    temperature: float              # Response randomness (0 for deterministic)
    max_tokens: int                 # Maximum tokens in response
    timeout: int                    # Request timeout in seconds
```

**Validation Rules**:
- model must match configured model
- messages must contain at least one user message
- temperature must be 0 for constitutional compliance
- max_tokens must not exceed configured limit

### 3. Message
**Purpose**: Individual message in the conversation

```python
class Message:
    role: str                       # Message role: "system", "user", or "assistant"
    content: str                    # Message content
```

**Validation Rules**:
- role must be one of allowed values
- content must not be empty

### 4. OpenRouter API Response
**Purpose**: Structure of OpenRouter API responses

```python
class OpenRouterResponse:
    id: str                         # Unique response identifier
    choices: List[Choice]           # List of response choices
    usage: Usage                    # Token usage statistics
    model: str                      # Model that generated the response
```

### 5. Choice
**Purpose**: Individual choice in OpenRouter response

```python
class Choice:
    index: int                      # Choice index
    message: Message                # The generated message
    finish_reason: str              # Reason for finishing (e.g., "stop", "length")
```

### 6. Usage
**Purpose**: Token usage statistics

```python
class Usage:
    prompt_tokens: int              # Number of tokens in the prompt
    completion_tokens: int          # Number of tokens in the completion
    total_tokens: int               # Total number of tokens used
```

## Relationship Diagram

```
OpenRouterConfig
       |
       | (used by)
       |
OpenRouterProviderAdapter
       |
       | (creates)
       |
OpenRouterRequest ---------> Message (1..*)
       |                          |
       | (processed by)           | (contains)
       |                          |
       +----------------> OpenRouterResponse
                                  |
                                  | (contains)
                                  |
                            Choice (1..*) -----> Usage (1)
```

## State Transitions

### OpenRouterProviderAdapter States:
1. **Initialization**: Adapter is created with configuration
   - Input: OpenRouterConfig
   - Validation: API key exists, configuration is valid
   - Output: Ready state

2. **Request Processing**: Processing a generate_response call
   - Input: Query, context_chunks, system_prompt
   - Processing: Construct OpenRouterRequest, call API
   - Output: OpenRouterResponse or Error

3. **Response Processing**: Processing API response
   - Input: OpenRouterResponse
   - Processing: Extract text, track citations, validate grounding
   - Output: Response dictionary with text and usage

4. **Error Handling**: When API call fails
   - Input: Exception
   - Processing: Log error, raise APIConnectionError
   - Output: Error state or fallback response

## API Contracts

### OpenRouterProviderAdapter Interface
```python
def generate_response(
    query: str,
    context_chunks: List[Dict[str, Any]],
    system_prompt: str = None
) -> Dict[str, Any]:
    """
    Generate a response using the OpenRouter API based on query and context.

    Args:
        query: The user's question/query
        context_chunks: List of context chunks to ground the response
        system_prompt: Optional system prompt to guide the response

    Returns:
        Dictionary containing the generated response and metadata:
        {
            "text": str,           # Generated response text
            "usage": Usage         # Token usage statistics
        }

    Raises:
        APIConnectionError: If OpenRouter API is unavailable
        ModelResponseError: If API returns invalid response
    """
```

### Constitutional Compliance Validation
```python
def validate_no_retrieval_capability() -> bool:
    """
    Verify that this adapter enforces the 'NO RETRIEVAL' hard guard.

    Returns:
        bool: Always True, confirming no external retrieval capabilities
    """
```

### Citation Extraction
```python
def extract_citations_from_response(
    response_text: str,
    context_chunks: List[Dict[str, Any]]
) -> List[Dict[str, str]]:
    """
    Extract citations from the response by matching content to context chunks.

    Args:
        response_text: Generated response text
        context_chunks: Original context chunks used for generation

    Returns:
        List of citation dictionaries with chunk_id and source_url
    """
```

## Validation Rules Summary

1. **Constitutional Compliance**: All responses must be grounded in provided context
2. **Determinism**: Temperature must be 0 for all API calls
3. **API Key Security**: API key must be stored securely and not exposed in logs
4. **Context Integrity**: Context chunks must not be modified during processing
5. **Response Validation**: Generated responses must relate to provided context
6. **Token Limits**: API calls must respect configured token limits