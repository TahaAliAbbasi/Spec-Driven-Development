# Agent Context: OpenRouter Integration with uv Package Manager and Docusaurus Urdu Translation

## New Technologies Added

### Package Management
- **uv**: Fast Python package installer and resolver written in Rust, providing up to 10x faster dependency resolution than pip
- **pyproject.toml**: Modern Python project configuration with uv.lock for deterministic builds
- **Virtual Environment Management**: Built-in virtual environment creation and management with uv

### OpenRouter Integration
- **OpenRouter API**: Cloud-based API for accessing OpenAI-compatible GPT models (gpt-4.1-mini, gpt-4o, etc.)
- **OpenAI SDK Compatibility**: Using OpenAI SDK to interface with OpenRouter's API endpoints
- **Constitutional Compliance**: Zero hallucination, deterministic outputs (temperature=0), context-only responses

### API & Web Framework
- **FastAPI**: Modern Python web framework for building APIs with automatic documentation
- **RESTful Endpoints**: Well-defined API contracts for response generation services
- **Async Processing**: Asynchronous request handling for better performance

### Data Processing
- **Context Grounding**: Responses strictly based on provided ContextBundle content
- **Citation Tracking**: Automatic tracking of which chunks contributed to the response
- **Constitutional Validation**: Verification that responses comply with zero hallucination policy

## Docusaurus Urdu Translation Specifics

### Docusaurus Framework
- **Version**: Docusaurus v3.x with future v4 compatibility flag
- **Configuration**: docusaurus.config.ts with i18n support
- **Content Structure**: docs/ directory with modular organization
- **Sidebar Configuration**: sidebars.ts for navigation structure
- **Custom Styling**: src/css/custom.css for custom CSS

### i18n Implementation
- **Native Support**: Docusaurus built-in internationalization system
- **Locale Code**: 'ur' for Urdu language
- **Language Switcher**: localeDropdown navbar item type
- **RTL Support**: Right-to-left text rendering for Urdu
- **Content Directory**: i18n/{locale}/docusaurus-plugin-content-docs/current/

### Urdu Translation Requirements
- **Language Direction**: Right-to-left (RTL) text rendering
- **Font Support**: Proper Urdu script font rendering
- **Content Structure**: Mirror English content structure in Urdu
- **HTML Attributes**: Direction and language attributes for accessibility
- **CSS Modifications**: RTL-specific styling for proper layout

## Integration Points

### With Phase 2 (Retrieval & Context Assembly)
- Compatible ContextBundle format for input to response generation
- RetrievedChunk objects with proper attribution information (chunk_id, source_url)
- Deterministic processing for consistent results

### With Phase 4 (Frontend Integration)
- API contract designed for seamless frontend integration
- Structured response format with citations for attribution
- Error handling compatible with frontend expectations

## Key Constraints

### Zero Hallucination Policy
- Response generation agents ONLY use provided context for answers
- No external knowledge retrieval or inference during response generation
- Strict enforcement of context-only answering behavior

### Constitutional Compliance
- Deterministic outputs with temperature=0 for reproducible results
- Fixed system prompts to maintain consistent behavior
- Proper refusal when context is insufficient

### Performance Requirements
- Response time under 5 seconds for 95th percentile
- Support for configurable token limits (max 2048 tokens per response)
- Efficient API call handling with proper timeout and retry logic

### Security & Compliance
- Secure API key handling without exposure in logs
- Proper validation of all input parameters
- No modification of original context content during processing

### Docusaurus Implementation Constraints
- Maintain content structure consistency between languages
- Ensure proper RTL styling for all components
- Test accessibility with screen readers
- Consider font support for Urdu script
- Preserve existing functionality while adding translation