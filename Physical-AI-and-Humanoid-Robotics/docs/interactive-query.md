# Interactive Query System

## Overview
The Interactive Query System allows users to ask questions about the Physical AI and Humanoid Robotics book content and receive AI-generated responses with proper citations. The system maintains constitutional compliance with zero hallucination policy while providing a seamless user experience.

## Features
- **Query Input**: Users can submit questions about the book content
- **AI Responses**: Answers generated based on book context with zero hallucination
- **Citation Tracking**: All responses include proper citations showing source material
- **Error Handling**: Clear feedback for various error conditions
- **Rate Limiting**: Protection against API abuse

## How to Use
1. Navigate to the `/query` page
2. Enter your question in the text area
3. Click "Submit Query"
4. View the AI-generated response with citations
5. Click on citation links to view source material

## Technical Details
- **Frontend**: React components integrated with Docusaurus
- **Backend**: Phase 3 response generation system
- **API**: `/api/answer` endpoint for query processing
- **Citations**: Source URLs and chunk IDs for transparency

## Response Types
- **ANSWERED**: Valid response with citations
- **INSUFFICIENT_CONTEXT**: Query cannot be answered with available content
- **REFUSED**: Query refused based on constitutional compliance rules

## Security
- Input validation and sanitization
- XSS prevention
- Rate limiting to prevent API abuse
- No API keys exposed on frontend

## Accessibility
- Proper ARIA labels
- Keyboard navigation support
- Screen reader friendly
- High contrast for readability