# Quickstart Guide: Docusaurus Agent Integration

## Prerequisites
- Node.js and npm/yarn installed
- Docusaurus project already set up
- Phase 3 backend API running and accessible

## Setup

### 1. Clone and Navigate
```bash
cd Physical-AI-and-Humanoid-Robotics
```

### 2. Install Dependencies (if needed)
```bash
npm install axios  # or yarn add axios
# Note: Docusaurus and React are already available
```

### 3. Create the Interactive Page
Create the query page component:
```bash
mkdir -p src/pages
# Create src/pages/query.js with the interactive query component
```

### 4. Create Component Files
Create the reusable components:
```bash
mkdir -p src/components
# Create:
# - src/components/QueryInput.js
# - src/components/ResponseViewer.js
# - src/components/CitationsPanel.js
# - src/components/ErrorNotifier.js
```

## Running the Application

### 1. Start the Phase 3 Backend
Ensure the Phase 3 response generation backend is running:
```bash
cd backend/response_generation
uvicorn main:app --reload
```

### 2. Start Docusaurus
```bash
cd Physical-AI-and-Humanoid-Robotics
npm run start
```

### 3. Access the Interactive Page
Navigate to `http://localhost:3000/query` to access the interactive query page.

## Basic Usage

1. Enter your question about the book content in the query input field
2. Submit the query
3. View the AI-generated response with proper citations
4. Check the citations panel for source references

## API Configuration
The frontend will connect to the Phase 3 backend API at the configured endpoint (typically `http://localhost:8000/api/answer` during development).

## Building for Production
```bash
npm run build
```
The interactive page will be included in the production build and deployed with the rest of the Docusaurus site.