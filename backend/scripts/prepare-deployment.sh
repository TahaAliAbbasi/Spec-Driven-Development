#!/bin/bash
# Deployment preparation script

echo "Preparing backend for deployment..."

# Check if .env file exists and has placeholder values
if [ ! -f ".env" ]; then
    echo "Creating .env from .env.example..."
    cp .env.example .env
    echo "Please update .env with your actual API keys before deployment."
else
    # Check if .env has placeholder values (not actual API keys)
    if grep -q "your_openai_api_key_here\|your_cohere_api_key_here\|your_qdrant_api_key_here" .env; then
        echo ".env file contains placeholder values. Please update with actual API keys before deployment."
    else
        echo "Warning: .env file contains values that may be real API keys."
        echo "Make sure you're not committing real API keys to the repository."
    fi
fi

echo "Deployment preparation complete."
echo "Remember to set your environment variables before running in production."