# Deployment Guide for RAG Chatbot Backend

This document provides instructions for deploying the RAG Chatbot Backend service in various environments.

## Table of Contents

- [Docker Deployment](#docker-deployment)
- [Kubernetes Deployment](#kubernetes-deployment)
- [Environment Variables](#environment-variables)
- [Production Considerations](#production-considerations)
- [Scaling](#scaling)
- [Monitoring](#monitoring)

## Docker Deployment

### Prerequisites

- Docker
- Docker Compose

### Steps

1. **Build and run with Docker Compose:**
   ```bash
   cd backend/deployment
   docker-compose up -d --build
   ```

2. **Or build and run the Docker image directly:**
   ```bash
   # Build the image
   docker build -t rag-chatbot-backend -f deployment/Dockerfile ../

   # Run the container
   docker run -d \
     --name rag-chatbot-backend \
     -p 8000:8000 \
     -e OPENAI_API_KEY=your_openai_api_key \
     -e COHERE_API_KEY=your_cohere_api_key \
     -e QDRANT_API_KEY=your_qdrant_api_key \
     -e QDRANT_URL=your_qdrant_url \
     rag-chatbot-backend
   ```

### Environment Variables

Create a `.env` file in the deployment directory:

```env
OPENAI_API_KEY=your_openai_api_key
COHERE_API_KEY=your_cohere_api_key
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_URL=your_qdrant_url
QDRANT_COLLECTION_NAME=physical_ai_humanoid_docs_v3
FRONTEND_API_KEY=your_frontend_api_key
PORT=8000
LOG_LEVEL=INFO
MAX_CONTEXT_TOKENS=4000
RATE_LIMIT_REQUESTS=100
RATE_LIMIT_WINDOW=60
SESSION_TIMEOUT_MINUTES=30
DEFAULT_CONFIDENCE_THRESHOLD=0.7
OPENAI_MODEL=gpt-4-turbo
COHERE_MODEL=embed-multilingual-v3.0
```

## Kubernetes Deployment

### Prerequisites

- Kubernetes cluster
- kubectl configured
- cert-manager (for TLS) - optional

### Steps

1. **Create secrets for API keys:**
   ```bash
   kubectl create secret generic rag-chatbot-secrets \
     --from-literal=openai-api-key=your_openai_api_key \
     --from-literal=cohere-api-key=your_cohere_api_key \
     --from-literal=qdrant-api-key=your_qdrant_api_key \
     --from-literal=qdrant-url=your_qdrant_url \
     --from-literal=frontend-api-key=your_frontend_api_key
   ```

2. **Deploy to Kubernetes:**
   ```bash
   kubectl apply -f backend/deployment/k8s-deployment.yaml
   ```

3. **Check deployment status:**
   ```bash
   kubectl get pods
   kubectl get services
   kubectl get ingress
   ```

## Environment Variables

All environment variables can be configured as needed for your deployment environment:

| Variable | Default | Description |
|----------|---------|-------------|
| `OPENAI_API_KEY` | - | Your OpenAI API key |
| `COHERE_API_KEY` | - | Your Cohere API key |
| `QDRANT_API_KEY` | - | Your Qdrant API key |
| `QDRANT_URL` | - | Your Qdrant URL |
| `QDRANT_COLLECTION_NAME` | `physical_ai_humanoid_docs_v3` | Name of the Qdrant collection |
| `FRONTEND_API_KEY` | - | API key for frontend authentication |
| `PORT` | `8000` | Port to run the service on |
| `LOG_LEVEL` | `INFO` | Logging level (DEBUG, INFO, WARNING, ERROR) |
| `MAX_CONTEXT_TOKENS` | `4000` | Maximum tokens for context |
| `RATE_LIMIT_REQUESTS` | `100` | Rate limit requests per window |
| `RATE_LIMIT_WINDOW` | `60` | Rate limit window in seconds |
| `SESSION_TIMEOUT_MINUTES` | `30` | Session timeout in minutes |
| `DEFAULT_CONFIDENCE_THRESHOLD` | `0.7` | Minimum confidence threshold |

## Production Considerations

### Security

- Store API keys in Kubernetes secrets or Docker secrets, not in environment files
- Use HTTPS with TLS certificates
- Implement proper network policies
- Regular security scanning of images

### Performance

- Configure appropriate resource limits and requests
- Use horizontal pod autoscaling based on CPU/memory
- Implement proper caching strategies
- Monitor response times and error rates

### Monitoring

- Set up health checks
- Monitor application logs
- Track performance metrics
- Set up alerting for critical issues

## Scaling

### Horizontal Scaling

The application is designed to be horizontally scalable:

- Multiple instances can run simultaneously
- Session data is managed per-instance but can be externalized if needed
- Load balancer distributes requests across instances

### Resource Configuration

Adjust resource limits in the Kubernetes deployment based on your needs:

```yaml
resources:
  requests:
    memory: "512Mi"
    cpu: "500m"
  limits:
    memory: "1Gi"
    cpu: "1000m"
```

## Monitoring

### Health Checks

The application provides a health check endpoint at `/api/health` which returns:

```json
{
  "status": "healthy",
  "service": "rag-chatbot-backend",
  "timestamp": "2023-12-24T10:00:00.000Z"
}
```

### Metrics

The application includes built-in performance monitoring. You can access metrics through:

- Health endpoint
- Application logs
- Custom metrics endpoints (if configured)

## Troubleshooting

### Common Issues

#### Container Won't Start
- Check that all required environment variables are set
- Verify API keys are valid
- Ensure Qdrant is accessible from the container

#### High Resource Usage
- Monitor container resource usage
- Adjust resource limits as needed
- Check for memory leaks or performance bottlenecks

#### Scaling Issues
- Verify that the application is stateless
- Check session management
- Monitor load distribution

### Debugging

Enable more verbose logging by setting LOG_LEVEL to DEBUG:

```bash
docker run -e LOG_LEVEL=DEBUG ...
```

## Rollback Strategy

### Docker Compose

To rollback to a previous version:

```bash
docker-compose down
# Revert to previous image tag
docker-compose up -d
```

### Kubernetes

To rollback to a previous deployment:

```bash
kubectl rollout undo deployment/rag-chatbot-backend
```

## Backup and Recovery

### Configuration
- Version control all deployment configurations
- Regularly backup secrets and configurations

### Data
- The application is stateless, no persistent data to backup
- Session data is temporary and expires
- Knowledge base is stored in Qdrant (backup separately)