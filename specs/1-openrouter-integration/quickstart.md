# Quickstart Guide: OpenRouter Integration with uv Package Manager

## Prerequisites

- Python 3.8 or higher
- uv package manager installed

## Installation

### 1. Install uv Package Manager

```bash
# Install uv using the official installer
curl -LsSf https://astral.sh/uv/install.sh | sh

# Or install via pip (if curl is not available)
pip install uv
```

### 2. Clone the Repository

```bash
git clone <repository-url>
cd <repository-directory>
```

### 3. Navigate to the Backend Directory

```bash
cd backend/response_generation
```

### 4. Install Dependencies with uv

```bash
# Create virtual environment and install dependencies
uv sync

# Or if you want to install in development mode
uv sync --dev
```

### 5. Set Up Environment Variables

Create a `.env` file in the `backend/response_generation` directory:

```env
# OpenRouter API Configuration
OPENROUTER_API_KEY=your_openrouter_api_key_here
OPENROUTER_BASE_URL=https://openrouter.ai/api/v1
OPENROUTER_MODEL=gpt-4.1-mini

# Application Configuration
APP_ENV=development
LOG_LEVEL=info
HOST=localhost
PORT=8000

# Model Configuration
MODEL_TEMPERATURE=0
MAX_TOKENS=2048
```

## Running the Application

### 1. Activate the Virtual Environment

```bash
# uv automatically activates the virtual environment for commands
uv run python main.py
```

### 2. Or Run with uv Run Command

```bash
uv run uvicorn main:app --reload --host localhost --port 8000
```

## Managing Dependencies

### Adding New Dependencies

```bash
# Add a new dependency
uv add package-name

# Add a specific version
uv add package-name==1.2.3

# Add a development dependency
uv add --dev pytest
```

### Removing Dependencies

```bash
# Remove a dependency
uv remove package-name
```

### Syncing Dependencies

```bash
# Sync dependencies from uv.lock
uv sync

# Force reinstall all packages
uv sync --refresh
```

## Development Commands

### Running Tests

```bash
uv run pytest
```

### Running the Debug Test

```bash
uv run python debug_test.py
```

### Linting (if applicable)

```bash
uv run ruff check .
uv run black .
```

## Deployment

### Building for Production

```bash
# Create a production lock file
uv lock --locked

# Sync production dependencies only
uv sync --locked
```

### Docker Integration

If using Docker, you can use uv in your Dockerfile:

```dockerfile
# Install uv
RUN curl -LsSf https://astral.sh/uv/install.sh | sh
ENV PATH="/root/.cargo/bin:$PATH"

# Copy requirements and lock file
COPY uv.lock pyproject.toml ./

# Install dependencies
RUN uv sync --frozen

# Run the application
CMD ["uv", "run", "python", "main.py"]
```

## Troubleshooting

### Common Issues

1. **uv not found**: Make sure uv is installed and in your PATH
2. **Permission errors**: Run commands with appropriate permissions or use virtual environment
3. **Dependency conflicts**: Run `uv lock --upgrade` to resolve conflicts

### Useful Commands

```bash
# Check uv version
uv --version

# Show dependency tree
uv tree

# Run shell in virtual environment
uv shell
```

## Performance Benefits

Using uv provides several performance improvements over traditional pip:

- Up to 10x faster dependency resolution
- Faster virtual environment creation
- Better parallelization of operations
- More efficient caching