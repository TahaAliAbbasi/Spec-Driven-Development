# Research: Backend Build Readiness

## Decision: Fix Dockerfile for proper dependency management
**Rationale**: The original Dockerfile was trying to copy files that didn't exist in the expected location, causing build failures. The fix ensures proper dependency installation using uv.lock for reproducible builds.

**Alternatives considered**:
- Using requirements.txt instead of uv.lock (rejected because uv.lock provides better dependency resolution)
- Installing dependencies directly without lock file (rejected due to security/reproducibility concerns)

## Decision: Implement environment variable-based CORS configuration
**Rationale**: The original CORS configuration used wildcard origins which is a security risk in production. The new approach uses environment variables to specify allowed origins.

**Alternatives considered**:
- Hardcoded origins (rejected due to inflexibility across environments)
- No CORS configuration (rejected due to security requirements)

## Decision: Create proper .env.example and deployment scripts
**Rationale**: To ensure secure deployment practices and prevent accidental exposure of API keys, proper documentation and scripts are needed.

**Alternatives considered**:
- Leaving .env in repository (rejected due to security concerns)
- No deployment guidance (rejected due to operational requirements)

## Decision: Update .gitignore to track uv.lock while ignoring .env
**Rationale**: uv.lock ensures reproducible builds across environments while .env files contain sensitive information that should not be committed.

**Alternatives considered**:
- Using requirements.txt instead (rejected as uv.lock is more precise)
- Tracking .env files (rejected due to security concerns)