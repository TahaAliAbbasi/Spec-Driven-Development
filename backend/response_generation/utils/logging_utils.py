"""Logging and monitoring utilities for agent operations"""


import logging
import sys
from typing import Any, Dict
from config import settings


def setup_logging():
    """Set up logging configuration based on settings"""
    log_level = getattr(logging, settings.LOG_LEVEL.upper(), logging.INFO)

    # Create a custom format that includes more details
    log_format = (
        "%(asctime)s [%(levelname)s] %(name)s:%(lineno)d - %(message)s"
    )

    # Configure root logger
    logging.basicConfig(
        level=log_level,
        format=log_format,
        handlers=[
            logging.StreamHandler(sys.stdout),
            # In production, you might also want to add a file handler
        ]
    )


def get_logger(name: str) -> logging.Logger:
    """Get a configured logger instance"""
    logger = logging.getLogger(name)
    return logger


def log_agent_operation(
    operation: str,
    query: str,
    context_size: int,
    response_status: str,
    execution_time: float = None
):
    """Log agent operations for monitoring and debugging"""
    logger = get_logger("agent_operations")
    logger.info(
        f"Agent operation: {operation}, "
        f"Query length: {len(query)}, "
        f"Context size: {context_size} chunks, "
        f"Response status: {response_status}"
        + (f", Execution time: {execution_time:.2f}s" if execution_time else "")
    )


def log_constitutional_check(
    check_type: str,
    passed: bool,
    details: Dict[str, Any] = None
):
    """Log constitutional compliance checks"""
    logger = get_logger("constitutional_compliance")
    status = "PASSED" if passed else "FAILED"
    logger.info(f"Constitutional check '{check_type}' {status}")

    if details and not passed:
        logger.warning(f"Check details: {details}")


def log_citation_trace(
    chunk_id: str,
    source_url: str,
    content_preview: str = None
):
    """Log citation creation for traceability"""
    logger = get_logger("citations")
    logger.info(
        f"Citation created: chunk_id={chunk_id}, source_url={source_url}"
        + (f", content_preview='{content_preview[:50]}...'" if content_preview else "")
    )