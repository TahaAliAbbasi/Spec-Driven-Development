"""
API Response Caching Service
Implements caching for API responses to improve performance
"""
import time
import hashlib
import json
from typing import Any, Optional, Dict
from datetime import datetime, timedelta
from threading import Lock
import logging

# Configure logging
logger = logging.getLogger(__name__)


class CacheService:
    """
    Service for caching API responses to improve performance
    """

    def __init__(self, default_ttl: int = 300):  # 5 minutes default TTL
        """
        Initialize the cache service

        Args:
            default_ttl: Default time-to-live in seconds for cached items
        """
        self.cache: Dict[str, Dict[str, Any]] = {}
        self.default_ttl = default_ttl
        self.lock = Lock()  # Thread-safe operations
        logger.info(f"CacheService initialized with default TTL: {default_ttl}s")

    def _generate_cache_key(self, endpoint: str, params: Dict[str, Any]) -> str:
        """
        Generate a unique cache key based on endpoint and parameters

        Args:
            endpoint: API endpoint
            params: Request parameters

        Returns:
            Generated cache key
        """
        # Create a hash of the endpoint and sorted parameters to ensure consistent keys
        sorted_params = json.dumps(params, sort_keys=True, default=str)
        key_string = f"{endpoint}:{sorted_params}"
        return hashlib.md5(key_string.encode()).hexdigest()

    def get(self, cache_key: str) -> Optional[Any]:
        """
        Get a value from the cache

        Args:
            cache_key: Key to retrieve value for

        Returns:
            Cached value if exists and not expired, None otherwise
        """
        with self.lock:
            if cache_key in self.cache:
                cached_item = self.cache[cache_key]
                current_time = time.time()

                # Check if the item has expired
                if current_time < cached_item['expires_at']:
                    logger.debug(f"Cache hit for key: {cache_key}")
                    return cached_item['value']
                else:
                    # Remove expired item
                    del self.cache[cache_key]
                    logger.debug(f"Cache miss (expired) for key: {cache_key}")
                    return None
            else:
                logger.debug(f"Cache miss for key: {cache_key}")
                return None

    def set(self, cache_key: str, value: Any, ttl: Optional[int] = None) -> None:
        """
        Set a value in the cache

        Args:
            cache_key: Key to store value under
            value: Value to cache
            ttl: Time-to-live in seconds (uses default if not provided)
        """
        if ttl is None:
            ttl = self.default_ttl

        with self.lock:
            expires_at = time.time() + ttl
            self.cache[cache_key] = {
                'value': value,
                'expires_at': expires_at,
                'created_at': time.time()
            }
            logger.debug(f"Cache set for key: {cache_key} with TTL: {ttl}s")

    def invalidate(self, cache_key: str) -> bool:
        """
        Remove a specific key from the cache

        Args:
            cache_key: Key to remove from cache

        Returns:
            True if key was found and removed, False otherwise
        """
        with self.lock:
            if cache_key in self.cache:
                del self.cache[cache_key]
                logger.debug(f"Cache invalidated for key: {cache_key}")
                return True
            return False

    def invalidate_by_prefix(self, prefix: str) -> int:
        """
        Remove all keys that start with the given prefix

        Args:
            prefix: Prefix to match keys against

        Returns:
            Number of keys removed
        """
        with self.lock:
            keys_to_remove = [key for key in self.cache.keys() if key.startswith(prefix)]
            for key in keys_to_remove:
                del self.cache[key]
            logger.debug(f"Cache invalidated {len(keys_to_remove)} keys with prefix: {prefix}")
            return len(keys_to_remove)

    def clear(self) -> None:
        """
        Clear all items from the cache
        """
        with self.lock:
            count = len(self.cache)
            self.cache.clear()
            logger.debug(f"Cleared {count} items from cache")

    def get_cache_stats(self) -> Dict[str, Any]:
        """
        Get statistics about the cache

        Returns:
            Dictionary with cache statistics
        """
        with self.lock:
            current_time = time.time()
            active_items = 0
            expired_items = 0

            for item in self.cache.values():
                if current_time < item['expires_at']:
                    active_items += 1
                else:
                    expired_items += 1

            return {
                'total_items': len(self.cache),
                'active_items': active_items,
                'expired_items': expired_items,
                'default_ttl': self.default_ttl
            }


# Global cache service instance
cache_service = CacheService()