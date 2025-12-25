"""Debug script to test the OpenRouter provider adapter initialization"""


from config import settings
from agents.openrouter_provider_adapter import OpenRouterProviderAdapter

print(f"OPENROUTER_API_KEY from settings: '{settings.OPENROUTER_API_KEY}'")
print(f"Boolean evaluation of OPENROUTER_API_KEY: {bool(settings.OPENROUTER_API_KEY)}")
print(f"Is OPENROUTER_API_KEY empty string: {settings.OPENROUTER_API_KEY == ''}")
print(f"OPENROUTER_MODEL: {settings.OPENROUTER_MODEL}")
print(f"OPENROUTER_BASE_URL: {settings.OPENROUTER_BASE_URL}")

try:
    adapter = OpenRouterProviderAdapter()
    print("OpenRouter provider adapter initialized successfully")
except Exception as e:
    print(f"Error initializing OpenRouter provider adapter: {e}")