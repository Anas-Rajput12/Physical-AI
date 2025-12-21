#!/usr/bin/env python3
"""
Test script to run the backend without Unicode issues
"""
import os
import sys
from pathlib import Path

# Add the backend src directory to the path
sys.path.insert(0, str(Path(__file__).parent / "src"))

def test_backend():
    """Test if the backend can be imported and run"""
    try:
        from src.api.main import app
        print("[SUCCESS] Backend module imported successfully")

        # Check if we can access the settings
        from src.config.settings import settings
        print(f"[SUCCESS] Settings loaded: {settings.app_name}")

        # Check environment variables
        required_vars = [
            'OPENROUTER_API_KEY',
            'QDRANT_API_KEY',
            'QDRANT_URL',
            'NEON_DATABASE_URL'
        ]

        missing_vars = []
        for var in required_vars:
            if not os.getenv(var):
                missing_vars.append(var)

        if missing_vars:
            print("[WARNING] Missing required environment variables:")
            for var in missing_vars:
                print(f"  - {var}")
            print("Please set these environment variables before running the backend.")
            return False
        else:
            print("[SUCCESS] All required environment variables are set")

        return True

    except ImportError as e:
        print(f"[ERROR] Import error: {e}")
        return False
    except Exception as e:
        print(f"[ERROR] Error testing backend: {e}")
        return False

if __name__ == "__main__":
    success = test_backend()
    if success:
        print("\n[SUCCESS] Backend is ready to run!")
    else:
        print("\n[ERROR] Backend has issues that need to be resolved")
        sys.exit(1)