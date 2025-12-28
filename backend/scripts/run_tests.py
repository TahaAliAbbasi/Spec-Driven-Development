#!/usr/bin/env python3
"""
Test runner for backend services
This script ensures tests run with the correct Python path
"""
import sys
import os
import subprocess
from pathlib import Path

def run_tests():
    """Run backend tests with proper Python path configuration"""
    # Add the backend directory to Python path
    backend_dir = Path(__file__).parent.absolute()
    sys.path.insert(0, str(backend_dir))

    # Set the PYTHONPATH environment variable for subprocess calls
    env = os.environ.copy()
    env['PYTHONPATH'] = str(backend_dir) + os.pathsep + env.get('PYTHONPATH', '')

    # Run pytest with the correct configuration
    try:
        result = subprocess.run([
            sys.executable, '-m', 'pytest', 'tests/', '-v'
        ], env=env, cwd=backend_dir, check=True)

        print("All tests passed successfully!")
        return True
    except subprocess.CalledProcessError as e:
        print(f"Tests failed with exit code: {e.returncode}")
        return False

if __name__ == "__main__":
    success = run_tests()
    sys.exit(0 if success else 1)