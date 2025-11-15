#!/usr/bin/env python3
"""
Script to run run_trained_model.py with predefined arguments.
"""

import subprocess
import sys

if __name__ == "__main__":
    args = [
        sys.executable,
        "run_trained_model.py",
        "--gui",
        "--episodes", "10",
        "--max-steps", "1000",
        "--model", "q-tabular.pkl"
    ]
    
    subprocess.run(args)

