#!/usr/bin/env python3
"""
Autonomous Navigation Node Executable Wrapper
"""
import sys
import os
from pathlib import Path

# Add the script directory to Python path
script_dir = Path(__file__).parent.parent / 'share' / 'labrobot' / 'scripts'
sys.path.insert(0, str(script_dir))

# Import and run the main script
if __name__ == '__main__':
    import importlib.util
    spec = importlib.util.spec_from_file_location("autonomous_navigation_node", script_dir / "autonomous_navigation_node.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
