#!/usr/bin/env python3
"""Wrapper script to launch the dock UI from the correct location."""

import sys
import os

# Get the directory where this script is located
script_dir = os.path.dirname(os.path.abspath(__file__))

# Add the scripts directory to the Python path
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

# Import and run the main UI entry point exposed by the package
from dock_sim_ui import main

if __name__ == "__main__":
    main()
