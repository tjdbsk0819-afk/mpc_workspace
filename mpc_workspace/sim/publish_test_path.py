#!/usr/bin/env python3
"""Compatibility entry point for the packaged test-path publisher."""
from pathlib import Path
import runpy

if __name__ == '__main__':
    runpy.run_path(str(Path(__file__).resolve().parents[1] /
                      'src/antbot_mpc_sim/scripts/publish_test_path.py'), run_name='__main__')
