#!/usr/bin/env python3
"""Compatibility command; implementation lives in tools.audit.audit_cartesian_edge_rejections."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tools.audit.audit_cartesian_edge_rejections import *
from tools.audit.audit_cartesian_edge_rejections import main

if __name__ == "__main__":
    main()
