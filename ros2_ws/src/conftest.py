"""Make the pure-Python package cores importable for pytest without a colcon build."""

import sys
from pathlib import Path

for pkg in sorted(Path(__file__).parent.glob('tank01_*')):
    if (pkg / 'setup.py').exists() and str(pkg) not in sys.path:
        sys.path.insert(0, str(pkg))
