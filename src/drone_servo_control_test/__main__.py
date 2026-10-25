"""Allow ``python -m drone_servo_control_test``."""

import sys

from .main import cli

sys.exit(cli())
