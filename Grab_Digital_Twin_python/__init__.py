import sys, os

if not (
    any(flag in sys.argv for flag in ("--headless", "--no-window"))
    or "headless_runner" in sys.argv[0].lower()
    or os.environ.get("GRAB_RUN_HEADLESS") == "1"
):
    from .extension import *
