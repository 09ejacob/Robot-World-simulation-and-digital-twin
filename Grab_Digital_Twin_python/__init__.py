# Copyright (c) 2022-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import sys, os

if not (any(flag in sys.argv for flag in ("--headless", "--no-window"))
        or "headless_runner" in sys.argv[0].lower()
        or os.environ.get("GRAB_RUN_HEADLESS") == "1"):
    from .extension import *


