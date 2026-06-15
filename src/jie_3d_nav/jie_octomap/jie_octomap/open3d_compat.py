from __future__ import annotations

import importlib
import os
import sys
import types


def import_open3d():
    """Import Open3D without loading its optional ML package by default."""
    enable_ml = os.environ.get("JIE_OCTOMAP_ENABLE_OPEN3D_ML", "").lower()
    if enable_ml not in {"1", "true", "yes", "on"}:
        sys.modules.setdefault("open3d.ml", types.ModuleType("open3d.ml"))
    return importlib.import_module("open3d")
