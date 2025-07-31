import os
from typing import Any, List


def get_build_input_globs(config: Any, editable: bool) -> List[str]:
    """Get build input globs for ROS package."""
    base_globs = [
        # Source files
        "**/*.c",
        "**/*.cpp",
        "**/*.h"
        "**/*.hpp",
        "**/*.rs",
        "**/*.sh",
        # Project configuration
        "package.xml",
        "setup.py",
        "setup.cfg",
        "pyproject.toml",
        # Build configuration
        "Makefile",
        "CMakeLists.txt",
        "MANIFEST.in",
        "tests/**/*.py",
        "docs/**/*.rst",
        "docs/**/*.md",
    ]

    python_globs = [] if editable else ["**/*.py", "**/*.pyx"]

    all_globs = base_globs + python_globs
    if hasattr(config, "extra_input_globs"):
        all_globs.extend(config.extra_input_globs)

    return all_globs


def get_editable_setting(python_params: Any) -> bool:
    """Get editable setting from environment or params."""
    env_editable = os.environ.get("BUILD_EDITABLE_PYTHON", "").lower() == "true"
    if env_editable:
        return True

    if python_params and hasattr(python_params, "editable"):
        return bool(python_params.editable)

    return False
