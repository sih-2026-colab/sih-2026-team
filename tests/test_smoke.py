"""Smoke tests: verify the project skeleton imports correctly."""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))


def test_python_packages_importable():
    from python import perception, prediction, evaluation, utilities  # noqa: F401


def test_core_dependencies_available():
    import numpy, cv2, scipy, pandas, sklearn, torch  # noqa: F401


def test_project_structure():
    for rel in ["simulink/perception.slx", "simulink/planner.slx",
                "simulink/main_integration.slx", "requirements.txt", "README.md"]:
        assert (ROOT / rel).exists(), f"missing: {rel}"


def test_subdirectories_exist():
    for rel in ["data", "python", "matlab", "roadrunner", "results", "tests", "docs",
                "matlab/planning", "matlab/control", "roadrunner/village_road",
                "roadrunner/urban_intersection"]:
        assert (ROOT / rel).is_dir(), f"missing dir: {rel}"
