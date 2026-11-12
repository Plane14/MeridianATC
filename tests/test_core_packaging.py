from __future__ import annotations

import ast
import re
import tomllib
from pathlib import Path

from meridianatc import __version__


def test_python_package_and_frozen_metadata_have_one_version():
    root = Path(__file__).resolve().parents[1]
    project = tomllib.loads((root / "pyproject.toml").read_text(encoding="utf-8"))
    assert project["project"]["version"] == __version__
    source = ast.parse((root / "meridianatc.spec").read_text(encoding="utf-8"))
    reader = next(node for node in source.body if isinstance(node, ast.FunctionDef)
                  and node.name == "_read_metadata")
    namespace = {"ROOT": root, "re": re}
    exec(compile(ast.Module(body=[reader], type_ignores=[]), "metadata", "exec"), namespace)
    assert namespace["_read_metadata"]()["__version__"] == __version__
    installer = (root / "installer" / "MeridianATC.iss").read_text(encoding="utf-8")
    assert "AppVersion={#AppVersion}" in installer
    assert "VersionInfoVersion={#NumericVersion}" in installer
