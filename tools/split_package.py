"""Split one god-object module into a package along its banner seams.

Reads X.py, finds the named class, splits its body at section banners --
both ``# ---- / # name / # ----`` boxes and inline ``# -- name ----``
comments -- into mixin modules ``X/x_<name>.py``, puts every other
module-level statement into ``X/_shared.py``, and writes ``X/core.py``
with the class shell assembled from the mixins plus ``X/__init__.py``
re-exporting everything the module exported. Relative imports gain one
dot for the package level. ``ruff --fix`` afterward trims each mixin's
import list to what it actually uses.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

INLINE = re.compile(r"^    # -- (.*?) -{3,}\s*$")
DASH = re.compile(r"^    # -{10,}\s*$")
TITLE = re.compile(r"^    #\s+(.+?)\s*$")
IDENT = re.compile(r"[^0-9a-zA-Z_]+")
RELFROM = re.compile(r"^(\s*from\s+)(\.+)")


def _mixin_name(banner: str) -> str:
    words = [w for w in IDENT.split(banner.strip().lower()) if w]
    name = "_".join(words[:3]) or "section"
    return f"_{name[0].upper()}{name[1:]}"


def _bound_name(line: str) -> str | None:
    """The name a module-level statement's first line binds."""
    m = re.match(r"(?:class|def|async def)\s+([A-Za-z_][0-9A-Za-z_]*)", line)
    if m:
        return m.group(1)
    m = re.match(r"([A-Za-z_][0-9A-Za-z_]*)\s*(?::[^=]+)?=[^=]", line)
    return m.group(1) if m else None


FROMIMP = re.compile(r"^from\s+\S+\s+import\s+(.*)")


def _import_names(lines: list[str]) -> list[str]:
    """Names bound by import statements, multi-line parens included."""
    names: list[str] = []
    i = 0
    while i < len(lines):
        m = FROMIMP.match(lines[i])
        if m and "__future__" not in lines[i]:
            text = m.group(1)
            while "(" in text and ")" not in text and i + 1 < len(lines):
                i += 1
                text += lines[i]
            names += [p.strip().split(" as ")[-1].strip()
                      for p in text.replace("(", "").replace(")", "")
                      .replace("\\", "").split(",")
                      if p.strip() and p.strip() != "*"]
        elif re.match(r"^import\s+", lines[i]):
            names += [p.strip().split(" as ")[-1].split(".")[0].strip()
                      for p in lines[i].split("import", 1)[1].split(",")
                      if p.strip()]
        i += 1
    return names


def _shift_import(line: str) -> str:
    """One more dot on a relative import, for one package level down."""
    return RELFROM.sub(lambda m: m.group(1) + "." + m.group(2), line)


def split(path: Path, cls_name: str) -> None:
    lines = path.read_text(encoding="utf-8").splitlines(keepends=True)
    start = next(i for i, l in enumerate(lines)
                 if l.startswith(f"class {cls_name}"))
    end = next((i for i in range(start + 1, len(lines))
                if lines[i] and not lines[i][0].isspace()
                and not lines[i].startswith("#")), len(lines))
    head, body, tail = lines[:start], lines[start + 1:end], lines[end:]

    # Segment the class body at section banners; whatever precedes the
    # first banner is the class's own (docstring, attrs, __init__).
    segments: list[tuple[str, list[str]]] = [("core", [])]
    i = 0
    while i < len(body):
        line = body[i]
        m = INLINE.match(line.rstrip("\n"))
        if m:
            segments.append((_mixin_name(m.group(1)), [line]))
            i += 1
            continue
        if DASH.match(line.rstrip("\n")) and i + 2 < len(body) \
                and TITLE.match(body[i + 1].rstrip("\n")) \
                and DASH.match(body[i + 2].rstrip("\n")):
            name = TITLE.match(body[i + 1].rstrip("\n")).group(1)
            segments.append((_mixin_name(name),
                             [_shift_import(l) for l in body[i:i + 3]]))
            i += 3
            continue
        segments[-1][1].append(_shift_import(line))
        i += 1
    if not segments[0][1]:
        segments[0][1] = ["    pass\n"]

    pkg = path.with_suffix("")
    stem = pkg.name
    pkg.mkdir(exist_ok=True)

    # Shared: everything module-level except the target class. A tail
    # statement that names the class (a factory, an __all__ mention)
    # moves to core.py so it lands under the class it uses.
    shared_lines = [_shift_import(l) for l in head]
    core_tail: list[str] = []
    blocks: list[list[str]] = [[]]
    for line in tail:
        blocks[-1].append(line)
        if line.strip() == "":
            blocks.append([])
    for block in blocks:
        target = core_tail if any(
            re.search(rf"\b{cls_name}\b", l) for l in block) else shared_lines
        target += [_shift_import(l) for l in block]

    shared_names = [n for l in shared_lines if (n := _bound_name(l))]
    shared_names += [n for n in _import_names(shared_lines)
                     if n not in shared_names]
    if "__all__" not in shared_names:
        shared_lines += ["\n__all__ = [\n"
                         + "".join(f'    "{n}",\n' for n in shared_names)
                         + "]\n"]
        shared_names.append("__all__")
    (pkg / "_shared.py").write_text("".join(shared_lines), encoding="utf-8")
    shared_imp = ("from ._shared import (\n"
                  + "".join(f"    {n},\n" for n in shared_names
                            if n != "__all__") + ")\n")

    mixin_names: list[str] = []
    for name, seg in segments[1:]:
        mixin_names.append(name)
        if not any(re.match(r"    [^#\s]", l) for l in seg):
            seg = seg + ["    pass\n"]
        (pkg / f"x_{name[1:].lower()}.py").write_text(
            "from __future__ import annotations\n\n" + shared_imp
            + f"\n\nclass {name}:\n" + "".join(seg), encoding="utf-8")

    mixin_imp = "".join(f"from .x_{n[1:].lower()} import {n}\n"
                        for n in mixin_names)
    (pkg / "core.py").write_text(
        "from __future__ import annotations\n\n" + shared_imp + mixin_imp
        + f"\n\nclass {cls_name}({', '.join(mixin_names)}):\n"
        + "".join(segments[0][1]) + "".join(core_tail), encoding="utf-8")

    core_names = [n for l in core_tail if (n := _bound_name(l))]
    core_imp = ("from .core import (\n    " + cls_name + ",\n"
                + "".join(f"    {n},\n" for n in core_names
                          if n != "__all__") + ")\n")
    all_names = [n for n in shared_names + [cls_name] + core_names
                 if n != "__all__"]
    (pkg / "__init__.py").write_text(
        f'"""{stem}: split package (see core.py and x_*.py)."""\n\n'
        + shared_imp + core_imp
        + "\n__all__ = [\n" + "".join(f'    "{n}",\n' for n in all_names)
        + "]\n", encoding="utf-8")

    path.unlink()
    print(f"{path} -> {pkg}/ ({len(mixin_names)} mixins)")


if __name__ == "__main__":
    split(Path(sys.argv[1]), sys.argv[2])
