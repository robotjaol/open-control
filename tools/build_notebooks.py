"""Generate notebooks from tools/specs (does not execute them)."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import nbgen  # noqa: E402
from specs import ORDER  # noqa: E402


def main(names=None):
    for name in names or ORDER:
        try:
            path = nbgen.write_notebook(name)
        except ModuleNotFoundError as exc:
            if names is None and exc.name and exc.name.endswith(name):
                print(f"skip (spec not written yet): {name}")
                continue
            raise
        print("wrote", path.relative_to(nbgen.ROOT))


if __name__ == "__main__":
    main(sys.argv[1:] or None)
