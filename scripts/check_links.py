"""Check relative links and image references in markdown files. External URLs are not fetched."""

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
LINK = re.compile(r"\]\(([^)\s]+)\)")
SKIP = {".git", "node_modules", "dist", ".pytest_cache", ".ruff_cache"}


def main() -> int:
    bad = 0
    n = 0
    for md in ROOT.rglob("*.md"):
        if SKIP & set(md.relative_to(ROOT).parts):
            continue
        in_code = False
        for ln, line in enumerate(md.read_text().splitlines(), 1):
            if line.strip().startswith("```"):
                in_code = not in_code
            if in_code:
                continue
            for target in LINK.findall(line):
                if re.match(r"^(https?:|mailto:|#)", target):
                    continue
                path = target.split("#")[0]
                if not path:
                    continue
                n += 1
                if not (md.parent / path).resolve().exists():
                    print(f"broken: {md.relative_to(ROOT)}:{ln} -> {target}")
                    bad += 1
    print(f"{n} relative links checked, {bad} broken")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
