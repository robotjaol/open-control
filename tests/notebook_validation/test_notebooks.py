"""Structure of the committed notebooks. Execution itself is done by tools/run_notebooks.py."""

import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))

import registry as R  # noqa: E402

METAS = R.load_metas()


@pytest.mark.parametrize("m", METAS, ids=[m["id"] for m in METAS])
def test_notebook_has_all_fifteen_sections_in_order(m):
    heads = R.notebook_headings(ROOT / m["notebook"])
    assert heads == R.expected_headings()


@pytest.mark.parametrize("m", METAS, ids=[m["id"] for m in METAS])
def test_notebook_was_executed_without_errors(m):
    nb = json.loads((ROOT / m["notebook"]).read_text())
    code = [c for c in nb["cells"] if c["cell_type"] == "code"]
    assert code and all(c.get("execution_count") for c in code), "unexecuted code cell"
    for c in code:
        for out in c.get("outputs", []):
            assert out.get("output_type") != "error", out.get("ename")


@pytest.mark.parametrize("m", METAS, ids=[m["id"] for m in METAS])
def test_notebook_states_that_hardware_is_not_validated(m):
    nb = json.loads((ROOT / m["notebook"]).read_text())
    first = "".join(nb["cells"][0]["source"])
    assert "Not validated on hardware" in first or "Not validated" in first


@pytest.mark.parametrize("m", METAS, ids=[m["id"] for m in METAS])
def test_section_14_lists_limitations(m):
    nb = json.loads((ROOT / m["notebook"]).read_text())
    text = "\n".join("".join(c["source"]) for c in nb["cells"] if c["cell_type"] == "markdown")
    i = text.index("## 14. Research Discussion and Limitations")
    j = text.index("## 15. Summary")
    assert len(text[i:j]) > 400
