"""Cross-file consistency: specs, results, registers, catalog and the preview manifest."""

import json
import re
import sys
from pathlib import Path

import jsonschema
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))

import refs  # noqa: E402
import registry as R  # noqa: E402
import taxonomy  # noqa: E402

METAS = R.load_metas()


def test_every_spec_has_a_result_file_and_notebook():
    for m in METAS:
        assert (ROOT / m["notebook"]).exists(), m["notebook"]
        for exp in m["experiments"]:
            assert R.load_result(exp) is not None, exp


@pytest.mark.parametrize("m", METAS, ids=[m["id"] for m in METAS])
def test_result_matches_schema_and_has_no_failed_check(m):
    schema = json.loads((ROOT / "experiments/schema/experiment_result.schema.json").read_text())
    for exp in m["experiments"]:
        res = R.load_result(exp)
        jsonschema.validate(res, schema)
        assert res["result_status"] == "completed"
        assert res["experiment_type"] == "software_simulation"
        assert res["hardware_configuration"] is None
        assert all(c["passed"] for c in res["checks"])
        assert res["notebook"] == m["notebook"]
        for fig in res["figure_references"]:
            assert (ROOT / fig).exists(), fig


def test_references_used_by_specs_exist_and_carry_verification():
    for m in METAS:
        for k in m["refs"]:
            v = refs.REFS[k]["verification"]
            assert v["status"] in {"verified", "partial"} and v["evidence_url"].startswith("http")
        for k in m.get("standards", []):
            assert k in refs.STANDARDS


def test_partial_references_state_what_is_unconfirmed():
    for k, r in refs.REFS.items():
        if r["verification"]["status"] == "partial":
            assert len(r["verification"]["note"]) > 40, k


def test_taxonomy_notebook_ids_exist_and_statuses_are_in_vocabulary():
    numbers = {m["number"] for m in METAS}
    vocab = {"Planned", "In Progress", "Research Verified", "Simulation Validated", "Hardware Validated"}
    for r in taxonomy.topic_rows():
        assert r["status"] in vocab
        assert set(r["notebooks"]) <= numbers, r["topic"]
        if r["status"] in {"Simulation Validated", "In Progress"}:
            assert r["notebooks"] and r["note"], r["topic"]
        if r["status"] == "Planned":
            assert not r["notebooks"], r["topic"]
    assert not any(r["status"] == "Hardware Validated" for r in taxonomy.topic_rows())


def test_project_catalog_matches_the_fifteen_projects():
    assert [p[0] for p in taxonomy.PROJECTS] == list(range(1, 16))
    status = {p[0]: p[2] for p in taxonomy.PROJECTS}
    assert status[14] == "Blocked"
    assert all(s in {"Planned", "In Progress", "Blocked"} for s in status.values())


def test_manifest_counts_match_result_files():
    idx = json.loads((ROOT / "experiments/index.json").read_text())
    results = list((ROOT / "experiments/results").glob("*.json"))
    assert idx["summary"]["experiments"] == len(results)
    assert idx["summary"]["checks_total"] == sum(len(json.loads(p.read_text())["checks"]) for p in results)
    assert idx["summary"]["hardware_validated"] == 0
    assert idx["summary"]["checks_passed"] == idx["summary"]["checks_total"]


def test_source_register_lists_every_reference():
    text = (ROOT / "research/source-register.md").read_text()
    for k in refs.REFS:
        assert f"| {k} |" in text, k
    assert str(len(refs.REFS)) in text


def test_synthetic_dataset_is_labelled_and_checksummed():
    cat = json.loads((ROOT / "datasets/catalog/catalog.json").read_text())
    for d in cat["datasets"]:
        assert d["kind"] in {"synthetic_illustrative", "simulated", "measured_experiment", "public_dataset"}
        if d["kind"] == "synthetic_illustrative":
            assert "synthetic" in d["name"].lower() and d["seed"] is not None
    assert not any(d["kind"] in {"measured_experiment", "public_dataset"} for d in cat["datasets"]), \
        "no measured dataset exists in this repository; remove this assertion only when one is added with provenance"


def test_generated_documents_have_no_hardware_validated_claim():
    for p in list((ROOT / "docs").glob("*.md")) + list((ROOT / "research").rglob("*.md")):
        for line in p.read_text().splitlines():
            if re.search(r"hardware[- ]validated", line, re.I) and not re.search(
                    r"no |not |0|zero|never|none|neither|vocabulary|reached|Validated, ", line, re.I):
                pytest.fail(f"{p.name}: suspicious hardware claim: {line}")
