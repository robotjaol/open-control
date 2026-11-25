"""Structured experiment results.

An ExperimentRecord is validated against experiments/schema/experiment_result.schema.json before
it is written, so an invalid record cannot reach the results folder. Results keep traceability
between the notebook (source), the configuration (controller and plant parameters), the software
environment and the saved figures.
"""

from __future__ import annotations

import json
import os
import platform
import sys
from dataclasses import dataclass, field
from datetime import datetime, timezone
from importlib import metadata
from pathlib import Path

from control_lab.metrics import jsonable

SCHEMA_VERSION = "1.0"
_PACKAGES = ("numpy", "scipy", "matplotlib", "pandas", "control")


def repo_root() -> Path:
    """Repository root: CONTROL_LAB_ROOT if set, else the nearest parent holding pyproject.toml."""
    env = os.environ.get("CONTROL_LAB_ROOT")
    if env:
        return Path(env)
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / "pyproject.toml").exists() and (parent / "src" / "control_lab").exists():
            return parent
    return Path.cwd()


def check(name: str, passed, detail: str = "") -> dict:
    """One automated comparison against an analytic or independent reference."""
    return {"name": name, "passed": bool(passed), "detail": str(detail)}


def environment_info() -> dict:
    pkgs = {}
    for name in _PACKAGES:
        try:
            pkgs[name] = metadata.version(name)
        except metadata.PackageNotFoundError:
            pkgs[name] = None
    return {
        "python": sys.version.split()[0],
        "platform": platform.platform(),
        "cpu_count": os.cpu_count(),
        "packages": pkgs,
    }


def load_schema() -> dict:
    return json.loads((repo_root() / "experiments" / "schema" / "experiment_result.schema.json")
                      .read_text())


def validate_record(payload: dict) -> None:
    """Raise jsonschema.ValidationError when the payload does not match the schema."""
    import jsonschema

    jsonschema.validate(payload, load_schema())


@dataclass
class ExperimentRecord:
    experiment_id: str
    title: str
    notebook: str
    algorithm: str
    application: str
    plant_model: str
    dataset: dict
    metrics: dict
    limitations: list
    controller_parameters: dict = field(default_factory=dict)
    plant_parameters: dict = field(default_factory=dict)
    sampling_time_s: float | None = None
    input_configuration: dict = field(default_factory=dict)
    disturbance_configuration: dict = field(default_factory=dict)
    operating_conditions: dict = field(default_factory=dict)
    number_of_trials: int = 1
    random_seed: int | None = None
    baseline: dict | None = None
    checks: list = field(default_factory=list)
    experiment_type: str = "software_simulation"
    hardware_configuration: dict | None = None
    firmware_version: str | None = None
    result_status: str = "completed"
    figure_references: list = field(default_factory=list)
    reproducibility_instructions: str = ""

    def to_payload(self) -> dict:
        if self.result_status == "completed" and any(not c["passed"] for c in self.checks):
            self.result_status = "partial"  # a failed check is never reported as completed
        if not self.reproducibility_instructions:
            self.reproducibility_instructions = (
                f"pip install -e .[development]; python tools/run_notebooks.py {self.notebook}"
            )
        payload = {
            "schema_version": SCHEMA_VERSION,
            "experiment_id": self.experiment_id,
            "title": self.title,
            "notebook": self.notebook,
            "algorithm": self.algorithm,
            "application": self.application,
            "plant_model": self.plant_model,
            "dataset": self.dataset,
            "experiment_type": self.experiment_type,
            "controller_parameters": self.controller_parameters,
            "plant_parameters": self.plant_parameters,
            "sampling_time_s": self.sampling_time_s,
            "input_configuration": self.input_configuration,
            "disturbance_configuration": self.disturbance_configuration,
            "operating_conditions": self.operating_conditions,
            "number_of_trials": self.number_of_trials,
            "random_seed": self.random_seed,
            "software_environment": environment_info(),
            "hardware_configuration": self.hardware_configuration,
            "firmware_version": self.firmware_version,
            "metrics": self.metrics,
            "baseline": self.baseline,
            "checks": self.checks,
            "limitations": self.limitations,
            "result_status": self.result_status,
            "figure_references": self.figure_references,
            "execution_timestamp": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "reproducibility_instructions": self.reproducibility_instructions,
        }
        return jsonable(payload)

    def save(self) -> Path:
        payload = self.to_payload()
        validate_record(payload)
        folder = repo_root() / "experiments" / "results"
        folder.mkdir(parents=True, exist_ok=True)
        path = folder / f"{self.experiment_id}.json"
        path.write_text(json.dumps(payload, indent=2))
        return path.relative_to(repo_root())


def save_figure(fig, experiment_id: str, name: str) -> str:
    """Save a matplotlib figure under experiments/figures/<id>/<name>.png, return the repo path."""
    folder = repo_root() / "experiments" / "figures" / experiment_id
    folder.mkdir(parents=True, exist_ok=True)
    path = folder / f"{name}.png"
    fig.savefig(path, dpi=110, bbox_inches="tight")
    return str(path.relative_to(repo_root()))


def load_records() -> list[dict]:
    folder = repo_root() / "experiments" / "results"
    return [json.loads(p.read_text()) for p in sorted(folder.glob("*.json"))]
