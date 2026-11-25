"""Write experiments/metadata, experiments/configurations and experiments/index.json.

index.json is the single manifest the preview app reads. It is generated from the notebook specs,
the executed result files and tools/refs.py, never written by hand.
"""

from __future__ import annotations

import refs
import registry as R
import taxonomy

CONFIG_KEYS = ["controller_parameters", "plant_parameters", "sampling_time_s", "input_configuration",
               "disturbance_configuration", "operating_conditions", "number_of_trials",
               "random_seed", "reproducibility_instructions"]

CATEGORY = {
    "01_fundamentals": "Fundamentals", "02_classical": "Classical control",
    "03_discrete": "Sequential and PLC", "04_motor_control": "Motor and drive control",
    "05_intelligent": "Intelligent control", "06_advanced_control": "Advanced control",
}


def main():
    metas = R.load_metas()
    experiments, notebooks = [], []
    for m in metas:
        for exp_id in m["experiments"]:
            res = R.load_result(exp_id)
            if res is None:
                continue
            cs = R.checks_summary(res)
            status = dict(
                content=R.content_status(m),
                software="Executed from a clean kernel; local unit tests passed at build time; "
                         "GitHub Actions has not been run on this repository",
                research=f"{R.ref_summary(m)['verified']} verified, {R.ref_summary(m)['partial']} partial",
                simulation=f"{cs['passed']}/{cs['total']} checks passed ({res['result_status']})",
                hardware="Not validated",
            )
            meta_out = dict(
                experiment_id=exp_id, title=res["title"], notebook=m["notebook"], notebook_id=m["id"],
                algorithm=res["algorithm"], application=res["application"], plant_model=res["plant_model"],
                category=CATEGORY[m["folder"]], difficulty=taxonomy.difficulty(m["level"]),
                roadmap_level=taxonomy.NOTEBOOK_ROADMAP[m["number"]],
                lab=taxonomy.LAB_OF[m["number"]], experiment_type=res["experiment_type"],
                dataset=res["dataset"], references=R.ref_summary(m)["items"], status=status,
                figures=res["figure_references"], limitations=res["limitations"],
                checks=res["checks"], metrics=res["metrics"], execution_timestamp=res["execution_timestamp"],
                result_file=f"experiments/results/{exp_id}.json",
            )
            R.write_json(R.ROOT / "experiments" / "metadata" / f"{exp_id}.json",
                         {k: v for k, v in meta_out.items() if k not in ("metrics",)})
            R.write_json(R.ROOT / "experiments" / "configurations" / f"{exp_id}.json",
                         {k: res[k] for k in CONFIG_KEYS})
            experiments.append(meta_out)
    for m in metas:
        rs = R.ref_summary(m)
        notebooks.append(dict(
            id=m["id"], number=m["number"], title=m["title"], path=m["notebook"], category=CATEGORY[m["folder"]],
            difficulty=taxonomy.difficulty(m["level"]), roadmap_level=taxonomy.NOTEBOOK_ROADMAP[m["number"]],
            lab=taxonomy.LAB_OF[m["number"]], experiments=m["experiments"], references=rs["items"],
            content_status=R.content_status(m)))
    rows = taxonomy.topic_rows()
    ref_list = [dict(key=k, **{kk: v for kk, v in r.items() if kk != "verification"},
                     verification=r["verification"]) for k, r in refs.REFS.items()]
    std_list = [dict(key=k, **{kk: v for kk, v in r.items() if kk != "verification"},
                     verification=r["verification"]) for k, r in refs.STANDARDS.items()]
    index = dict(
        schema_version="1.0",
        project="open-control",
        notice="Every number in this manifest comes from executed notebooks. All experiments are "
               "software simulations; no hardware validation exists.",
        summary=dict(
            notebooks=len(notebooks), experiments=len(experiments),
            checks_total=sum(len(e["checks"]) for e in experiments),
            checks_passed=sum(1 for e in experiments for c in e["checks"] if c["passed"]),
            references=len(ref_list), standards=len(std_list),
            topics=len(rows), topic_status=R.topic_counts(rows),
            hardware_validated=0),
        notebooks=notebooks, experiments=experiments, topics=rows,
        roadmap_levels=[dict(level=k, name=v) for k, v in taxonomy.ROADMAP_LEVELS.items()],
        projects=[dict(number=n, title=t, status=s, notebooks=nb, note=note)
                  for n, t, s, nb, note in taxonomy.PROJECTS],
        references=ref_list, standards=std_list,
        unverified_candidates=[dict(item=a, note=b) for a, b in refs.UNVERIFIED_CANDIDATES],
        status_vocabulary=["Planned", "In Progress", "Research Verified", "Simulation Validated",
                           "Hardware Validated"],
    )
    R.write_json(R.ROOT / "experiments" / "index.json", index)
    print("index:", index["summary"])


if __name__ == "__main__":
    main()
