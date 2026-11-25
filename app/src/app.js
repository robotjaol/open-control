"use strict";
/* open-control preview. Every number shown is read from OC_DATA (built from experiments/index.json,
   datasets/catalog/catalog.json and the dataset CSV). Nothing is hardcoded here. */
(function () {
  const D = window.OC_DATA;
  const I = D.index;
  const view = document.getElementById("view");

  const SECTIONS = [
    ["overview", "Overview"], ["paths", "Learning Paths"], ["algorithms", "Algorithm Library"],
    ["motor", "Motor Lab"], ["plc", "PLC"], ["process", "Process"], ["research", "Research Library"],
    ["datasets", "Dataset Catalog"], ["gallery", "Experiment Gallery"], ["projects", "Projects"],
    ["docs", "Docs"],
  ];

  const esc = (s) => String(s == null ? "" : s).replace(/[&<>"']/g,
    (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
  const fmt = (x, nd = 3) => {
    if (x == null) return "n/a";
    if (typeof x === "boolean") return x ? "yes" : "no";
    if (typeof x === "number") return Number.isInteger(x) ? String(x) : Number(x.toPrecision(nd)).toString();
    return String(x);
  };
  const statusClass = (s) => (/Validated|completed|verified|Complete/.test(s) ? "ok"
    : /Blocked|Not validated|Missing|failed/.test(s) ? "bad" : "warn");
  const chip = (text, cls) => `<span class="chip ${cls || ""}">${esc(text)}</span>`;
  const nbById = Object.fromEntries(I.notebooks.map((n) => [n.id, n]));
  const nbByNum = Object.fromEntries(I.notebooks.map((n) => [n.number, n]));
  const expById = Object.fromEntries(I.experiments.map((e) => [e.experiment_id, e]));

  function table(rows, cols, heads) {
    if (!rows.length) return `<div class="empty">No rows.</div>`;
    const h = (heads || cols).map((c) => `<th>${esc(c)}</th>`).join("");
    const b = rows.map((r) => "<tr>" + cols.map((c) => {
      const v = r[c];
      return typeof v === "number" ? `<td class="num">${esc(fmt(v))}</td>` : `<td>${esc(fmt(v))}</td>`;
    }).join("") + "</tr>").join("");
    return `<div class="tablewrap"><table><thead><tr>${h}</tr></thead><tbody>${b}</tbody></table></div>`;
  }

  /* generic bar chart from any metrics table: label key + numeric key found in the data */
  const LABEL_KEYS = ["L_over_T", "rule", "variant", "controller", "setting", "payload_kg", "scenario"];
  const VALUE_KEYS = ["iae", "iae_setpoint", "iae_total", "iae_after", "overshoot_percent", "overshoot_pct",
    "settling_time_ms", "settling_s", "tv"];
  function charts(exp) {
    const out = [];
    for (const [name, val] of Object.entries(exp.metrics || {})) {
      if (!Array.isArray(val) || val.length < 2 || typeof val[0] !== "object") continue;
      const vk = VALUE_KEYS.find((k) => k in val[0] && typeof val[0][k] === "number");
      if (!vk) continue;
      const lk = LABEL_KEYS.filter((k) => k in val[0]);
      if (!lk.length) continue;
      const rows = val.slice(0, 40).map((r) => ({ label: lk.map((k) => `${k === "L_over_T" ? "L/T " : ""}${r[k]}`).join(" | "), v: r[vk] }));
      const max = Math.max(...rows.map((r) => Math.abs(r.v)), 1e-12);
      out.push(`<h3>${esc(name)}: ${esc(vk)}</h3><div class="bars" role="img" aria-label="Bar chart of ${esc(vk)} from metric table ${esc(name)}">` +
        rows.map((r) => `<div class="bar"><span>${esc(r.label)}</span><span class="track"><span class="fill" style="display:block;width:${(100 * Math.abs(r.v) / max).toFixed(1)}%"></span></span><span class="v">${esc(fmt(r.v))}</span></div>`).join("") +
        `</div>`);
      if (out.length >= 3) break;
    }
    return out.join("");
  }

  function statusBlock(st) {
    const rows = [["Content", st.content], ["Software", st.software], ["Research", st.research],
      ["Simulation", st.simulation], ["Hardware", st.hardware]];
    return `<div class="tablewrap"><table><tbody>${rows.map(([k, v]) =>
      `<tr><th>${k}</th><td>${chip({ ok: "ok", warn: "partial", bad: "not validated" }[statusClass(v)], statusClass(v))} ${esc(v)}</td></tr>`).join("")}</tbody></table></div>`;
  }

  function refTable(items) {
    const refs = Object.fromEntries(I.references.map((r) => [r.key, r]));
    const stds = Object.fromEntries(I.standards.map((r) => [r.key, r]));
    return table(items.map((i) => {
      const r = i.kind === "reference" ? refs[i.key] : stds[i.key];
      const cite = i.kind === "reference" ? `${r.authors} (${r.year}). ${r.title}` : `${r.code}, ${r.title}`;
      return { key: i.key, citation: cite, status: i.status };
    }), ["key", "citation", "status"], ["Key", "Citation", "Verification"]);
  }

  /* ---------- pages ---------- */
  function pageOverview() {
    const s = I.summary;
    const t = s.topic_status;
    const total = s.topics;
    const colors = { "Simulation Validated": "#1d6b3a", "In Progress": "#c2891b", "Planned": "#b9bfb4" };
    const meter = Object.entries(t).map(([k, v]) =>
      `<i style="width:${(100 * v / total).toFixed(2)}%;background:${colors[k] || "#888"}" title="${esc(k)}: ${v}"></i>`).join("");
    return `<h1>open-control</h1>
<p class="lede">A research-based laboratory for industrial control engineering: models, controllers, executed notebooks, structured experiment results and a source register. Built to be checked, not just read.</p>
<div class="notice"><b>Read this first.</b> ${esc(I.notice)}</div>
<div class="stats">
 <div class="stat"><b>${s.notebooks}</b><span>research notebooks</span></div>
 <div class="stat"><b>${s.checks_passed}/${s.checks_total}</b><span>automated checks passed</span></div>
 <div class="stat"><b>${s.references}</b><span>references (see Research Library)</span></div>
 <div class="stat"><b>${s.hardware_validated}</b><span>hardware-validated items</span></div>
</div>
<h2>Topic coverage</h2>
<p>${total} topics are defined. Status counts: ${Object.entries(t).map(([k, v]) => `${esc(k)} ${v}`).join(", ")}.</p>
<div class="meter" role="img" aria-label="Topic status proportions">${meter}</div>
<p class="legend">${Object.entries(colors).map(([k, c]) => `<span><i style="display:inline-block;width:10px;height:10px;background:${c};margin-right:4px"></i>${esc(k)}</span>`).join("")}</p>
<h2>Status vocabulary</h2><p>${I.status_vocabulary.map((x) => chip(x)).join("")}</p>
<p>Content, software, research, simulation and hardware status are reported separately for every notebook.</p>
<h2>Start here</h2>
<div class="grid">
 <div class="card"><h3><a class="title" href="#/paths">Learning paths</a></h3><p>Eight levels from fundamentals to integrated projects, with the notebooks that exist for each.</p></div>
 <div class="card"><h3><a class="title" href="#/algorithms">Algorithm library</a></h3><p>Search and filter ${s.notebooks} notebooks by category and difficulty.</p></div>
 <div class="card"><h3><a class="title" href="#/gallery">Experiment gallery</a></h3><p>Figures produced by the executed notebooks.</p></div>
</div>`;
  }

  function pagePaths() {
    const lv = I.roadmap_levels.map((l) => {
      const nbs = I.notebooks.filter((n) => n.roadmap_level === l.level);
      return `<div class="card"><h3>Level ${l.level}: ${esc(l.name)}</h3>` +
        (nbs.length ? "<ul>" + nbs.map((n) => `<li><a href="#/algorithms/${esc(n.id)}">${esc(n.number)} ${esc(n.title)}</a></li>`).join("") + "</ul>"
          : `<p class="empty">No notebook yet.</p>`) + `</div>`;
    }).join("");
    return `<h1>Learning paths</h1><p class="lede">Levels 0 to 7. Recommended study hours and prerequisites are in the roadmap document and are recommendations, not measurements.</p>
<p><a href="#/docs/learning-roadmap">Read the roadmap</a></p><div class="grid">${lv}</div>`;
  }

  function algoCards(list) {
    if (!list.length) return `<div class="empty">No notebook matches. Clear a filter or change the search.</div>`;
    return `<div class="grid">${list.map((n) => {
      const e = expById[n.experiments[0]];
      return `<div class="card"><h3><a class="title" href="#/algorithms/${esc(n.id)}">${esc(n.number)}. ${esc(n.title)}</a></h3>
<p>${chip(n.category)}${chip(n.difficulty)}${chip("Level " + n.roadmap_level)}</p>
<p style="margin:4px 0">${e ? esc(e.algorithm) : ""}</p>
<p>${e ? chip(e.status.simulation, statusClass(e.status.simulation)) : ""}${chip("Hardware: not validated", "bad")}</p></div>`;
    }).join("")}</div>`;
  }

  function filteredList(base, hash) {
    const cats = [...new Set(base.map((n) => n.category))].sort();
    const diffs = ["Beginner", "Intermediate", "Advanced"];
    const q = new URLSearchParams((location.hash.split("?")[1]) || "");
    const state = { q: q.get("q") || "", cat: q.get("cat") || "", diff: q.get("diff") || "" };
    const el = document.createElement("div");
    const draw = () => {
      const needle = state.q.toLowerCase();
      const list = base.filter((n) => {
        const e = expById[n.experiments[0]];
        const hay = [n.title, n.category, e && e.algorithm, e && e.application, e && e.plant_model].join(" ").toLowerCase();
        return (!needle || hay.includes(needle)) && (!state.cat || n.category === state.cat) && (!state.diff || n.difficulty === state.diff);
      });
      el.querySelector("#res").innerHTML = `<p aria-live="polite">${list.length} of ${base.length} notebooks</p>` + algoCards(list);
    };
    el.innerHTML = `<div class="toolbar">
<label>Search<input type="search" id="q" value="${esc(state.q)}" placeholder="PID, MPC, BLDC"></label>
<label>Category<select id="cat"><option value="">All</option>${cats.map((c) => `<option ${state.cat === c ? "selected" : ""}>${esc(c)}</option>`).join("")}</select></label>
<label>Difficulty<select id="diff"><option value="">All</option>${diffs.map((c) => `<option ${state.diff === c ? "selected" : ""}>${c}</option>`).join("")}</select></label>
</div><div id="res"></div>`;
    el.querySelector("#q").addEventListener("input", (ev) => { state.q = ev.target.value; draw(); });
    el.querySelector("#cat").addEventListener("change", (ev) => { state.cat = ev.target.value; draw(); });
    el.querySelector("#diff").addEventListener("change", (ev) => { state.diff = ev.target.value; draw(); });
    draw();
    return el;
  }

  function pageAlgorithms(id) {
    if (id) return pageNotebook(id);
    const wrap = document.createElement("div");
    wrap.innerHTML = `<h1>Algorithm library</h1><p class="lede">Each entry is an executed research notebook with its structured result file.</p>`;
    wrap.appendChild(filteredList(I.notebooks));
    return wrap;
  }

  function pageNotebook(id) {
    const n = nbById[id];
    if (!n) return `<div class="empty">No notebook with id ${esc(id)}.</div>`;
    const e = expById[n.experiments[0]];
    const checks = e.checks.map((c) => `<div class="check"><span>${chip(c.passed ? "pass" : "FAIL", c.passed ? "ok" : "bad")}</span><span>${esc(c.name)}</span><span class="d">${esc(c.detail)}</span></div>`).join("");
    return `<p class="crumb"><a href="#/algorithms">Algorithm library</a> / ${esc(n.number)}</p>
<h1>${esc(n.title)}</h1>
<p>${chip(n.category)}${chip(n.difficulty)}${chip("Roadmap level " + n.roadmap_level)}${chip(e.experiment_type)}</p>
<p class="lede">${esc(e.algorithm)}. Application: ${esc(e.application)}. Plant: ${esc(e.plant_model)}.</p>
<h2>Status</h2>${statusBlock(e.status)}
<p>Notebook file: <code>${esc(n.path)}</code>. Result file: <code>${esc(e.result_file)}</code>. Executed ${esc(e.execution_timestamp)}.
Data: ${esc(e.dataset.kind)}. ${esc(e.dataset.description)}</p>
<h2>Charts drawn from the result file</h2>${charts(e) || `<div class="empty">This result file has no metric table suited to a bar chart.</div>`}
<h2>Figures</h2><div class="figs">${e.figures.map(figure).join("")}</div>
<h2>Automated checks (${e.checks.filter((c) => c.passed).length}/${e.checks.length})</h2>${checks}
<h2>Limitations</h2><ul>${e.limitations.map((l) => `<li>${esc(l)}</li>`).join("")}</ul>
<h2>References</h2>${refTable(n.references)}`;
  }

  function figure(path) {
    const name = path.split("/").slice(-2).join("/");
    return `<figure><a href="${esc(path)}"><img loading="lazy" src="${esc(path)}" alt="Figure ${esc(name)} generated by the notebook"></a><figcaption>${esc(name)}</figcaption></figure>`;
  }

  function pageLab(lab, title, blurb, sectionPrefix) {
    const base = I.notebooks.filter((n) => n.lab === lab);
    const topics = I.topics.filter((t) => t.section.startsWith(sectionPrefix));
    const counts = {};
    topics.forEach((t) => { counts[t.status] = (counts[t.status] || 0) + 1; });
    const wrap = document.createElement("div");
    wrap.innerHTML = `<h1>${esc(title)}</h1><p class="lede">${esc(blurb)}</p>`;
    wrap.appendChild(filteredList(base));
    const extra = document.createElement("div");
    extra.innerHTML = `<h2>Topic coverage in this area</h2><p>${topics.length} topics: ${Object.entries(counts).map(([k, v]) => `${esc(k)} ${v}`).join(", ")}.</p>` +
      table(topics.filter((t) => t.status !== "Planned"), ["topic", "status", "note"], ["Topic", "Status", "Scope note"]);
    wrap.appendChild(extra);
    return wrap;
  }

  function pageResearch() {
    const wrap = document.createElement("div");
    wrap.innerHTML = `<h1>Research library</h1><p class="lede">${I.references.length} references and ${I.standards.length} standards. Verification means a bibliographic record was found on a publisher, index or library page; no full text was read.</p>
<div class="toolbar"><label>Search<input type="search" id="rq" placeholder="author, title, year"></label>
<label>Status<select id="rs"><option value="">All</option><option>verified</option><option>partial</option></select></label></div><div id="rr"></div>`;
    const draw = () => {
      const q = wrap.querySelector("#rq").value.toLowerCase();
      const st = wrap.querySelector("#rs").value;
      const rows = I.references.filter((r) => (!st || r.verification.status === st) &&
        (!q || [r.authors, r.title, r.year, r.role].join(" ").toLowerCase().includes(q)))
        .map((r) => ({ key: r.key, authors: r.authors, year: r.year, title: r.title, status: r.verification.status, role: r.role, limits: r.limits }));
      wrap.querySelector("#rr").innerHTML = `<p aria-live="polite">${rows.length} references</p>` +
        table(rows, ["key", "authors", "year", "title", "status", "role", "limits"], ["Key", "Authors", "Year", "Title", "Status", "Role here", "Limitation"]) +
        `<h2>Standards</h2>` + table(I.standards.map((s) => ({ code: s.body + " " + s.code, title: s.title, relevance: s.relevance, note: s.status_note, status: s.verification.status })),
          ["code", "title", "relevance", "note", "status"], ["Code", "Title", "Relevance", "Status note", "Verification"]) +
        `<h2>Named in the brief, not verified</h2>` + table(I.unverified_candidates, ["item", "note"], ["Item", "Note"]);
    };
    wrap.querySelector("#rq").addEventListener("input", draw);
    wrap.querySelector("#rs").addEventListener("change", draw);
    draw();
    return wrap;
  }

  function parseCsv(text) {
    const lines = text.trim().split(/\r?\n/);
    const head = lines[0].split(",");
    return { head, rows: lines.slice(1).map((l) => l.split(",").map(Number)) };
  }

  function lineChart(csv, xi, yis) {
    const w = 720, h = 260, pad = 36;
    const xs = csv.rows.map((r) => r[xi]);
    const all = yis.flatMap((yi) => csv.rows.map((r) => r[yi]));
    const [x0, x1] = [Math.min(...xs), Math.max(...xs)];
    const [y0, y1] = [Math.min(...all), Math.max(...all)];
    const sx = (x) => pad + (w - 2 * pad) * (x - x0) / (x1 - x0 || 1);
    const sy = (y) => h - pad - (h - 2 * pad) * (y - y0) / (y1 - y0 || 1);
    const cols = ["#1f5f8b", "#b9bfb4", "#9a2a2a"];
    const paths = yis.map((yi, k) => `<polyline fill="none" stroke="${cols[k % 3]}" stroke-width="${k ? 1 : 1.6}" points="${csv.rows.map((r) => `${sx(r[xi]).toFixed(1)},${sy(r[yi]).toFixed(1)}`).join(" ")}"/>`).join("");
    const legend = yis.map((yi, k) => `<text x="${pad + 6 + k * 150}" y="14" font-size="11" fill="${cols[k % 3]}">${esc(csv.head[yi])}</text>`).join("");
    return `<svg viewBox="0 0 ${w} ${h}" width="100%" role="img" aria-label="Line chart of the dataset columns ${esc(yis.map((i) => csv.head[i]).join(", "))} against ${esc(csv.head[xi])}">
<rect x="${pad}" y="${pad}" width="${w - 2 * pad}" height="${h - 2 * pad}" fill="none" stroke="#d9dcd6"/>${paths}${legend}
<text x="${pad}" y="${h - 12}" font-size="11" fill="#5b6470">${esc(csv.head[xi])}: ${fmt(x0)} to ${fmt(x1)}</text>
<text x="${w - pad}" y="${h - 12}" font-size="11" text-anchor="end" fill="#5b6470">y range ${fmt(y0)} to ${fmt(y1)}</text></svg>`;
  }

  function pageDatasets() {
    const cat = D.catalog;
    const cards = cat.datasets.map((d) => {
      const csvText = D.csv[d.file];
      let chart = "";
      if (csvText) {
        const csv = parseCsv(csvText);
        const yi = ["v_true_m_s", "v_meas_m_s"].map((k) => csv.head.indexOf(k)).filter((i) => i >= 0);
        if (yi.length) chart = `<h3>Preview drawn from the CSV</h3>` + lineChart(csv, 0, yi);
      }
      return `<div class="card" style="grid-column:1/-1"><h3>${esc(d.name)}</h3>
<p>${chip(d.kind, d.kind === "measured_experiment" || d.kind === "public_dataset" ? "ok" : "warn")}${chip(d.n_rows + " rows")}${chip("Ts " + d.sampling_time_s + " s")}${chip(d.license)}</p>
<p>${esc(d.description)}</p><p>Origin: ${esc(d.origin)}. File: <code>${esc(d.file)}</code>. SHA-256: <code>${esc(d.sha256.slice(0, 16))}...</code></p>
${table(d.columns, ["name", "unit", "description"], ["Column", "Unit", "Description"])}
<h3>Limitations</h3><ul>${d.limitations.map((l) => `<li>${esc(l)}</li>`).join("")}</ul>${chart}</div>`;
    }).join("");
    return `<h1>Dataset catalog</h1><p class="lede">${esc(cat.notice)}</p><div class="grid">${cards}</div>
<h2>Not included</h2>${table(cat.candidates_not_included, ["topic", "note"], ["Topic", "Note"])}`;
  }

  function pageGallery() {
    const cats = [...new Set(I.experiments.map((e) => e.category))].sort();
    const wrap = document.createElement("div");
    wrap.innerHTML = `<h1>Experiment gallery</h1><p class="lede">Every figure below was written by an executed notebook. Open a figure to see it at full size.</p>
<div class="toolbar"><label>Category<select id="gc"><option value="">All</option>${cats.map((c) => `<option>${esc(c)}</option>`).join("")}</select></label>
<label>Search<input type="search" id="gq" placeholder="figure or experiment"></label></div><div id="gr"></div>`;
    const draw = () => {
      const c = wrap.querySelector("#gc").value, q = wrap.querySelector("#gq").value.toLowerCase();
      const items = I.experiments.filter((e) => !c || e.category === c).flatMap((e) =>
        e.figures.map((f) => ({ e, f }))).filter((x) => !q || (x.f + x.e.title).toLowerCase().includes(q));
      wrap.querySelector("#gr").innerHTML = `<p aria-live="polite">${items.length} figures</p>` + (items.length
        ? `<div class="figs">${items.map((x) => figure(x.f).replace("</figcaption>", ` | <a href="#/algorithms/${esc(x.e.notebook_id)}">${esc(x.e.experiment_id)}</a></figcaption>`)).join("")}</div>`
        : `<div class="empty">No figures match.</div>`);
    };
    wrap.querySelector("#gc").addEventListener("change", draw);
    wrap.querySelector("#gq").addEventListener("input", draw);
    draw();
    return wrap;
  }

  function pageProjects() {
    const wrap = document.createElement("div");
    const sts = [...new Set(I.projects.map((p) => p.status))];
    wrap.innerHTML = `<h1>Projects</h1><p class="lede">Fifteen end-to-end projects are defined. None is complete; see the catalog document for what exists.</p>
<div class="toolbar"><label>Status<select id="ps"><option value="">All</option>${sts.map((s) => `<option>${esc(s)}</option>`).join("")}</select></label></div><div id="pr"></div>`;
    const draw = () => {
      const st = wrap.querySelector("#ps").value;
      const list = I.projects.filter((p) => !st || p.status === st);
      wrap.querySelector("#pr").innerHTML = `<div class="grid">${list.map((p) => `<div class="card"><h3>Project ${String(p.number).padStart(2, "0")}: ${esc(p.title)}</h3>
<p>${chip(p.status, p.status === "Blocked" ? "bad" : "warn")}${p.notebooks.map((n) => `<a class="chip" href="#/algorithms/${esc((nbByNum[n] || {}).id)}">notebook ${esc(n)}</a>`).join("")}</p><p>${esc(p.note)}</p></div>`).join("")}</div>
<p><a href="#/docs/projects">Project catalog document</a></p>`;
    };
    wrap.querySelector("#ps").addEventListener("change", draw);
    draw();
    return wrap;
  }

  /* minimal markdown renderer: headings, lists, tables, code, links, emphasis. Text is escaped first. */
  function md(src) {
    const lines = src.replace(/<!--[\s\S]*?-->/g, "").split(/\r?\n/);
    let out = "", i = 0;
    const inline = (t) => esc(t).replace(/`([^`]+)`/g, "<code>$1</code>")
      .replace(/\*\*([^*]+)\*\*/g, "<b>$1</b>")
      .replace(/\[([^\]]+)\]\(([^)\s]+)\)/g, (m, a, u) => {
        const dm = u.match(/^(?:\.\/)?([\w-]+)\.md(?:#.*)?$/);
        if (dm && D.docs[dm[1] + ".md"]) return `<a href="#/docs/${dm[1]}">${a}</a>`;
        return /^https?:/.test(u) ? `<a href="${u}" rel="noopener">${a}</a>` : `<span>${a}</span>`;
      });
    while (i < lines.length) {
      const l = lines[i];
      if (l.startsWith("```")) { let c = ""; i++; while (i < lines.length && !lines[i].startsWith("```")) c += esc(lines[i++]) + "\n"; i++; out += `<pre><code>${c}</code></pre>`; continue; }
      if (/^#{1,4} /.test(l)) { const n = l.match(/^#+/)[0].length; out += `<h${n}>${inline(l.slice(n + 1))}</h${n}>`; i++; continue; }
      if (l.startsWith("|")) {
        const rows = []; while (i < lines.length && lines[i].startsWith("|")) rows.push(lines[i++]);
        const cells = (r) => r.replace(/^\||\|$/g, "").split("|").map((c) => c.trim());
        const head = cells(rows[0]); const body = rows.slice(2).map(cells);
        out += `<div class="tablewrap"><table><thead><tr>${head.map((c) => `<th>${inline(c)}</th>`).join("")}</tr></thead><tbody>${body.map((r) => `<tr>${r.map((c) => `<td>${inline(c)}</td>`).join("")}</tr>`).join("")}</tbody></table></div>`;
        continue;
      }
      if (/^\s*[-*] /.test(l)) { out += "<ul>"; while (i < lines.length && /^\s*[-*] /.test(lines[i])) out += `<li>${inline(lines[i++].replace(/^\s*[-*] /, ""))}</li>`; out += "</ul>"; continue; }
      if (/^\d+\. /.test(l)) { out += "<ol>"; while (i < lines.length && /^\d+\. /.test(lines[i])) out += `<li>${inline(lines[i++].replace(/^\d+\. /, ""))}</li>`; out += "</ol>"; continue; }
      if (!l.trim()) { i++; continue; }
      let p = l; i++;
      while (i < lines.length && lines[i].trim() && !/^(#|\||```|\s*[-*] |\d+\. )/.test(lines[i])) p += " " + lines[i++];
      out += `<p>${inline(p)}</p>`;
    }
    return out;
  }

  function pageDocs(name) {
    const names = Object.keys(D.docs).sort();
    if (name && D.docs[name + ".md"]) {
      return `<p class="crumb"><a href="#/docs">Docs</a> / ${esc(name)}</p><div class="md">${md(D.docs[name + ".md"])}</div>`;
    }
    const groups = { docs: "Documents", research: "Research registers" };
    return `<h1>Docs</h1><p class="lede">Documents and research registers shipped with the repository. Generated pages say so in their source.</p>` +
      Object.entries(groups).map(([g, label]) => `<h2>${label}</h2><ul>${names.filter((n) => (g === "docs" ? !n.includes("/") : n.startsWith("research/")))
        .map((n) => `<li><a href="#/docs/${esc(n.replace(/\.md$/, ""))}">${esc(n.replace(/\.md$/, ""))}</a></li>`).join("")}</ul>`).join("");
  }

  /* ---------- router ---------- */
  function render() {
    const parts = (location.hash.replace(/^#\//, "").split("?")[0] || "overview").split("/");
    const sec = parts[0];
    const rest = parts.slice(1).join("/");
    document.querySelectorAll("#nav a").forEach((a) => { if (a.dataset.s === sec) a.setAttribute("aria-current", "page"); else a.removeAttribute("aria-current"); });
    let content;
    switch (sec) {
      case "overview": content = pageOverview(); break;
      case "paths": content = pagePaths(); break;
      case "algorithms": content = pageAlgorithms(rest); break;
      case "motor": content = pageLab("motor", "Motor lab", "DC, BLDC and PMSM drives and the belt conveyor, each with its executed experiment.", "3.5"); break;
      case "plc": content = pageLab("plc", "PLC and sequential control", "State machines and PLC scan-cycle emulation. These are Python emulations, not vendor PLC programs.", "3.6"); break;
      case "process": content = pageLab("process", "Process and loop control", "Feedback fundamentals, identification, PID, tuning rules and a nonlinear tank.", "3.8"); break;
      case "research": content = pageResearch(); break;
      case "datasets": content = pageDatasets(); break;
      case "gallery": content = pageGallery(); break;
      case "projects": content = pageProjects(); break;
      case "docs": content = pageDocs(rest); break;
      default: content = `<div class="empty">Page not found. <a href="#/overview">Go to the overview</a>.</div>`;
    }
    view.innerHTML = "";
    if (typeof content === "string") view.innerHTML = content; else view.appendChild(content);
    document.title = (SECTIONS.find((s) => s[0] === sec) || ["", "open-control"])[1] + " | open-control";
    window.scrollTo(0, 0);
  }

  const nav = document.getElementById("nav");
  nav.innerHTML = SECTIONS.map(([k, label]) => `<a href="#/${k}" data-s="${k}">${esc(label)}</a>`).join("");
  const btn = document.getElementById("menu-btn");
  btn.addEventListener("click", () => { const o = nav.classList.toggle("open"); btn.setAttribute("aria-expanded", String(o)); });
  nav.addEventListener("click", () => { nav.classList.remove("open"); btn.setAttribute("aria-expanded", "false"); });
  document.getElementById("foot-meta").textContent = `Data: ${I.summary.experiments} experiments, ${I.summary.checks_total} checks.`;
  window.addEventListener("hashchange", render);
  render();
  window.__OC_RENDER = render;
})();
