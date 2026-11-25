// Smoke test of the built site in jsdom: every section renders, filters work, no undefined/NaN text.
import { JSDOM, VirtualConsole } from "jsdom";
import assert from "node:assert/strict";
import path from "node:path";
import { fileURLToPath } from "node:url";

const here = path.dirname(fileURLToPath(import.meta.url));
const file = path.join(here, "..", "dist", "index.html");
const errors = [];
const vc = new VirtualConsole();
vc.on("jsdomError", (e) => { if (!/window.scrollTo/.test(e.message)) errors.push(String(e.stack || e.message || e)); });
vc.on("error", (e) => errors.push(String(e)));

const dom = await JSDOM.fromFile(file, { runScripts: "dangerously", resources: "usable", virtualConsole: vc });
await new Promise((r) => dom.window.addEventListener("load", r));
const { window } = dom;
window.scrollTo = () => {};
const doc = window.document;
const go = async (hash) => {
  window.location.hash = hash;
  window.dispatchEvent(new window.HashChangeEvent("hashchange"));
  await new Promise((r) => setTimeout(r, 10));
};
let passed = 0;
const ok = (cond, msg) => { assert.ok(cond, msg); passed++; };

const sections = ["overview", "paths", "algorithms", "motor", "plc", "process", "research", "datasets", "gallery", "projects", "docs"];
const nav = [...doc.querySelectorAll("#nav a")].map((a) => a.dataset.s);
assert.deepEqual(nav, sections); passed++;

for (const s of sections) {
  await go("#/" + s);
  const text = doc.getElementById("view").textContent;
  ok(doc.querySelector("#view h1"), `${s}: has h1`);
  ok(text.length > 200, `${s}: has content`);
  ok(!/undefined|NaN|\[object/.test(text), `${s}: no undefined/NaN/[object] in text`);
  ok(doc.querySelector("#nav a[aria-current=page]").dataset.s === s, `${s}: nav marks current page`);
}

// every notebook page and every doc page renders
await go("#/algorithms");
const links = [...doc.querySelectorAll("#view a.title")].map((a) => a.getAttribute("href"));
ok(links.length === 12, `12 notebook cards, got ${links.length}`);
for (const l of links) {
  await go(l);
  const t = doc.getElementById("view").textContent;
  ok(/Automated checks/.test(t) && !/undefined|NaN|\[object/.test(t), `${l}: renders`);
  ok(doc.querySelectorAll("#view .check").length > 0, `${l}: checks listed`);
  ok(doc.querySelectorAll("#view img").length > 0, `${l}: figures present`);
}
await go("#/docs");
const docLinks = [...doc.querySelectorAll("#view li a")].map((a) => a.getAttribute("href"));
ok(docLinks.length >= 20, `doc links ${docLinks.length}`);
for (const l of docLinks) {
  await go(l);
  ok(doc.querySelector("#view .md h1, #view .md h2"), `${l}: markdown rendered`);
  ok(!/<!--/.test(doc.getElementById("view").innerHTML), `${l}: comments stripped`);
}

// search and filters
await go("#/algorithms");
const q = doc.getElementById("q");
q.value = "fuzzy"; q.dispatchEvent(new window.Event("input"));
ok(doc.querySelectorAll("#res .card").length === 1, "search 'fuzzy' returns one notebook");
q.value = "zzzz-no-match"; q.dispatchEvent(new window.Event("input"));
ok(/No notebook matches/.test(doc.getElementById("res").textContent), "empty state shown");
q.value = ""; q.dispatchEvent(new window.Event("input"));
const diff = doc.getElementById("diff");
diff.value = "Advanced"; diff.dispatchEvent(new window.Event("change"));
const adv = doc.querySelectorAll("#res .card").length;
ok(adv > 0 && adv < 12, `difficulty filter narrows list (${adv})`);
diff.value = ""; diff.dispatchEvent(new window.Event("change"));
const cat = doc.getElementById("cat");
cat.value = "Motor and drive control"; cat.dispatchEvent(new window.Event("change"));
ok(doc.querySelectorAll("#res .card").length === 4, "category filter returns 4 motor notebooks");

// charts come from data, not constants: the iae bar widths follow the data
await go("#/algorithms/12_mpc");
ok(doc.querySelectorAll("#view .bars .bar").length > 3, "bar chart rows rendered from metrics");
await go("#/datasets");
ok(doc.querySelector("#view svg polyline"), "dataset preview chart drawn from CSV");

// no network or script errors
assert.deepEqual(errors, [], "jsdom errors: " + errors.join("; ")); passed++;
console.log(`smoke ok: ${passed} assertions`);
window.close();
