/* Thesis B record: hash-routed views over the local JSON API.
 * Markdown arrives as escaped, server-rendered HTML; every other string from the
 * API (run names, tags, captions) is inserted with textContent. */
(function () {
  "use strict";
  const view = document.getElementById("view");
  const refreshLabel = document.getElementById("refresh");
  const SERIES = [1, 2, 3, 4, 5, 6, 7, 8].map((i) => `var(--s${i})`);
  const GROUP_ORDER = ["Metrics", "Train", "Episode_Termination", "Episode_Reward", "Loss", "Policy", "Perf", "Curriculum"];
  const OPEN_GROUPS = new Set(["Metrics", "Train", "Episode_Termination"]);
  const STATUS = {
    "passed": ["good", "✓"], "confirmed": ["good", "✓"], "complete": ["good", "✓"], "done": ["good", "✓"],
    "smoke_finished": ["good", "✓"], "training_finished": ["good", "✓"], "finished": ["good", "✓"],
    "in progress": ["progress", "◐"], "running": ["progress", "◐"], "initializing": ["progress", "◐"],
    "provisional": ["warning", "!"], "partial": ["warning", "!"], "at risk": ["warning", "!"],
    "blocked": ["serious", "■"], "failed": ["critical", "✕"],
    "retracted": ["neutral", "–"], "superseded": ["neutral", "–"],
    "not started": ["neutral", ""], "planned": ["neutral", ""], "unknown": ["neutral", "?"],
  };
  let overview = null;
  let stamp = null;
  const openRuns = new Set();

  // Helpers ------------------------------------------------------------------
  function h(tag, props, ...children) {
    const node = document.createElement(tag);
    for (const [key, value] of Object.entries(props || {})) {
      if (value === undefined || value === null || value === false) continue;
      if (key === "class") node.className = value;
      else if (key === "text") node.textContent = value;
      else if (key === "html") node.innerHTML = value; // Server-escaped Markdown only.
      else if (key.startsWith("on")) node.addEventListener(key.slice(2), value);
      else node.setAttribute(key, value);
    }
    for (const child of children.flat()) {
      if (child === null || child === undefined || child === false) continue;
      node.appendChild(typeof child === "string" ? document.createTextNode(child) : child);
    }
    return node;
  }

  async function api(path) {
    const response = await fetch(path, { cache: "no-store" });
    const data = await response.json();
    if (!response.ok) throw new Error(data.error || response.statusText);
    return data;
  }

  function badge(status) {
    const key = String(status || "unknown").toLowerCase().trim();
    const [tone, icon] = STATUS[key] || ["neutral", "·"];
    return h("span", { class: `badge ${tone}` }, h("span", { class: "badge-icon", "aria-hidden": "true", text: icon }), key.replace(/_/g, " "));
  }

  function fmt(value, digits = 3) {
    if (typeof value !== "number") return value === null || value === undefined ? "—" : String(value);
    if (Number.isInteger(value)) return value.toLocaleString();
    return window.LineChart.formatValue(+value.toPrecision(digits + 1));
  }

  function timeAgo(seconds) {
    const delta = Date.now() / 1000 - seconds;
    if (delta < 90) return "just now";
    if (delta < 5400) return `${Math.round(delta / 60)} min ago`;
    if (delta < 129600) return `${Math.round(delta / 3600)} h ago`;
    return new Date(seconds * 1000).toLocaleDateString(undefined, { day: "numeric", month: "short" });
  }

  function section(id, title, count, ...children) {
    return h("section", { class: "section", id },
      h("h2", {}, title, count !== undefined ? h("span", { class: "count", text: String(count) }) : null), ...children);
  }

  function prose(html) {
    return h("div", { class: "card prose-card" }, h("div", { class: "prose", html }));
  }

  function scalarLabel(tag) {
    const labels = (overview && overview.scalar_labels) || {};
    return labels[tag] || tag;
  }

  // Post-render decoration ----------------------------------------------------
  function decorate(root) {
    root.querySelectorAll(".prose td").forEach((td) => {
      const key = td.textContent.trim().toLowerCase();
      if (STATUS[key] && td.children.length === 0) td.replaceChildren(badge(key));
    });
    root.querySelectorAll(".prose strong").forEach((strong) => {
      if (!/^status:?$/i.test(strong.textContent.trim())) return;
      const next = strong.nextSibling;
      if (!next || next.nodeType !== Node.TEXT_NODE) return;
      const raw = next.textContent;
      const text = raw.trimStart();
      const key = Object.keys(STATUS).sort((a, b) => b.length - a.length)
        .find((k) => text.toLowerCase().startsWith(k) && !/[a-z]/i.test(text.charAt(k.length)));
      if (!key) return;
      // Keep whatever follows the status word, including the line break before the next field.
      next.replaceWith(document.createTextNode(" "), badge(key), document.createTextNode(text.slice(key.length)));
    });
    root.querySelectorAll(".prose img").forEach((img) => {
      img.addEventListener("click", () => openLightbox({ kind: "image", url: img.src, caption: img.alt }));
    });
    root.querySelectorAll('a[href^="#"]:not([href^="#/"])').forEach((a) => {
      a.addEventListener("click", (event) => {
        const target = root.querySelector(`[id="${CSS.escape(a.getAttribute("href").slice(1))}"]`);
        if (target) { event.preventDefault(); target.scrollIntoView({ behavior: "smooth", block: "start" }); }
      });
    });
  }

  // Lightbox ------------------------------------------------------------------
  const lightbox = document.getElementById("lightbox");
  function openLightbox(item) {
    const holder = lightbox.querySelector(".lightbox-media");
    holder.replaceChildren(item.kind === "video"
      ? h("video", { src: item.url, controls: true, autoplay: true })
      : h("img", { src: item.url, alt: item.caption || "" }));
    lightbox.querySelector("figcaption").textContent = item.caption || "";
    lightbox.hidden = false;
    lightbox.querySelector(".lightbox-close").focus();
  }
  function closeLightbox() {
    lightbox.hidden = true;
    lightbox.querySelector(".lightbox-media").replaceChildren();
  }
  lightbox.addEventListener("click", (e) => { if (e.target === lightbox || e.target.closest(".lightbox-close")) closeLightbox(); });
  document.addEventListener("keydown", (e) => { if (e.key === "Escape" && !lightbox.hidden) closeLightbox(); });

  function gallery(items, emptyMessage) {
    if (!items.length) return h("div", { class: "empty" }, ...emptyMessage);
    return h("div", { class: "gallery" }, items.map((item) => h("figure", { class: "card shot" },
      h("button", { type: "button", "aria-label": `Open ${item.caption}`, onclick: () => openLightbox(item) },
        item.kind === "video" ? h("video", { src: item.url, muted: true, preload: "metadata" })
          : h("img", { src: item.url, alt: item.caption, loading: "lazy" })),
      h("figcaption", { title: item.path, text: item.caption }))));
  }

  // Charts --------------------------------------------------------------------
  function groupedCharts(tags) {
    const groups = new Map();
    for (const tag of Object.keys(tags).sort()) {
      const cut = tag.indexOf("/");
      const group = cut < 0 ? "Other" : tag.slice(0, cut);
      if (!groups.has(group)) groups.set(group, []);
      groups.get(group).push(tag);
    }
    const rank = (g) => (GROUP_ORDER.includes(g) ? GROUP_ORDER.indexOf(g) : GROUP_ORDER.length);
    const ordered = [...groups.keys()].sort((a, b) => rank(a) - rank(b) || a.localeCompare(b));
    if (!ordered.length) return h("p", { class: "hint", text: "No scalars logged yet." });
    return h("div", {}, ordered.map((group) => {
      const grid = h("div", { class: "chart-grid" });
      for (const tag of groups.get(group)) {
        const title = overview.scalar_labels[tag] || tag.slice(group.length + 1) || tag;
        window.LineChart.lineChart(grid, { title, subtitle: overview.scalar_labels[tag] ? tag : undefined,
          series: [{ name: tag, color: SERIES[0], points: tags[tag] }] });
      }
      return h("details", { open: OPEN_GROUPS.has(group) },
        h("summary", { class: "chart-group", text: `${group.replace(/_/g, " ")} · ${groups.get(group).length}` }), grid);
    }));
  }

  // Navigation ----------------------------------------------------------------
  function renderNav(hash) {
    const active = (href) => hash === href || (href !== "#/" && href !== "#/week" && hash.startsWith(href + "/"));
    const link = (href, label, sub, dot) => h("a", { href, class: active(href) ? "active" : "" },
      dot || null, label, sub ? h("span", { class: "nav-sub", text: sub }) : null);
    const findings = overview.findings.length;
    document.getElementById("nav-main").replaceChildren(
      link("#/", "Overview"),
      link("#/findings", "Findings", findings ? String(findings) : ""),
      link("#/live", "Live runs"));
    document.getElementById("nav-weeks").replaceChildren(...overview.weeks.map((w) => {
      const tone = (STATUS[w.status] || ["neutral"])[0];
      const dot = h("span", { class: `nav-dot ${tone}${w.current ? " now" : ""}`, "aria-hidden": "true" });
      return link(`#/week/${w.n}`, `Week ${w.n}`, w.dates.replace(/ \d{4}$/, ""), dot);
    }));
    document.getElementById("nav-docs").replaceChildren(...overview.docs.map((d) =>
      link(`#/doc/${d.path}`, d.title.length > 34 ? d.title.slice(0, 32) + "…" : d.title)));
  }

  // Views ---------------------------------------------------------------------
  function overviewView() {
    const d = overview;
    const current = d.weeks.find((w) => w.current);
    const runs = d.weeks.reduce((sum, w) => sum + w.runs, 0);
    const shots = d.weeks.reduce((sum, w) => sum + w.screenshots, 0);
    const byStatus = (s) => d.findings.filter((f) => f.status.startsWith(s)).length;
    const gatesPassed = d.gates.filter((g) => g.status === "passed").length;
    const tile = (label, value, note) => h("div", { class: "card tile" },
      h("div", { class: "tile-label", text: label }), h("div", { class: "tile-value", text: String(value) }),
      note ? h("div", { class: "tile-note", text: note }) : null);

    return h("div", {},
      h("p", { class: "eyebrow", text: d.term }),
      h("h1", { class: "page-title", text: d.title }),
      h("div", { class: "hero" },
        current
          ? h("a", { href: `#/week/${current.n}`, class: "hero-figure", style: "color:inherit" }, `Week ${current.n}`, h("small", { text: `of ${d.weeks.length} · ${current.dates}` }))
          : h("div", { class: "hero-figure" }, d.today < d.weeks[0].start ? "Before term" : "Term complete")),
      current && current.focus ? h("p", { class: "page-sub", text: current.focus }) : null,
      h("div", { class: "tiles" },
        tile("Runs recorded", runs, "copied into results/"),
        tile("Findings", d.findings.length, `${byStatus("confirmed")} confirmed · ${byStatus("provisional")} provisional`),
        tile("Gates passed", `${gatesPassed} of ${d.gates.length}`),
        tile("Screenshots", shots)),
      section("gates", "Decision gates", undefined, prose(d.gates_html)),
      section("findings", "Findings", d.findings.length,
        d.findings.length ? h("div", { class: "card table-wrap" }, h("table", {},
          h("thead", {}, h("tr", {}, ["ID", "Finding", "Status", "Week"].map((t) => h("th", { text: t })))),
          h("tbody", {}, d.findings.slice().reverse().map((f) => h("tr", {},
            h("td", { class: "run-id", text: f.id }),
            h("td", {}, h("a", { href: `#/findings/${f.anchor}`, text: f.title })),
            h("td", {}, badge(f.status)), h("td", { text: f.week })))))) : h("div", { class: "empty", text: "No findings yet." })),
      section("weeks", "Weeks", undefined, h("div", { class: "weeks-grid" }, d.weeks.map((w) =>
        h("a", { href: `#/week/${w.n}`, class: `card week-card${w.current ? " current" : ""}` },
          h("div", { class: "week-card-head" }, h("h3", { text: `Week ${w.n}` }), h("span", { class: "dates", text: w.dates })),
          h("div", { class: "focus", text: w.focus || "—" }),
          h("div", { class: "week-card-head" }, badge(w.status),
            h("span", { class: "meta", text: [w.runs && `${w.runs} runs`, w.screenshots && `${w.screenshots} shots`, w.figures && `${w.figures} figures`].filter(Boolean).join(" · ") })))))));
  }

  function keyResult(run) {
    const parts = [];
    for (const tag of overview.key_scalars) {
      const stats = (run.scalar_summary || {})[tag];
      if (stats && parts.length < 2) parts.push(`${scalarLabel(tag)} ${fmt(stats.mean_last_10)}`);
    }
    const smoke = run.smoke_summary;
    if (smoke) {
      if (smoke.failure_resets !== undefined) parts.push(`${smoke.failure_resets} failure / ${smoke.time_limit_resets} time-limit resets`);
      if (smoke.final_position_error_m) parts.push(`final error mean ${fmt(smoke.final_position_error_m.mean)} m`);
    }
    return parts.join(" · ") || "—";
  }

  function runDetail(run, week, scalars) {
    const pairs = [
      ["Started (UTC)", run.started_utc], ["Recorded", run.recorded_at], ["Source", run.source_dir],
      ["Mode", run.mode], ["Seed", run.seed], ["Environments", run.num_envs],
      ["Iterations requested", run.iterations], ["Last logged iteration", run.last_iteration], ["Smoke steps", run.mode === "smoke" ? run.steps : null],
      ["Articulation mass (kg)", run.articulation_mass_kg], ["Git commit", run.git_commit ? run.git_commit.slice(0, 12) + (run.git_dirty ? " (uncommitted changes)" : "") : null],
      ["Final checkpoint", run.final_checkpoint ? `${run.final_checkpoint.file} · sha256 ${run.final_checkpoint.sha256.slice(0, 12)}` : null],
    ];
    for (const [key, value] of Object.entries(run.smoke_summary || {})) {
      if (key === "interpretation") continue;
      pairs.push([`smoke: ${key}`, value && typeof value === "object" ? `mean ${fmt(value.mean)} · min ${fmt(value.min)} · max ${fmt(value.max)} (n=${value.count})` : value]);
    }
    return h("div", {},
      run.notes ? h("p", { text: run.notes }) : null,
      run.error ? h("p", { class: "error-text", text: run.error }) : null,
      run.smoke_summary && run.smoke_summary.interpretation ? h("p", { class: "hint", text: run.smoke_summary.interpretation }) : null,
      h("dl", { class: "kv" }, pairs.filter(([, v]) => v !== null && v !== undefined && v !== "").map(([k, v]) =>
        h("div", {}, h("dt", { text: k }), h("dd", { text: typeof v === "number" ? fmt(v, 4) : String(v) })))),
      h("div", { class: "files" }, h("span", { class: "hint", text: "Files:" }), run.files.map((f) => h("a", { href: f.url, target: "_blank", text: f.name }))),
      scalars ? groupedCharts(scalars) : null);
  }

  async function weekView(n) {
    const w = await api(`/api/week/${n}`);
    const scalars = {};
    await Promise.all(w.run_list.filter((r) => r.has_scalars).map(async (r) => {
      scalars[r.id] = (await api(`/api/scalars?week=${n}&run=${encodeURIComponent(r.id)}`)).tags;
    }));

    const columns = ["", "Run", "Mode", "Seed", "Envs", "Length", "Status", "Key result", "Commit"];
    const rows = w.run_list.flatMap((run) => {
      const detail = h("tr", { class: "run-detail", hidden: !openRuns.has(run.id) }, h("td", { colspan: columns.length }));
      let filled = false;
      const fill = () => { if (!filled) { detail.firstChild.appendChild(runDetail(run, n, scalars[run.id])); filled = true; } };
      if (openRuns.has(run.id)) fill();
      const toggle = () => {
        fill();
        detail.hidden = !detail.hidden;
        row.classList.toggle("open", !detail.hidden);
        row.setAttribute("aria-expanded", String(!detail.hidden));
        if (detail.hidden) openRuns.delete(run.id); else openRuns.add(run.id);
      };
      const length = run.mode === "smoke" ? `${fmt(run.steps)} steps` : run.last_iteration !== null && run.last_iteration !== undefined ? `${fmt(run.last_iteration + 1)} it` : run.iterations ? `${fmt(run.iterations)} it` : "—";
      const row = h("tr", { id: `run-${run.id}`, class: `run-row${openRuns.has(run.id) ? " open" : ""}`, tabindex: 0, "aria-expanded": String(openRuns.has(run.id)),
        onclick: toggle, onkeydown: (e) => { if (e.key === "Enter" || e.key === " ") { e.preventDefault(); toggle(); } } },
        h("td", {}, h("span", { class: "caret", text: "›" })),
        h("td", {}, h("div", { class: "run-title", text: run.title }), h("div", { class: "run-id", text: run.id })),
        h("td", { text: run.mode || "—" }), h("td", { class: "num", text: fmt(run.seed) }),
        h("td", { class: "num", text: fmt(run.num_envs) }), h("td", { class: "num", text: length }),
        h("td", {}, badge(run.status)), h("td", { text: keyResult(run) }),
        h("td", { class: "run-id", text: run.git_commit ? run.git_commit.slice(0, 7) + (run.git_dirty ? "*" : "") : "—",
          title: run.git_dirty ? "Launched with uncommitted changes" : null }));
      return [row, detail];
    });

    const withScalars = w.run_list.filter((r) => scalars[r.id]).slice(0, 8);
    const curves = h("div", { class: "chart-grid" });
    for (const tag of overview.key_scalars) {
      const series = withScalars.map((run, i) => ({ name: run.title, color: SERIES[i], points: scalars[run.id][tag] }))
        .filter((s) => s.points && s.points.length);
      if (series.length) window.LineChart.lineChart(curves, { title: scalarLabel(tag), subtitle: scalarLabel(tag) !== tag ? tag : undefined, series });
    }
    const extra = w.run_list.filter((r) => scalars[r.id]).length - withScalars.length;

    const nav = (k, label) => (k >= 1 && k <= overview.weeks.length ? h("a", { href: `#/week/${k}`, text: label }) : h("span"));
    return h("div", {},
      h("p", { class: "eyebrow", text: `${overview.term} · Week ${n} of ${overview.weeks.length}` }),
      h("h1", { class: "page-title", text: `Week ${n}` }),
      h("div", { class: "page-sub" }, h("span", { text: w.dates }), badge(w.status), w.current ? h("span", { class: "chip", text: "This week" }) : null,
        h("a", { class: "hint", href: `/files/${w.notes_path}`, target: "_blank", text: w.notes_path })),
      w.focus ? h("p", { class: "focus-line", text: w.focus }) : null,
      h("nav", { class: "jump", "aria-label": "Sections" },
        [["notes", "Notes"], ["runs", `Runs · ${w.run_list.length}`], ["curves", "Curves"], ["figures", `Figures · ${w.figure_list.length}`], ["screenshots", `Screenshots · ${w.screenshot_list.length}`]]
          .map(([id, label]) => h("a", { href: `#${id}`, text: label }))),
      section("notes", "Notes", undefined, prose(w.notes_html || "<p>No notes yet.</p>")),
      section("runs", "Runs", w.run_list.length || undefined, w.run_list.length
        ? h("div", { class: "card table-wrap" }, h("table", { class: "runs-table" },
          h("thead", {}, h("tr", {}, columns.map((c) => h("th", { text: c })))), h("tbody", {}, rows)))
        : h("div", { class: "empty" }, "No runs recorded this week. After a run, record it with ", h("code", { text: "./dashboard.py record logs/position_only/<run>" }), ".")),
      section("curves", "Training curves", undefined,
        curves.childElementCount ? curves : h("div", { class: "empty", text: "Curves appear once a recorded run has logged scalars." }),
        extra > 0 ? h("p", { class: "hint", text: `Overlays show the first 8 runs; ${extra} more are under their own rows above.` }) : null),
      section("figures", "Figures", w.figure_list.length || undefined,
        gallery(w.figure_list, ["Generated plots go in ", h("code", { text: `${w.folder}/figures/` }), "."])),
      section("screenshots", "Screenshots", w.screenshot_list.length || undefined,
        gallery(w.screenshot_list, ["None this week. Drop PNG, JPG or MP4 files into ", h("code", { text: `${w.folder}/screenshots/` }), " and they appear here."])),
      h("div", { class: "pager" }, nav(n - 1, `← Week ${n - 1}`), nav(n + 1, `Week ${n + 1} →`)));
  }

  async function findingsView(anchor) {
    const data = await api("/api/findings");
    return { node: h("div", {}, h("p", { class: "eyebrow", text: "results/findings.md" }), prose(data.html)), anchor };
  }

  async function docView(path) {
    const data = await api(`/api/doc?path=${encodeURIComponent(path)}`);
    return h("div", {}, h("p", { class: "eyebrow" }, h("a", { href: `/files/${path}`, target: "_blank", text: path })), prose(data.html));
  }

  async function liveView() {
    const data = await api("/api/live");
    const body = data.runs.map((run) => h("tr", {},
      h("td", {}, h("a", { href: `#/live/${run.path}`, text: run.id })),
      h("td", { text: run.mode || "—" }), h("td", { class: "num", text: fmt(run.seed) }), h("td", { class: "num", text: fmt(run.num_envs) }),
      h("td", {}, badge(run.status)), h("td", { text: timeAgo(run.updated) }),
      h("td", {}, run.recorded_week ? h("a", { href: `#/week/${run.recorded_week}`, text: `Week ${run.recorded_week}` }) : h("span", { class: "hint", text: "not recorded" }))));
    return h("div", {},
      h("p", { class: "eyebrow", text: (overview.log_roots || []).join(", ") }),
      h("h1", { class: "page-title", text: "Live runs" }),
      h("p", { class: "page-sub", text: "Runs in the log folders, read straight from their TensorBoard files. Record a run to copy it into its week as evidence." }),
      data.runs.length ? h("div", { class: "card table-wrap" }, h("table", {},
        h("thead", {}, h("tr", {}, ["Run", "Mode", "Seed", "Envs", "Status", "Updated", "Record"].map((t) => h("th", { text: t })))),
        h("tbody", {}, body))) : h("div", { class: "empty", text: "No runs in the log folders yet." }));
  }

  async function liveRunView(path) {
    const [list, data] = await Promise.all([api("/api/live"), api(`/api/scalars?live=${encodeURIComponent(path)}`).catch(() => ({ tags: {} }))]);
    const run = list.runs.find((r) => r.path === path) || { id: path, status: "unknown" };
    return h("div", {},
      h("p", { class: "eyebrow" }, h("a", { href: "#/live", text: "Live runs" }), ` / ${path}`),
      h("h1", { class: "page-title", text: run.id }),
      h("div", { class: "page-sub" }, badge(run.status), run.updated ? h("span", { text: `updated ${timeAgo(run.updated)}` }) : null,
        run.recorded_week ? h("a", { href: `#/week/${run.recorded_week}`, text: `Recorded in week ${run.recorded_week}` }) : null),
      run.error ? h("p", { class: "error-text", text: run.error }) : null,
      run.recorded_week ? null : h("p", { class: "hint" }, "Record it: ", h("code", { text: `./dashboard.py record ${path} --title "…"` })),
      groupedCharts(data.tags));
  }

  // Router --------------------------------------------------------------------
  async function route(soft) {
    const hash = location.hash || "#/";
    const scroll = soft ? window.scrollY : 0;
    view.classList.add("stale");
    try {
      overview = await api("/api/overview");
      renderNav(hash);
      let match, result;
      if ((match = hash.match(/^#\/week\/(\d+)(?:\/run\/(.+))?$/))) {
        const runId = match[2] && decodeURIComponent(match[2]);
        if (runId && !soft) openRuns.add(runId);
        result = { node: await weekView(Number(match[1])), anchor: runId && `run-${runId}` };
      }
      else if ((match = hash.match(/^#\/findings(?:\/(.+))?$/))) result = await findingsView(match[1]);
      else if ((match = hash.match(/^#\/doc\/(.+)$/))) result = await docView(decodeURIComponent(match[1]));
      else if (hash === "#/live") result = await liveView();
      else if ((match = hash.match(/^#\/live\/(.+)$/))) result = await liveRunView(decodeURIComponent(match[1]));
      else result = overviewView();
      const node = result.node || result;
      view.replaceChildren(node);
      decorate(node);
      document.title = `${document.querySelector("h1")?.textContent || "Record"} · Thesis B record`;
      const target = result.anchor && node.querySelector(`[id="${CSS.escape(result.anchor)}"]`);
      if (target && !soft) target.scrollIntoView();
      else window.scrollTo(0, scroll);
    } catch (error) {
      view.replaceChildren(h("div", { class: "empty" }, h("strong", { text: "Could not load this page. " }), String(error.message || error)));
    } finally {
      view.classList.remove("stale");
    }
  }

  async function poll() {
    if (document.hidden) return;
    try {
      const data = await api("/api/stamp");
      if (data.stamp !== stamp) {
        // Don't redraw under an active hover, keyboard read-out or open viewer; try next tick.
        if (document.querySelector(".plot:hover, .plot:focus") || !lightbox.hidden) return;
        stamp = data.stamp;
        await route(true);
      }
      refreshLabel.textContent = `Watching for changes · ${new Date().toLocaleTimeString([], { hour: "2-digit", minute: "2-digit" })}`;
    } catch (error) {
      refreshLabel.textContent = "Dashboard server is not responding.";
    }
  }

  window.addEventListener("hashchange", () => route(false));
  (async () => {
    try { stamp = (await api("/api/stamp")).stamp; } catch (error) { /* route() reports it */ }
    await route(false);
    setInterval(poll, 4000);
  })();
})();
