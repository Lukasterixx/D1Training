/* SVG line charts for training curves.
 * 2px lines on a hairline grid, one y-axis, a crosshair tooltip that lists every
 * series at the hovered step, a legend for two or more series, and a table view
 * so no value is reachable only by hovering. Labels are set with textContent. */
(function () {
  "use strict";
  const NS = "http://www.w3.org/2000/svg";
  const PLOT_H = 168, TOP = 10, BOTTOM = 26, RIGHT = 14;

  function svg(tag, attrs, parent) {
    const node = document.createElementNS(NS, tag);
    for (const [key, value] of Object.entries(attrs || {})) node.setAttribute(key, value);
    if (parent) parent.appendChild(node);
    return node;
  }

  function el(tag, cls, parent, text) {
    const node = document.createElement(tag);
    if (cls) node.className = cls;
    if (text !== undefined) node.textContent = text;
    if (parent) parent.appendChild(node);
    return node;
  }

  function niceStep(span, count) {
    const raw = span / Math.max(count, 1);
    const magnitude = Math.pow(10, Math.floor(Math.log10(raw)));
    const norm = raw / magnitude;
    return (norm <= 1 ? 1 : norm <= 2 ? 2 : norm <= 5 ? 5 : 10) * magnitude;
  }

  function niceDomain(min, max) {
    if (min === max) {
      const pad = Math.abs(min) * 0.1 || 1;
      min -= pad; max += pad;
    }
    const step = niceStep(max - min, 4);
    return [Math.floor(min / step) * step, Math.ceil(max / step) * step, step];
  }

  function ticks(min, max, step) {
    const out = [];
    for (let v = Math.ceil(min / step - 1e-9) * step; v <= max + step * 1e-6; v += step) {
      out.push(Math.abs(v) < step * 1e-6 ? 0 : v);
    }
    return out;
  }

  /** Axis tick text: compact and clean. */
  function formatTick(v) {
    const a = Math.abs(v);
    if (a >= 1e9) return v.toExponential(0).replace("e+", "e");
    if (a >= 1e6) return +(v / 1e6).toFixed(1) + "M";
    if (a >= 1e4) return +(v / 1e3).toFixed(1) + "K";
    if (a >= 100) return Math.round(v).toLocaleString();
    if (a === 0) return "0";
    return String(+v.toPrecision(3));
  }

  /** Tooltip/table text: enough digits to compare runs. */
  function formatValue(v) {
    if (!isFinite(v)) return String(v);
    const a = Math.abs(v);
    if (a >= 1e9) return v.toExponential(3);
    if (a >= 1e5) return Math.round(v).toLocaleString();
    if (a >= 100) return v.toLocaleString(undefined, { maximumFractionDigits: 1 });
    if (a === 0) return "0";
    return String(+v.toPrecision(4));
  }

  function nearestIndex(points, x) {
    let lo = 0, hi = points.length - 1;
    while (hi - lo > 1) {
      const mid = (lo + hi) >> 1;
      if (points[mid][0] < x) lo = mid; else hi = mid;
    }
    return Math.abs(points[lo][0] - x) <= Math.abs(points[hi][0] - x) ? lo : hi;
  }

  /** Min/max per pixel bucket keeps spikes that plain striding would drop. */
  function decimate(points, buckets) {
    if (points.length <= buckets * 2) return points;
    const out = [points[0]];
    const size = points.length / buckets;
    for (let b = 0; b < buckets; b++) {
      const start = Math.floor(b * size), end = Math.min(points.length, Math.floor((b + 1) * size));
      let lo = start, hi = start;
      for (let i = start; i < end; i++) {
        if (points[i][1] < points[lo][1]) lo = i;
        if (points[i][1] > points[hi][1]) hi = i;
      }
      if (lo === hi) out.push(points[lo]);
      else out.push(points[Math.min(lo, hi)], points[Math.max(lo, hi)]);
    }
    out.push(points[points.length - 1]);
    return out;
  }

  /**
   * opts.title, opts.subtitle, opts.xLabel ("iteration"),
   * opts.series: [{ name, color, points: [[x, y], ...] sorted by x }]
   */
  function lineChart(host, opts) {
    const series = opts.series.filter((s) => s.points && s.points.length);
    const xLabel = opts.xLabel || "iteration";
    const figure = el("figure", "chart card", host);
    const head = el("div", "chart-head", figure);
    el("div", "chart-title", head, opts.title);
    if (opts.subtitle) el("div", "chart-sub", head, opts.subtitle);
    if (series.length > 1) {
      const legend = el("div", "legend", figure);
      for (const s of series) {
        const item = el("span", "", legend);
        el("i", "", item).style.background = s.color;
        item.appendChild(document.createTextNode(s.name));
      }
    }
    const plot = el("div", "plot", figure);
    plot.tabIndex = 0;
    plot.setAttribute("role", "img");
    plot.setAttribute("aria-label", `${opts.title}. ` + series.map((s) => {
      const last = s.points[s.points.length - 1];
      return `${s.name}: ${formatValue(last[1])} at ${xLabel} ${last[0]}`;
    }).join("; ") + ". Use the arrow keys to read values, or show the table.");
    const tooltip = el("div", "tooltip", plot);
    tooltip.hidden = true;
    const foot = el("div", "chart-foot", figure);
    const toggle = el("button", "link-button", foot, "Show table");
    toggle.type = "button";
    const tableBox = el("div", "chart-table", figure);
    tableBox.hidden = true;
    toggle.addEventListener("click", () => {
      if (!tableBox.firstChild) buildTable();
      tableBox.hidden = !tableBox.hidden;
      toggle.textContent = tableBox.hidden ? "Show table" : "Hide table";
    });

    if (!series.length) {
      el("div", "hint", plot, "No data.");
      return figure;
    }

    const xs = [...new Set(series.flatMap((s) => s.points.map((p) => p[0])))].sort((a, b) => a - b);
    const xPoints = xs.map((x) => [x]); // nearestIndex() reads [x, ...] pairs.
    let state = null, lastWidth = 0, cursor = -1;

    function draw() {
      const width = Math.floor(plot.clientWidth);
      if (!width || width === lastWidth) return;
      lastWidth = width;
      const old = plot.querySelector("svg");
      if (old) old.remove();

      let xmin = Infinity, xmax = -Infinity, ymin = Infinity, ymax = -Infinity;
      for (const s of series) {
        xmin = Math.min(xmin, s.points[0][0]);
        xmax = Math.max(xmax, s.points[s.points.length - 1][0]);
        for (const p of s.points) {
          if (isFinite(p[1])) { ymin = Math.min(ymin, p[1]); ymax = Math.max(ymax, p[1]); }
        }
      }
      if (!isFinite(ymin)) { ymin = 0; ymax = 1; }
      if (xmin === xmax) { xmin -= 1; xmax += 1; }
      const [y0, y1, yStep] = niceDomain(ymin, ymax);
      const yTicks = ticks(y0, y1, yStep);
      const left = Math.ceil(Math.max(...yTicks.map((t) => formatTick(t).length)) * 6.6 + 12);
      const plotW = Math.max(40, width - left - RIGHT);
      const height = TOP + PLOT_H + BOTTOM;
      const sx = (x) => left + ((x - xmin) / (xmax - xmin)) * plotW;
      const sy = (y) => TOP + (1 - (y - y0) / (y1 - y0)) * PLOT_H;

      const root = svg("svg", { viewBox: `0 0 ${width} ${height}`, height, "aria-hidden": "true" });
      plot.insertBefore(root, tooltip);
      for (const t of yTicks) {
        const y = Math.round(sy(t)) + 0.5;
        svg("line", { class: "grid", x1: left, x2: left + plotW, y1: y, y2: y }, root);
        svg("text", { x: left - 8, y: y + 4, "text-anchor": "end" }, root).textContent = formatTick(t);
      }
      const baseline = Math.round(TOP + PLOT_H) + 0.5;
      svg("line", { class: "axis", x1: left, x2: left + plotW, y1: baseline, y2: baseline }, root);
      let xStep = niceStep(xmax - xmin, Math.max(2, Math.floor(plotW / 90)));
      if (xs.every(Number.isInteger)) xStep = Math.max(1, Math.round(xStep));
      for (const t of ticks(xmin, xmax, xStep)) {
        const x = sx(t);
        svg("line", { class: "axis", x1: x, x2: x, y1: baseline, y2: baseline + 4 }, root);
        svg("text", { x, y: baseline + 17, "text-anchor": "middle" }, root).textContent = formatTick(t);
      }

      for (const s of series) {
        const pts = decimate(s.points.filter((p) => isFinite(p[1])), plotW);
        if (pts.length > 1) {
          const d = pts.map((p, i) => `${i ? "L" : "M"}${sx(p[0]).toFixed(1)},${sy(p[1]).toFixed(1)}`).join("");
          svg("path", { class: "series-line", d, style: `stroke:${s.color}` }, root);
        }
        const last = s.points[s.points.length - 1];
        svg("circle", { class: "end-dot", cx: sx(last[0]), cy: sy(last[1]), r: 4, style: `fill:${s.color}` }, root);
        if (series.length === 1) {
          const label = svg("text", { class: "end-label", x: sx(last[0]) - 8, y: sy(last[1]) - 9, "text-anchor": "end" }, root);
          label.textContent = formatValue(last[1]);
        }
      }
      const crosshair = svg("line", { class: "crosshair", y1: TOP, y2: TOP + PLOT_H, visibility: "hidden" }, root);
      const dots = series.map((s) => svg("circle", { class: "hover-dot", r: 4, style: `fill:${s.color}`, visibility: "hidden" }, root));
      state = { sx, sy, left, plotW, xmin, xmax, width, crosshair, dots };
      if (cursor >= 0) showIndex(cursor);
    }

    function hide() {
      cursor = -1;
      tooltip.hidden = true;
      if (!state) return;
      state.crosshair.setAttribute("visibility", "hidden");
      state.dots.forEach((d) => d.setAttribute("visibility", "hidden"));
    }

    function showIndex(index) {
      if (!state) return;
      cursor = index;
      const x = xs[index];
      const cx = state.sx(x);
      const tolerance = ((state.xmax - state.xmin) / state.plotW) * 4;
      state.crosshair.setAttribute("x1", cx);
      state.crosshair.setAttribute("x2", cx);
      state.crosshair.setAttribute("visibility", "visible");
      tooltip.replaceChildren();
      el("div", "tt-head", tooltip, `${xLabel} ${x.toLocaleString()}`);
      series.forEach((s, k) => {
        const p = s.points[nearestIndex(s.points, x)];
        const dot = state.dots[k];
        if (Math.abs(p[0] - x) > tolerance) {
          dot.setAttribute("visibility", "hidden");
          return;
        }
        dot.setAttribute("cx", state.sx(p[0]));
        dot.setAttribute("cy", state.sy(p[1]));
        dot.setAttribute("visibility", "visible");
        const row = el("div", "tt-row", tooltip);
        el("i", "", row).style.background = s.color;
        el("b", "", row, formatValue(p[1]));
        if (series.length > 1) el("span", "", row, s.name);
      });
      tooltip.hidden = false;
      const tipW = tooltip.offsetWidth;
      let leftPx = cx + 14;
      if (leftPx + tipW > state.width) leftPx = Math.max(0, cx - 14 - tipW);
      tooltip.style.left = `${leftPx}px`;
      tooltip.style.top = "4px";
    }

    function showAtPixel(px) {
      if (!state || px < state.left - 6 || px > state.left + state.plotW + 6) return hide();
      const dataX = state.xmin + ((px - state.left) / state.plotW) * (state.xmax - state.xmin);
      showIndex(nearestIndex(xPoints, dataX));
    }

    plot.addEventListener("pointermove", (e) => showAtPixel(e.clientX - plot.getBoundingClientRect().left));
    plot.addEventListener("pointerleave", hide);
    plot.addEventListener("focus", () => showIndex(xs.length - 1));
    plot.addEventListener("blur", hide);
    plot.addEventListener("keydown", (e) => {
      const moves = { ArrowLeft: -1, ArrowRight: 1, PageDown: -10, PageUp: 10 };
      if (e.key in moves) showIndex(Math.min(xs.length - 1, Math.max(0, (cursor < 0 ? xs.length - 1 : cursor) + moves[e.key])));
      else if (e.key === "Home") showIndex(0);
      else if (e.key === "End") showIndex(xs.length - 1);
      else if (e.key === "Escape") hide();
      else return;
      e.preventDefault();
    });

    function buildTable() {
      const maxRows = 150;
      const stride = Math.max(1, Math.ceil(xs.length / maxRows));
      const rows = xs.filter((_, i) => i % stride === 0 || i === xs.length - 1);
      const lookups = series.map((s) => new Map(s.points.map((p) => [p[0], p[1]])));
      const table = el("table", "", tableBox);
      const headRow = el("tr", "", el("thead", "", table));
      el("th", "", headRow, xLabel);
      series.forEach((s) => el("th", "", headRow, series.length > 1 ? s.name : opts.title));
      const body = el("tbody", "", table);
      for (const x of rows) {
        const tr = el("tr", "", body);
        el("td", "", tr, x.toLocaleString());
        lookups.forEach((m) => el("td", "", tr, m.has(x) ? formatValue(m.get(x)) : ""));
      }
      if (stride > 1) el("div", "hint", tableBox, `Every ${stride}th ${xLabel} of ${xs.length}; the CSV holds all of them.`);
    }

    new ResizeObserver(() => requestAnimationFrame(draw)).observe(plot);
    return figure;
  }

  window.LineChart = { lineChart, formatValue };
})();
