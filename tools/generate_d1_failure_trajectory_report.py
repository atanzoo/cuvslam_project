#!/usr/bin/env python3
"""Create an inline HTML top-view plot for saved D1 failure replays."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


def _compact_trace(trace: list[dict[str, Any]]) -> list[dict[str, Any]]:
    compact: list[dict[str, Any]] = []
    for point in trace:
        obstacles = [
            [
                round(float(obstacle["x"]), 3),
                round(float(obstacle["y"]), 3),
                int(bool(obstacle.get("active", True))),
            ]
            for obstacle in point.get("obstacles", [])
        ]
        compact.append({
            "t": round(float(point["t"]), 3),
            "r": [round(float(value), 3) for value in point["robot"]],
            "o": obstacles,
            "m": point.get("mode", "CRUISE"),
            "c": round(float(point.get("min_clearance", 0.0)), 3),
            "a": int(bool(point.get("recovery_active", False))),
        })
    return compact


def load_cases(
    failure_dir: Path,
    maximum_trial: int | None,
    include_success: bool = False,
) -> list[dict[str, Any]]:
    cases: list[dict[str, Any]] = []
    for path in sorted(failure_dir.glob("trial_*.json")):
        payload = json.loads(path.read_text(encoding="utf-8"))
        trial = int(payload["trial"])
        if maximum_trial is not None and trial >= maximum_trial:
            continue
        result = payload["result"]
        # The benchmark keeps replay files on disk between runs.  A trial
        # that was previously a failure may become successful after a policy
        # change, so do not show that stale success as a failure case unless
        # the caller explicitly asks for outcome comparison.
        if result.get("success", False) and not include_success:
            continue
        trace = result.get("trace", [])
        if not trace:
            continue
        cases.append({
            "trial": trial,
            "status": (
                "success"
                if result.get("success", False)
                else result.get("failure_reason", "failure")
            ),
            "recovery": bool(result.get("recovery_triggered", False)),
            "clearance": round(float(result["min_clearance"]), 3),
            "distance": round(float(result["distance_to_goal"]), 3),
            "path": round(float(result["path_length"]), 3),
            "goal": [
                round(float(trace[0]["goal"][0]), 3),
                round(float(trace[0]["goal"][1]), 3),
            ],
            "trace": _compact_trace(trace),
        })
    return sorted(cases, key=lambda case: int(case["trial"]))


def build_fragment(cases: list[dict[str, Any]]) -> str:
    data = json.dumps(cases, ensure_ascii=False, separators=(",", ":"))
    return f'''<div id="d1FailureTrajectories">
  <style>
    #d1FailureTrajectories {{
      --d1-fg: light-dark(#17212b, #e7edf3);
      --d1-muted: light-dark(#52606d, #9eacba);
      --d1-grid: light-dark(#d9e1e8, #34424f);
      --d1-bg: light-dark(#f8fafc, #111922);
      --d1-blue: light-dark(#1769aa, #65b5f6);
      --d1-red: light-dark(#c13b4a, #ff7a86);
      --d1-orange: light-dark(#c76b16, #f3ac5c);
      color: var(--d1-fg);
      font-family: system-ui, -apple-system, sans-serif;
      width: 100%;
    }}
    #d1FailureTrajectories .d1-controls {{
      display: flex;
      align-items: center;
      gap: 0.75rem;
      flex-wrap: wrap;
      margin-bottom: 0.5rem;
      font-size: 0.9rem;
    }}
    #d1FailureTrajectories select {{
      color: var(--d1-fg);
      background: var(--d1-bg);
      border: 1px solid var(--d1-grid);
      border-radius: 0.35rem;
      padding: 0.25rem 0.45rem;
    }}
    #d1FailureTrajectories canvas {{
      display: block;
      width: 100%;
      height: min(62vw, 500px);
      min-height: 330px;
      background: var(--d1-bg);
      border: 1px solid var(--d1-grid);
      border-radius: 0.5rem;
    }}
    #d1FailureTrajectories .d1-detail {{
      color: var(--d1-muted);
      margin-top: 0.45rem;
      font-size: 0.88rem;
    }}
  </style>
  <div class="d1-controls">
    <label for="d1FailureCase">查看案例</label>
    <select id="d1FailureCase"></select>
    <span>藍：recoverable timeout　紅：deadlock　橘：障礙物　虛線：脫困作用區間</span>
  </div>
  <canvas id="d1FailureCanvas" aria-label="D1 failure replay top-view trajectory"></canvas>
  <div id="d1FailureDetail" class="d1-detail"></div>
  <script>
    (() => {{
      const root = document.getElementById('d1FailureTrajectories');
      const cases = {data};
      const select = root.querySelector('#d1FailureCase');
      const canvas = root.querySelector('#d1FailureCanvas');
      const detail = root.querySelector('#d1FailureDetail');
      const css = getComputedStyle(root);
      const colors = {{
        fg: css.getPropertyValue('--d1-fg').trim(),
        muted: css.getPropertyValue('--d1-muted').trim(),
        grid: css.getPropertyValue('--d1-grid').trim(),
        bg: css.getPropertyValue('--d1-bg').trim(),
        blue: css.getPropertyValue('--d1-blue').trim(),
        green: css.getPropertyValue('--d1-green').trim(),
        red: css.getPropertyValue('--d1-red').trim(),
        orange: css.getPropertyValue('--d1-orange').trim(),
      }};
      const ctx = canvas.getContext('2d');
      let selected = 0;

      cases.forEach((item, index) => {{
        const option = document.createElement('option');
        option.value = String(index);
        option.textContent = `trial ${{item.trial}} · ${{item.status}}`;
        select.appendChild(option);
      }});

      function allPoints() {{
        const points = [];
        cases.forEach(item => item.trace.forEach(point => {{
          points.push(point.r);
          point.o.forEach(obstacle => points.push(obstacle));
        }}));
        cases.forEach(item => points.push(item.goal));
        return points;
      }}

      function draw() {{
        if (!cases.length) return;
        selected = Number(select.value || 0);
        const item = cases[selected];
        const rect = canvas.getBoundingClientRect();
        const dpr = window.devicePixelRatio || 1;
        const width = Math.max(320, rect.width);
        const height = Math.max(330, rect.height);
        canvas.width = Math.round(width * dpr);
        canvas.height = Math.round(height * dpr);
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
        ctx.clearRect(0, 0, width, height);

        const points = allPoints();
        const xs = points.map(point => point[0]);
        const ys = points.map(point => point[1]);
        const xmin = Math.min(...xs) - 0.35;
        const xmax = Math.max(...xs) + 0.35;
        const ymin = Math.min(...ys) - 0.35;
        const ymax = Math.max(...ys) + 0.35;
        const pad = {{left: 48, right: 18, top: 18, bottom: 34}};
        const sx = (width - pad.left - pad.right) / (xmax - xmin);
        const sy = (height - pad.top - pad.bottom) / (ymax - ymin);
        const map = point => [
          pad.left + (point[0] - xmin) * sx,
          height - pad.bottom - (point[1] - ymin) * sy,
        ];

        ctx.fillStyle = colors.bg;
        ctx.fillRect(0, 0, width, height);
        ctx.font = '11px system-ui, sans-serif';
        ctx.lineWidth = 1;
        ctx.strokeStyle = colors.grid;
        ctx.fillStyle = colors.muted;
        for (let x = Math.ceil(xmin); x <= Math.floor(xmax); x += 1) {{
          const a = map([x, ymin]); const b = map([x, ymax]);
          ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke();
          ctx.fillText(`${{x}} m`, a[0] - 9, height - 10);
        }}
        for (let y = Math.ceil(ymin); y <= Math.floor(ymax); y += 1) {{
          const a = map([xmin, y]); const b = map([xmax, y]);
          ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke();
          ctx.fillText(`${{y}}`, 13, a[1] + 4);
        }}

        function path(points, color, lineWidth, dashed = false) {{
          if (points.length < 2) return;
          ctx.save();
          ctx.strokeStyle = color;
          ctx.lineWidth = lineWidth;
          ctx.setLineDash(dashed ? [5, 4] : []);
          ctx.beginPath();
          points.forEach((point, index) => {{
            const p = map(point);
            if (index === 0) ctx.moveTo(p[0], p[1]); else ctx.lineTo(p[0], p[1]);
          }});
          ctx.stroke();
          ctx.restore();
        }}

        cases.forEach((other, index) => {{
          if (index === selected) return;
          path(other.trace.map(point => point.r),
            other.status === 'success' ? colors.green : colors.red, 1);
        }});
        path(item.trace.map(point => point.r),
          item.status === 'success' ? colors.green : colors.red, 3);

        const obstacleCount = item.trace.reduce((max, point) => Math.max(max, point.o.length), 0);
        for (let obstacleIndex = 0; obstacleIndex < obstacleCount; obstacleIndex += 1) {{
          const obstaclePath = item.trace
            .map(point => point.o[obstacleIndex])
            .filter(Boolean)
            .filter(point => point[2])
            .map(point => [point[0], point[1]]);
          path(obstaclePath, colors.orange, 2);
        }}

        const recoveryPoints = item.trace.filter(point => point.a).map(point => point.r);
        path(recoveryPoints, colors.red, 5, true);
        const start = map(item.trace[0].r);
        const end = map(item.trace[item.trace.length - 1].r);
        const goal = map(item.goal);
        ctx.fillStyle = colors.blue;
        ctx.beginPath(); ctx.arc(start[0], start[1], 5, 0, Math.PI * 2); ctx.fill();
        ctx.fillStyle = colors.red;
        ctx.beginPath(); ctx.arc(end[0], end[1], 5, 0, Math.PI * 2); ctx.fill();
        ctx.fillStyle = colors.fg;
        ctx.beginPath(); ctx.arc(goal[0], goal[1], 6, 0, Math.PI * 2); ctx.fill();
        ctx.fillText('start', start[0] + 7, start[1] - 7);
        ctx.fillText('end', end[0] + 7, end[1] - 7);
        ctx.fillText('goal', goal[0] + 7, goal[1] - 7);
        detail.textContent = `trial ${{item.trial}} · ${{item.status}} · `
          + `min clearance ${{item.clearance.toFixed(3)}} m · `
          + `remaining goal distance ${{item.distance.toFixed(3)}} m · `
          + `path ${{item.path.toFixed(3)}} m · `
          + `recovery ${{item.recovery ? 'triggered' : 'not triggered'}}`;
      }}

      select.addEventListener('change', draw);
      window.addEventListener('resize', draw);
      draw();
    }})();
  </script>
</div>
'''


def build_svg_fragment(cases: list[dict[str, Any]]) -> str:
    """Build the same report with SVG instead of canvas.

    SVG keeps the plot visible in hosts that do not provide a reliable canvas
    layout or canvas color initialization.
    """
    data = json.dumps(cases, ensure_ascii=False, separators=(",", ":"))
    return f'''<div id="d1FailureTrajectories">
  <style>
    #d1FailureTrajectories {{
      --d1-fg: var(--foreground, #e7edf3);
      --d1-muted: var(--muted-foreground, #a9b5c1);
      --d1-grid: var(--border, #3b4753);
      --d1-bg: var(--background, #141414);
      --d1-blue: var(--viz-series-1, #65b5f6);
      --d1-red: #ff5c67;
      --d1-orange: #f3ac5c;
      --d1-green: #63d59a;
      color: var(--d1-fg);
      font-family: system-ui, -apple-system, sans-serif;
      width: 100%;
    }}
    #d1FailureTrajectories .d1-controls {{
      display: flex;
      align-items: center;
      gap: 0.75rem;
      flex-wrap: wrap;
      margin-bottom: 0.5rem;
      color: var(--d1-fg);
    }}
    #d1FailureTrajectories select {{
      color: var(--d1-fg);
      background: var(--d1-bg);
      border: 1px solid var(--d1-grid);
      border-radius: 0.35rem;
      padding: 0.25rem 0.45rem;
    }}
    #d1FailureTrajectories .d1-plot {{
      width: 100%;
      aspect-ratio: 1.9;
      min-height: 330px;
      max-height: 500px;
      background: var(--d1-bg);
      border: 1px solid var(--d1-grid);
      border-radius: 0.5rem;
      overflow: hidden;
    }}
    #d1FailureTrajectories svg {{ width: 100%; height: 100%; display: block; }}
    #d1FailureTrajectories .d1-detail {{
      color: var(--d1-muted);
      margin-top: 0.45rem;
    }}
  </style>
  <div class="d1-controls">
    <label for="d1FailureCase">查看案例</label>
    <select id="d1FailureCase"></select>
    <span>綠：success　藍：recoverable timeout　紅：deadlock　橘：障礙物　虛線：脫困作用區間</span>
  </div>
  <div id="d1FailurePlot" class="d1-plot" role="img" aria-label="D1 failure replay top-view trajectory"></div>
  <div id="d1FailureDetail" class="d1-detail"></div>
  <script>
    (() => {{
      const root = document.getElementById('d1FailureTrajectories');
      const cases = {data};
      const select = root.querySelector('#d1FailureCase');
      const plot = root.querySelector('#d1FailurePlot');
      const detail = root.querySelector('#d1FailureDetail');
      const svgNS = 'http://www.w3.org/2000/svg';
      let selected = 0;
      cases.forEach((item, index) => {{
        const option = document.createElement('option');
        option.value = String(index);
        option.textContent = `trial ${{item.trial}} · ${{item.status}}`;
        select.appendChild(option);
      }});
      const make = (tag, attrs = {{}}) => {{
        const node = document.createElementNS(svgNS, tag);
        Object.entries(attrs).forEach(([key, value]) => node.setAttribute(key, String(value)));
        return node;
      }};
      function render() {{
        if (!cases.length) return;
        selected = Number(select.value || 0);
        const item = cases[selected];
        const W = 920; const H = 480;
        const pad = {{left: 58, right: 18, top: 18, bottom: 38}};
        const all = [];
        cases.forEach(other => other.trace.forEach(point => {{
          all.push(point.r); point.o.forEach(obstacle => all.push(obstacle));
        }}));
        cases.forEach(other => all.push(other.goal));
        const xs = all.map(point => point[0]); const ys = all.map(point => point[1]);
        const xmin = Math.min(...xs) - 0.35; const xmax = Math.max(...xs) + 0.35;
        const ymin = Math.min(...ys) - 0.35; const ymax = Math.max(...ys) + 0.35;
        const sx = (W - pad.left - pad.right) / (xmax - xmin);
        const sy = (H - pad.top - pad.bottom) / (ymax - ymin);
        const map = point => [pad.left + (point[0] - xmin) * sx, H - pad.bottom - (point[1] - ymin) * sy];
        const svg = make('svg', {{viewBox: `0 0 ${{W}} ${{H}}`, preserveAspectRatio: 'none'}});
        svg.appendChild(make('rect', {{x: 0, y: 0, width: W, height: H, fill: 'var(--d1-bg)'}}));
        for (let x = Math.ceil(xmin); x <= Math.floor(xmax); x += 1) {{
          const a = map([x, ymin]); const b = map([x, ymax]);
          svg.appendChild(make('line', {{x1: a[0], y1: a[1], x2: b[0], y2: b[1], stroke: 'var(--d1-grid)'}}));
          const label = make('text', {{x: a[0], y: H - 12, fill: 'var(--d1-muted)', 'font-size': 12, 'text-anchor': 'middle'}});
          label.textContent = `${{x}} m`; svg.appendChild(label);
        }}
        for (let y = Math.ceil(ymin); y <= Math.floor(ymax); y += 1) {{
          const a = map([xmin, y]); const b = map([xmax, y]);
          svg.appendChild(make('line', {{x1: a[0], y1: a[1], x2: b[0], y2: b[1], stroke: 'var(--d1-grid)'}}));
          const label = make('text', {{x: 22, y: a[1] + 4, fill: 'var(--d1-muted)', 'font-size': 12, 'text-anchor': 'middle'}});
          label.textContent = `${{y}}`; svg.appendChild(label);
        }}
        const polyline = (points, attrs) => {{
          if (points.length < 2) return;
          svg.appendChild(make('polyline', {{points: points.map(point => map(point).join(',')).join(' '), fill: 'none', ...attrs}}));
        }};
        cases.forEach((other, index) => {{
          if (index === selected) return;
          polyline(other.trace.map(point => point.r), {{stroke: other.status === 'success' ? 'var(--d1-green)' : 'var(--d1-red)', 'stroke-width': 1.5, opacity: 0.25}});
        }});
        polyline(item.trace.map(point => point.r), {{stroke: item.status === 'success' ? 'var(--d1-green)' : 'var(--d1-red)', 'stroke-width': 4}});
        const obstacleCount = item.trace.reduce((max, point) => Math.max(max, point.o.length), 0);
        for (let obstacleIndex = 0; obstacleIndex < obstacleCount; obstacleIndex += 1) {{
          const path = item.trace.map(point => point.o[obstacleIndex]).filter(point => point && point[2]).map(point => [point[0], point[1]]);
          polyline(path, {{stroke: 'var(--d1-orange)', 'stroke-width': 2.5, opacity: 0.9}});
        }}
        const recovery = item.trace.filter(point => point.a).map(point => point.r);
        polyline(recovery, {{stroke: 'var(--d1-red)', 'stroke-width': 6, 'stroke-dasharray': '7 5', opacity: 0.9}});
        const mark = (point, color, radius, text) => {{
          const p = map(point); svg.appendChild(make('circle', {{cx: p[0], cy: p[1], r: radius, fill: color}}));
          const label = make('text', {{x: p[0] + 8, y: p[1] - 8, fill: 'var(--d1-fg)', 'font-size': 13}});
          label.textContent = text; svg.appendChild(label);
        }};
        mark(item.trace[0].r, 'var(--d1-blue)', 6, 'start');
        mark(item.trace[item.trace.length - 1].r, item.status === 'success' ? 'var(--d1-green)' : 'var(--d1-red)', 6, 'end');
        mark(item.goal, 'var(--d1-fg)', 7, 'goal');
        plot.replaceChildren(svg);
        detail.textContent = `trial ${{item.trial}} · ${{item.status}} · min clearance ${{item.clearance.toFixed(3)}} m · remaining goal distance ${{item.distance.toFixed(3)}} m · path ${{item.path.toFixed(3)}} m · recovery ${{item.recovery ? 'triggered' : 'not triggered'}}`;
      }}
      select.addEventListener('change', render);
      render();
    }})();
  </script>
</div>
'''


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--failure-dir", type=Path, default=Path("output/failures"))
    parser.add_argument("--maximum-trial", type=int, default=20)
    parser.add_argument("--include-success", action="store_true")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    cases = load_cases(args.failure_dir, args.maximum_trial, args.include_success)
    if not cases:
        raise SystemExit("no replay traces found; rerun the benchmark first")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(build_svg_fragment(cases), encoding="utf-8")
    print(f"cases={len(cases)} output={args.output}")


if __name__ == "__main__":
    main()
